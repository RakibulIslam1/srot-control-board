// =============================================================================
//  SROT FLIGHT CONTROLLER — main.cpp
//  MAVLink-native ArduSub-clone firmware | ESP32 DevKit V1 | dual-core FreeRTOS
//
//  Boot sequence (Phase 0 foundation):
//    1. Strapping-pin safety: force GPIO12/15 LOW and keep the buzzer (GPIO32) quiet.
//    2. Bring up UART0 (MAVLink link) and the two I2C buses.
//    3. Create the SystemState mutexes.
//    4. Launch the six FreeRTOS tasks, pinned per config.h.
//
//  All six tasks are fully implemented (sensors, control, DShot, MAVLink, LoRa/SD,
//  UI). Sensor bring-up (BNO085's 3 s delay) runs inside Task_SensorRead so the
//  MAVLink heartbeat starts within ms of boot.
// =============================================================================

#include <Arduino.h>
#include <Wire.h>
#include <esp_system.h>

#include "config.h"
#include "state_types.h"
#include "tasks/tasks.h"
#include "comms/params.h"
#include "comms/ui_log.h"
#include "control/calibration.h"

// Reset reason captured at boot (shared so the MAVLink task can report it and the
// OLED can show it — the panic backtrace itself goes to UART0/MAVLink and is lost).
const char* g_reset_reason = "?";

// Per-task stack high-water (words remaining, min ever seen). Filled by each task,
// streamed by the MAVLink task. [0]=MAVLink [1]=Sensor [2]=Control. A value near 0
// means that task is about to overflow its stack (a reboot cause).
// Per-task stack high-water (words free, min ever). [0]=MAVLink [1]=Sensor
// [2]=Control [3]=UI [4]=LoRa/SD [5]=DShot.
volatile uint32_t g_stack_hw[6] = {0, 0, 0, 0, 0, 0};

static const char* resetReasonStr(esp_reset_reason_t r) {
    switch (r) {
        case ESP_RST_POWERON:   return "POWERON";
        case ESP_RST_EXT:       return "EXT";
        case ESP_RST_SW:        return "SW";
        case ESP_RST_PANIC:     return "PANIC";      // crash / exception
        case ESP_RST_INT_WDT:   return "INT_WDT";    // interrupt watchdog
        case ESP_RST_TASK_WDT:  return "TASK_WDT";   // task starved idle
        case ESP_RST_WDT:       return "WDT";
        case ESP_RST_BROWNOUT:  return "BROWNOUT";   // supply sag
        case ESP_RST_DEEPSLEEP: return "DEEPSLEEP";
        case ESP_RST_SDIO:      return "SDIO";
        default:                return "UNKNOWN";
    }
}

// Free an I2C bus wedged by a slave that was left holding SDA low across a warm reset
// (EN button / brown-out) — the classic "boots on power-on, freezes on reset" cause.
// Clock SCL until the slave releases SDA, then issue a STOP. Call BEFORE Wire.begin().
static void i2cBusRecover(int sda, int scl) {
    pinMode(scl, OUTPUT); pinMode(sda, INPUT_PULLUP);
    for (int i = 0; i < 9 && digitalRead(sda) == LOW; ++i) {   // clock a stuck slave out
        digitalWrite(scl, LOW);  delayMicroseconds(5);
        digitalWrite(scl, HIGH); delayMicroseconds(5);
    }
    // STOP condition: SDA low→high while SCL is high.
    pinMode(sda, OUTPUT); digitalWrite(sda, LOW);  delayMicroseconds(5);
    digitalWrite(scl, HIGH); delayMicroseconds(5);
    digitalWrite(sda, HIGH); delayMicroseconds(5);
    pinMode(sda, INPUT); pinMode(scl, INPUT);      // release; Wire.begin() reconfigures them
}

// The single global shared state (declared extern in state_types.h).
SystemState g_state;

// Second I2C bus for display + PCA9685 aux expander.
TwoWire Wire1Bus = TwoWire(1);

void setup() {
    // ---- 0. Capture WHY we (re)booted, before anything can hide it ---------
    g_reset_reason = resetReasonStr(esp_reset_reason());

    // ---- 1. Strapping-pin safety FIRST -------------------------------------
    // GPIO12 (MTDI, flash-voltage select) and GPIO15 (MTDO) are strapping pins.
    // In the default PICO backend they are FREE expansion pins; in the RMT fallback
    // they are DShot outputs THR2/THR5. Either way, force a defined LOW at boot so a
    // floating input can't be latched HIGH at the next reset (1.8 V flash select /
    // brown-out). Buzzer (GPIO32) is forced LOW so it can't chirp. (GPIO0 = BOOT strap
    // is left to its internal pull-up; never driven LOW here.)
    pinMode(PIN_THR2, OUTPUT); digitalWrite(PIN_THR2, LOW);   // GPIO12 = MTDI strap
    pinMode(PIN_THR5, OUTPUT); digitalWrite(PIN_THR5, LOW);   // GPIO15 = MTDO strap
    pinMode(PIN_BUZZER, OUTPUT); digitalWrite(PIN_BUZZER, LOW);

    // ---- 2. Serial / MAVLink link ------------------------------------------
    // UART0 carries MAVLink. Enlarge the RX/TX buffers (default 256 B) so QGC's
    // bursty param download / command traffic can't overflow and desync the
    // stream. Must be set before begin().
    MAVLINK_SERIAL.setRxBufferSize(1024);
    MAVLINK_SERIAL.setTxBufferSize(1024);
    MAVLINK_SERIAL.begin(MAVLINK_BAUD);

#if SROT_DEBUG
    // Bench bring-up only (no GCS attached): confirm identity on the wire.
    delay(200);
    MAVLINK_SERIAL.println();
    MAVLINK_SERIAL.println(SROT_FW_VERSION_STR "  (" SROT_BUILD_DATE ")");
    MAVLINK_SERIAL.println("SROT foundation boot — dual-core, 6 tasks");
#endif

    // ---- 3. I2C buses ------------------------------------------------------
    // Bus0: navigation sensors (BNO085 + Bar30). Leave the I2C timeout at the
    // Arduino default (50 ms) — the BNO085's initial SHTP advertisement is a large
    // multi-hundred-byte read, and a shortened timeout aborts it (begin_I2C fails →
    // no IMU data at all).
    // Recover either bus if a slave was left holding SDA low across a warm reset
    // (else begin_I2C/first read wedges → the "EN-reset freeze"). No-op on a clean bus.
    i2cBusRecover(PIN_I2C0_SDA, PIN_I2C0_SCL);
    i2cBusRecover(PIN_I2C1_SDA, PIN_I2C1_SCL);
    Wire.begin(PIN_I2C0_SDA, PIN_I2C0_SCL, I2C0_FREQ);
    // Bus1: OLED + PCA9685 expander, standard speed.
    Wire1Bus.begin(PIN_I2C1_SDA, PIN_I2C1_SCL, I2C1_FREQ);

    // ---- 4. Shared-state mutexes -------------------------------------------
    g_state.initMutexes();

    // ---- 4b. Parameters (NVS) + calibration -------------------------------
    // NOTE: sensor bring-up (BNO085 has a 3 s cold-boot delay) is done INSIDE
    // Task_SensorRead, NOT here — so the MAVLink heartbeat starts within ms of
    // boot and QGC connects immediately (critical: opening the USB port resets
    // the board via DTR→EN).
    params::init();
    calibration::loadFromNVS();

    // Surface the boot reason on the OLED log ONLY for a fault reset (BROWNOUT/TASK_WDT/
    // PANIC). A normal POWERON/SW boot is not shown — else the stale "Boot: POWERON" line
    // lingers forever and looks like an ongoing reset.
    { esp_reset_reason_t r = esp_reset_reason();
      if (r != ESP_RST_POWERON && r != ESP_RST_SW) {
          static char bootmsg[32]; snprintf(bootmsg, sizeof(bootmsg), "Boot: %s", g_reset_reason);
          ui_log::set(bootmsg);
      } }

    // ---- 5. Launch tasks (pinned per config.h) -----------------------------
    // Core 0 — comms / display / filesystem
    xTaskCreatePinnedToCore(Task_MAVLink,   "MAVLink",   TASK_MAVLINK_STACK, nullptr,
                            TASK_MAVLINK_PRIO,   nullptr, TASK_MAVLINK_CORE);
    xTaskCreatePinnedToCore(Task_LoRa_SD,   "LoRa_SD",   TASK_LORA_SD_STACK, nullptr,
                            TASK_LORA_SD_PRIO,   nullptr, TASK_LORA_SD_CORE);
    xTaskCreatePinnedToCore(Task_UI_Status, "UI_Status", TASK_UI_STACK,      nullptr,
                            TASK_UI_PRIO,        nullptr, TASK_UI_CORE);
    xTaskCreatePinnedToCore(Task_Buzzer,    "Buzzer",    TASK_BUZZER_STACK,  nullptr,
                            TASK_BUZZER_PRIO,    nullptr, TASK_BUZZER_CORE);

    // Core 1 — deterministic real-time flight loop
    xTaskCreatePinnedToCore(Task_SensorRead,  "SensorRead",  TASK_SENSOR_STACK,  nullptr,
                            TASK_SENSOR_PRIO,  nullptr, TASK_SENSOR_CORE);
    xTaskCreatePinnedToCore(Task_ControlLoop, "ControlLoop", TASK_CONTROL_STACK, nullptr,
                            TASK_CONTROL_PRIO, nullptr, TASK_CONTROL_CORE);
    xTaskCreatePinnedToCore(Task_DShot_RMT,   "DShot_RMT",   TASK_DSHOT_STACK,   nullptr,
                            TASK_DSHOT_PRIO,   nullptr, TASK_DSHOT_CORE);
}

void loop() {
    // All work happens in the pinned tasks. The Arduino loop task idles.
    vTaskDelay(pdMS_TO_TICKS(1000));
}
