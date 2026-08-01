# SROT — Hardware & Pin Map

The complete, unambiguous wiring reference: **every pin and what it connects to**, for the
two-board setup — **ESP32 DevKit V1** (flight controller) + **Raspberry Pi Pico 2 / RP2350**
(thruster + RPM co-processor). Default `THRUSTER_BACKEND = PICO`; the standalone RMT fallback
is noted at the end.

---

## System block diagram

```
              ┌───────────── Main battery (3S–4S LiPo) ─────────────┐
              │ V+                                             GND   │
              ├───────────────┬──────────────────┐                  │
              ▼               ▼                   ▼                  │
        ┌───────────┐   ┌──────────┐        ┌──────────┐            │
        │ 8× ESC PWR│   │  5V BEC  │        │  5V BEC  │            │
        └─────┬─────┘   │ →ESP32   │        │ →Pico    │            │
              │ sig×8   │  VIN/5V  │        │  VSYS    │            │
              │         └────┬─────┘        └────┬─────┘            │
   ┌──────────┴──────────┐   │                   │                 │
   │  Pico 2 (RP2350)    │   │            ┌───────┴────────┐        │
   │ GP6..GP13 →ESC1..8  │   └───────────▶│     ESP32      │        │
   │  (pull-up optional) │                │                │        │
   │ GP1 ←ESP32 GPIO17   │◀── UART2 ─────▶│ GPIO17→ GP1    │        │
   │ GP0 →ESP32 GPIO16   │   1 Mbaud      │ GPIO16← GP0    │        │
   │ GP15←ESP32 GPIO27   │◀── e-stop ─────│ GPIO27→ GP15   │        │
   │ core0 DShot+RPM PI  │                │  ┌─ sensors ─┐ │        │
   │ core1 UART link     │                │  I2C0 21/22   │        │
   │ USB = debug         │                │  I2C1 25/26   │        │
   └─────────┬───────────┘                │  VSPI 18/19/23│        │
             │ GND                         │  UART0 1/3 →──┼─▶ BlueOS/QGC
             └─────────────────────────────┤  ADC 35/36/39 │        │
                                           └───────┬───────┘        │
      COMMON GROUND: battery−, all ESCs,           │ GND            │
      ESP32 GND, Pico GND  ────────────────────────┴────────────────┘
```

---

## ESP32 DevKit V1 — every used GPIO

| ESP32 GPIO | Direction | Connects to | Function / notes |
|---|---|---|---|
| **1** (U0TXD) | out | BlueOS/QGC RX (tether) | UART0 MAVLink TX |
| **3** (U0RXD) | in | BlueOS/QGC TX (tether) | UART0 MAVLink RX |
| **21** | I/O | BNO085 SDA **+** MS5837 SDA | I2C0 SDA (400 kHz) |
| **22** | I/O | BNO085 SCL **+** MS5837 SCL | I2C0 SCL |
| **4** | in | BNO085 INT (H_INTN) | INT-gated IMU read; normal (non-strapping) GPIO |
| **25** | I/O | OLED SDA **+** PCA9685 SDA | I2C1 SDA (100 kHz) |
| **26** | I/O | OLED SCL **+** PCA9685 SCL | I2C1 SCL |
| **18** | out | LoRa SCK **+** SD SCK | VSPI SCK (shared) |
| **19** | in | LoRa MISO **+** SD MISO | VSPI MISO |
| **23** | out | LoRa MOSI **+** SD MOSI | VSPI MOSI |
| **5** | out | LoRa NSS/CS | LoRa chip-select |
| **34** | in | LoRa DIO0 | LoRa IRQ (input-only pin) |
| — | — | LoRa RST → **ESP32 EN** | RST tied to EN (no GPIO; `PIN_LORA_RST=-1`) |
| **33** | out | SD card CS | SD chip-select |
| **17** (U2TXD) | out | **Pico GP1** | UART2 TX → Pico link |
| **16** (U2RXD) | in | **Pico GP0** | UART2 RX ← Pico link |
| **27** | out | **Pico GP15** | Pico e-stop (HIGH = stop) |
| **2** | out | WS2812B RGB DIN | NeoPixel bit-bang; strapping (DIN high-Z at boot OK) |
| **32** | out | Buzzer via 2N2222 base | LEDC tone (see buzzer note below) |
| **35** | in | Leak sensor (biased) | ADC1, input-only |
| **36** (VP) | in | Battery-voltage divider | ADC1 (PM1 when `PM1_SRC=1`) |
| **39** (VN) | in | Battery-current sensor | ADC1 → BATTERY_STATUS current |
| 0, 12, 13, 14, 15 | — | **FREE (expansion)** | see "Expansion / free GPIO" below |

**Power:** ESP32 `VIN`/`5V` ← 5 V BEC; `GND` ← common ground.

### Expansion / free GPIO (add-on sensors)
In the default **PICO backend the ESP32 drives no thruster pins**, so these are free:
| Pin(s) | Notes |
|---|---|
| **GPIO13, GPIO14** | fully free, unrestricted digital I/O |
| **GPIO0** | free but BOOT strap — must be HIGH at boot (has internal pull-up); fine as an output/​input after boot |
| **GPIO12, GPIO15** | free but strapping (MTDI/MTDO) — keep LOW/hi-Z at boot; firmware forces them LOW at startup |
| **I2C0 (21/22)**, **I2C1 (25/26)** | shared buses — just add more I2C sensors (unique addresses) |
| **VSPI (18/19/23)** | shared SPI — add devices with a spare CS on any free GPIO |

*(If you switch to `THRUSTER_BACKEND=RMT`, GPIO 0/12/13/14/15 become DShot outputs and are no longer free.)*

### Buzzer (GPIO32 → 2N2222 low-side driver)
Wiring: **GPIO32 → 2N2222 base**, buzzer **+ to 5 V**, buzzer **− to the transistor collector**,
**emitter → GND** (the transistor switches the buzzer's ground). Recommended: a **~1 kΩ resistor
in series with the base** — a bare 3.3 V GPIO drives the base too hard without it. Firmware plays
tones in the loud ~2.5–3.2 kHz band; a dedicated 200 Hz buzzer task keeps beep lengths uniform.

**Idle QGC-disconnect while the log shows "Pico link restored":** if QGC drops and the Pico link
flaps **but the OLED keeps running** (no boot splash), the ESP32 did **not** reset — it's a **5 V
rail dip** that reset the Pico (its VSYS) and glitched the DevKit's **USB-UART bridge** (→ QGC
disconnects), while the ESP32's own 3.3 V held up. Firmware now rides through brief blips
(`PICO_LINK_TIMEOUT_MS = 500`), but the cure is electrical: a **BEC with current headroom**, thick
**5 V + common (star) ground** wiring, a **bulk cap ≥470 µF on the 5 V rail** and **100 µF on the
Pico VSYS**, and keep the buzzer/thruster current off the logic-5 V run.

**EN-button (or brown-out) reset used to freeze the board:** on a warm reset the powered I2C
slaves could be left holding SDA low, wedging the bus. Firmware now **recovers both I2C buses**
(clocks them free) before init, so warm resets boot cleanly. If a freeze still happens, it's
strapping/power (hardware) — report it.

**If you see `rst:poweron` reboots + the startup tune replaying (sometimes cut short):** the
buzzer's current spike is sagging the 3.3 V so deep it reads as a power-on reset, and the boot
melody's own current re-triggers it (a reset loop). Firmware now drives the buzzer at a **lower
duty** (`BUZZ_DUTY`, default 30 %) to cut the current, but the real cure is a **~100 Ω resistor in
series with the buzzer** (limits the peak the 2N2222 pulls from 5 V) + a **≥470 µF cap on 5 V**.
With those you can raise `BUZZ_DUTY` back up. **To confirm it's the buzzer:** set `BUZZ_MASK = 0`
(silent) — if the reboots stop, it's the buzzer/power; if not, it's another power fault (BEC/wiring).

**If the buzzer blanks the OLED / resets the board:** the buzzer's 5 V current pulse can dip the
rail. Two effects: (a) it can glitch the SH1106 — firmware now silently re-enables the panel
(non-destructive keep-alive, no visible flicker); (b) a deeper dip **browns out the whole ESP32**
(you'll see the startup beep replay + splash "refresh"). The board shows the **reset reason on the
OLED boot line** (`Boot: BROWNOUT` / `POWERON` / `TASK_WDT` / `PANIC`) — if it reads `BROWNOUT`,
it's electrical. Cure: a **100 µF cap across the buzzer's 5 V** (and/or 0.1 µF across the buzzer),
keep the **OLED on a clean 5 V** run, and a ~1 kΩ base resistor. The startup melody is delayed
~900 ms so it isn't sounding during display init.

## Raspberry Pi Pico 2 (RP2350) — every used GP

| Pico GP | Direction | Connects to | Function |
|---|---|---|---|
| **GP0** (UART0 TX) | out | **ESP32 GPIO16** | `Serial1` TX → ESP32 (telemetry) |
| **GP1** (UART0 RX) | in | **ESP32 GPIO17** | `Serial1` RX ← ESP32 (commands) |
| **GP6** | I/O | ESC 1 signal | bidir DShot (PIO0) |
| **GP7** | I/O | ESC 2 signal | bidir DShot (PIO0) |
| **GP8** | I/O | ESC 3 signal | bidir DShot (PIO0) |
| **GP9** | I/O | ESC 4 signal | bidir DShot (PIO0) |
| **GP10** | I/O | ESC 5 signal | bidir DShot (PIO1) |
| **GP11** | I/O | ESC 6 signal | bidir DShot (PIO1) |
| **GP12** | I/O | ESC 7 signal | bidir DShot (PIO1) |
| **GP13** | I/O | ESC 8 signal | bidir DShot (PIO1) |
| **GP15** | in | **ESP32 GPIO27** | e-stop input (HIGH = stop) |
| **GP25** (LED_BUILTIN) | out | onboard LED | power + status indicator |
| USB | — | dev PC (bench) | debug console (`Serial`) + UF2 flashing |

**Onboard LED status** (the Pico 2 has no power LED, so GP25 shows both "powered" and state).
On at boot = powered. Then:

| LED pattern | Meaning |
|---|---|
| **Solid ON** | Link up **and armed** — thrusters live |
| **Heartbeat** (brief blink ~1 Hz) | Link up, **disarmed** — normal resting/idle state |
| **Slow blink** (~0.5 Hz) | **No link** from the ESP32 (check the UART cable / power) |
| **Fast blink** (~5 Hz) | A **thruster fault** is latched (present but not spinning) |

So heartbeat = "healthy, waiting", solid = "armed & driving", fast blink = "a motor problem".

**Power:** Pico `VSYS` ← 5 V BEC (or USB on the bench); `GND` ← common ground.

## ESC signal lines (Pico → 8 ESCs)
- `Pico GP6..GP13` → `ESC1..ESC8` **signal** pad (one wire each). ESC **power** = main
  battery; each ESC **GND** → common ground.
- **One-time ESC config** (BLHeli/AM32 tool): enable **DShot 3D mode** + bidirectional
  telemetry, and set each motor's spin direction. Reverse depends on 3D mode.

### About the "pull-up" (you asked — here's the plain version)
Bidirectional DShot sends telemetry **back on the same signal wire**: the Pico drives the
wire for the command, then briefly releases it and the ESC pulses it to report RPM. A
**pull-up resistor holds the wire HIGH in between**, so those pulses are clean.

- **For bench testing / short wires: you very likely need NOTHING extra** — the RP2350's
  internal pull-ups and short leads are usually enough. Just connect `GPn → ESC signal`.
- **If RPM readings are missing or jittery** (longer/noisier wiring in the sub), add **one
  resistor of ~1–4.7 kΩ (2.2 kΩ is a good middle) per signal line, from that wire to the
  Pico's 3.3 V pin.** One resistor per ESC = 8 total. That's the whole thing.
- Optional: a 33–100 Ω series resistor at each Pico pin reduces ringing. Keep leads short.

### Thruster "not detected" vs "fault" (QGC messages)
The Pico distinguishes two cases from the bidirectional-DShot telemetry:
- **"Thruster N not detected"** (WARNING) — that channel returns **no telemetry at all**: the
  ESC is unplugged, **or** it's connected but doesn't have **bidirectional DShot enabled**. (A
  bare bench with no ESCs reads *not detected*, not *fault*.)
- **"Thruster N fault"** (ERROR) — the ESC **is** present (sending telemetry) but isn't spinning
  while commanded. This is a real motor/prop/ESC problem.

Seeing either message confirms the **ESP32↔Pico link is working**. "Not detected" is only
reported while armed, once per arm session (not spammy).

### ESC beacon beep (Pixhawk-style)
On power-up and on the first arm, the Pico plays a short **beacon chirp through the ESCs**
(DShot beacon command — BLHeli's 5 fixed tones, not a melody). Disable with `THR_BEEP_EN=0`.
Only fires while disarmed.

### Buzzer tones (GPIO32) and the `BUZZ_MASK` toggle
Melodies/alerts play on the buzzer. Each situation can be turned off via the **`BUZZ_MASK`**
bitmask param (255 = all on, 0 = silent):

| Bit (value) | Situation | Tone |
|---|---|---|
| 0 (1) | Startup | Rick & Morty theme riff |
| 1 (2) | Arm | rising chirp |
| 2 (4) | Disarm | falling chirp |
| 3 (8) | Pico link lost / restored | down-sweep / up-chirp warble |
| 4 (16) | GCS link lost | two-tone warble |
| 5 (32) | Leak detected | urgent wave triple (repeats) |
| 6 (64) | Thruster fault | (reserved) |
| 7 (128) | Calibration step | short tick |

Example: `BUZZ_MASK = 254` keeps everything but the startup tune; `BUZZ_MASK = 0` is silent.

## Power & ground (critical)
- Main battery → 8 ESCs **and** a 5 V BEC.
- 5 V BEC → ESP32 `VIN/5V` **and** Pico `VSYS`. Each board makes its own 3.3 V. Bench: Pico via USB.
- **Common ground is mandatory:** battery−, every ESC GND, ESP32 GND, Pico GND all tied.
- Both MCUs are 3.3 V logic → UART/e-stop connect directly (no level shifter).

---

## Bus device addresses
| Bus | Pins | Speed | Devices |
|---|---|---|---|
| I2C0 (Wire) | 21/22 | 400 kHz | BNO085 `0x4A` (alt `0x4B`), MS5837 Bar30 `0x76` |
| I2C1 (Wire1) | 25/26 | 100 kHz | SH1106 OLED `0x3C`, PCA9685 `0x40` |
| VSPI | 18/19/23 | — | LoRa (SX127x, CS 5, DIO0 34), SD card (CS 33) |
| UART0 | 1/3 | 115200 | MAVLink ↔ BlueOS/QGC |
| UART2 | 17/16 | 1 Mbaud | ESP32 ↔ Pico thruster link |

## PCA9685 aux channels (16, on I2C1)
Each channel's role is runtime via the **`SERVOn_ROLE`** parameter: `0` = off, `1` = PWM
servo (µs; `SERVOn_MIN/MAX/TRIM`), `2` = MOSFET/relay ON-OFF. Default: ch1-8 servo, ch9-16
switch. See `PARAMETERS.md`.

---

## Thruster-voltage link (2nd board → SROT, ESP-NOW) — bring-up

The thruster pack and the electronics pack are **different batteries**. The SROT board's own ADC
(GPIO36) measures the *electronics/SBC* pack, so the thruster-pack voltage can only come from the
board physically wired to it: the **2nd board** (`env:second-board`), which broadcasts it over
ESP-NOW at **4 Hz**. Wire format: [`shared/espnow_proto.h`](shared/espnow_proto.h) — 6 bytes,
magic `0x53`, `thruster_kill`, `aux_voltage`.

Two features on the control board consume it, and **both are inert without it**:
the mixer's voltage feedforward (`mixer::setBatteryVoltage()`, what makes a timed move travel the
same distance on a full or a flat pack) and the low-thruster-battery failsafe.

### Steps

1. Flash the 2nd board: `pio run -e second-board -t upload -t monitor`. Its serial line should
   read e.g. `THR 15.10 V | knob 253 deg | power ON | tx 131 fail 0` — a rising `tx` with
   `fail 0` means it is broadcasting.
2. **Verify the voltage against a multimeter at the pack.** `PM1_VOLT_MULT` (0.009088) is
   inherited, and the ESP32 ADC is markedly non-linear above ~2.5 V. Every threshold below
   depends on this being right — if the meter disagrees, scale `PM1_VOLT_MULT` by
   `actual / reported` and reflash.
3. On the control board, set these parameters (Bondor → Parameters). Values shown are for a
   **4S LiPo**:

| Param | Set to | Why |
|---|---|---|
| `ESPNOW_EN` | **1** | Starts WiFi STA + ESP-NOW. Live within ~50 ms — `Task_LoRa_SD` polls the param at 20 Hz. No reboot needed. |
| `MOT_BAT_V_MAX` | **16.8** | 4.2 V/cell × 4 = full charge. **0 disables the feedforward entirely**, so this is the switch that turns compensation on. |
| `MOT_BAT_V_MIN` | 13.2 *(default)* | 3.3 V/cell clamp floor, so the compensation gain cannot blow up near empty. |
| `FS_BAT_VOLTAGE` | **13.6** | 3.4 V/cell. Above `MOT_BAT_V_MIN` on purpose — the vehicle should be surfacing before the compensation saturates at its floor. |
| `FS_BAT_ENABLE` | 1 *(default)* | See the warning below. |
| `PM2_SRC` | 2 *(default)* | Already routes the aux voltage to `BATTERY_STATUS` id 1. |

4. Save to flash, then **re-export your `.params` backup** so the new values are captured.

### Where it shows up (within ~2 s)

Bondor Dive view → **Battery 2** tile (`—` until the link is up) · the OLED aux-voltage field ·
`BATTERY_STATUS` **id 1** (id 0 is the electronics pack) · a `STATUSTEXT`
`"Thruster pack link up (15.1 V)"`.

On loss you get `"Thruster pack link LOST - voltage comp off"` within `ESPNOW_STALE_MS` (2 s),
Battery 2 returns to `—`, and the compensation and failsafe both go inert. That is fail-safe —
`thr_volts = 0` means "no source", never "flat battery" — but it is announced because otherwise
the compensation you tuned around switches itself off silently.

> ⚠️ **Enabling this makes the low-battery failsafe live for the first time.** `thr_volts` was
> always 0 before the 2nd board's sender existed, so the `> 1.0 V` guard kept the whole branch
> inert. Once real voltage arrives, the vehicle **will** auto-surface when the pack drains past
> `FS_BAT_VOLTAGE`. It is debounced by `FS_BAT_HOLD_MS` (3 s continuous) precisely because a 4S
> LiPo driving eight T200s sags over a volt under load and an instantaneous test would surface
> you mid-burst. Set `FS_BAT_ENABLE = 0` if you want to fly without it while you calibrate.

### Notes

- Both boards must be on **`ESPNOW_CHANNEL` 1**, with WiFi power-save off (both do this natively;
  the Arduino API cannot pin a channel without an AP association).
- The receiver registers a plain RX callback with **no peer**, so the sender broadcasts to
  `FF:FF:FF:FF:FF:FF`. No pairing or MAC configuration is needed.
- `fail 0` on the 2nd board only means the MAC accepted the frame for transmission — broadcast is
  unacknowledged, so it is **not** proof anything received it. The control board's `STATUSTEXT`
  is the proof.
- `ESPNOW_EN` is a one-shot latch in `Task_LoRa_SD`: setting it back to **0 does not stop the
  radio**, that needs a reboot. Turning it on is live.

---

## Standalone fallback (`THRUSTER_BACKEND = RMT`)
With no Pico, set `THRUSTER_BACKEND = RMT` in `config.h`: the ESP32 drives DShot directly on
its 8 thruster pins **4, 12, 13, 14, 15, 16, 17, 27** via RMT (no RPM feedback). In that mode
GPIO 16/17/27 are DShot outputs, **not** the Pico link. (GPIO12/15 are strapping pins forced
LOW at boot in `main.cpp` before the DShot task.)
