# SROT — Control Board Firmware

MAVLink-native flight-controller firmware for a custom AUV/ROV control board: an **ESP32 DevKit V1**
running the full flight stack (dual-core FreeRTOS, 500 Hz control loop, 8 vectored thrusters, BNO085
IMU + Bar30 depth), plus a **Raspberry Pi Pico 2 (RP2350)** co-processor that drives the ESCs with
**bidirectional DShot** and feeds real per-thruster RPM back.

It speaks enough ArduSub to work with **QGroundControl** and **BlueOS**, and adds a
companion-computer command set (`MAV_CMD_SROT_MOVE`) designed for a Jetson to issue *intent* —
"forward 3 m", "yaw 90°", "hold 10 s" — while the board owns attitude, depth, mixing, braking and
failsafes.

Part of the **SROT** control-board suite:

| Repo | What it is |
|---|---|
| **srot-control-board** *(this repo)* | ESP32 flight controller + RP2350 thruster co-processor |
| [srot-ground-station](https://github.com/RakibulIslam1/srot-ground-station) | Bondor desktop GCS + ESP32-C3 LoRa bridge |
| [srot-esc-flasher](https://github.com/RakibulIslam1/srot-esc-flasher) | BLHeli 4-way interface — flash Bluejay onto the ESCs |

> **Flash Bluejay first.** Bidirectional DShot — and therefore every RPM-based feature in this
> firmware (thruster-health detection, stall detect, thrust trim) — needs **Bluejay**, not stock
> BLHeli_S. Use `srot-esc-flasher` before you fly this.

---

## Two MCUs, one PlatformIO project

```
srot-control-board/
├── platformio.ini        two envs: esp32doit-devkit-v1 (default) and pico
├── src/
│   ├── main.cpp          ESP32 flight controller
│   ├── comms/            MAVLink stream, command handlers, params, LoRa, ESP-NOW
│   ├── control/          mixer, PIDs, movement primitives, arming, autotune, thrust trim
│   ├── drivers/          BNO085, MS5837, DShot, thruster link, OLED, RGB, PCA9685
│   ├── tasks/            FreeRTOS tasks (control loop, sensors, comms, telemetry)
│   └── pico/main.cpp     RP2350 co-processor: 8× bidirectional DShot + RPM
├── include/              config.h (pins, backend, defaults), state_types.h
├── shared/               thruster_link_proto.h (ESP32↔Pico), lora_telem_proto.h (↔ground station)
├── lib/mavlink/          vendored MAVLink v2 (common + ardupilotmega) — see its README
└── docs/                 T200_PROFILE.md · THRUSTER_MAP.md · ESC_FLASHING.md
```

## Build & flash

Needs [PlatformIO](https://platformio.org/).

| Board | Build | Flash |
|---|---|---|
| **ESP32** (flight controller) | `pio run` | `pio run -t upload` |
| **Pico 2** (thruster co-proc) | `pio run -e pico` | `pio run -e pico -t upload` |

In the **VSCode PlatformIO** toolbar, pick the env in the status-bar dropdown and hit Upload. First
Pico flash: hold **BOOTSEL** while plugging in USB (UF2 bootloader). The ESP32 is the default env, so
a plain `pio run` builds it.

The ESP32 thruster backend is selectable in [`include/config.h`](include/config.h)
(`THRUSTER_BACKEND`): `PICO` (default — uses the co-processor, gives you RPM) or `RMT` (on-board
DShot, no Pico needed, no RPM).

> **Parameters survive a flash.** They live in ESP32 NVS, and stored values beat the `DEF_*`
> compile-time defaults. Changing a default in `config.h` does **nothing** on a board that has
> already saved that param — bump `PARAM_DEFAULTS_VER` to force a re-seed, or reset params from the
> GCS.

`shared/thruster_link_proto.h` frames the ESP32↔Pico UART. Change it and **both** MCUs must be
reflashed together. `shared/lora_telem_proto.h` is **duplicated** in `srot-ground-station` — see the
banner at the top of that file.

## Documentation

| Doc | What's in it |
|---|---|
| **[HARDWARE.md](HARDWARE.md)** | Complete pin map (every pin → what it connects to), wiring, buses, ESC/power |
| **[ARCHITECTURE.md](ARCHITECTURE.md)** | Dual-core design, the control loop, flight modes, MAVLink/QGC/BlueOS, tuning & calibration |
| **[ALGORITHMS.md](ALGORITHMS.md)** | Every control algorithm in full — the math, step by step |
| **[PARAMETERS.md](PARAMETERS.md)** | Every parameter (real vs QGC-compat), defaults, units |
| **[JETSON_COMMS.md](JETSON_COMMS.md)** | **The companion-computer contract** — every message sent/received, rates, units, failsafes. Program against this. |
| **[DUBURI_WS_INTEGRATION.md](DUBURI_WS_INTEGRATION.md)** | How to port the [duburi_ws](https://github.com/fh1m/duburi_ws) ROS 2 stack off a Pixhawk and onto this board |
| **[ROADMAP.md](ROADMAP.md)** | Phases, current status, progress log, research notes |
| [docs/THRUSTER_MAP.md](docs/THRUSTER_MAP.md) | Which thruster is where, spin direction, and what each one does per axis |
| [docs/T200_PROFILE.md](docs/T200_PROFILE.md) | BlueRobotics T200 characteristics → the ESC settings that suit them |
| [docs/ESC_FLASHING.md](docs/ESC_FLASHING.md) | Flashing Bluejay (the tool itself is in `srot-esc-flasher`) |

## Status

Core firmware complete and audit-hardened. QGC/BlueOS ArduSub compatibility working; Phase 1
precision control shipped; the Pico thruster + RPM co-processor is built and flying. The
companion-computer command set is implemented and documented. See `ROADMAP.md`.

**Known-off-by-default:** `THR_TRIM_EN` and `MOT_BAT_V_MAX` ship disabled — repeatable-distance moves
need the trim enabled *in water*. The ESP-NOW thruster-pack voltage link is not yet implemented, so
voltage feedforward has no live source until it is. Details in `DUBURI_WS_INTEGRATION.md` §Gotchas.
