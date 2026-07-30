# Hengla — AUV/ROV flight-control firmware for the SROT board

**The board is SROT. The firmware is Hengla.**

MAVLink-native flight control on an **ESP32 DevKit V1** (dual-core FreeRTOS, 500 Hz control
loop, 8 vectored thrusters, BNO085 IMU + Bar30 depth), plus a **Raspberry Pi Pico 2 (RP2350)**
co-processor driving the ESCs with bidirectional DShot for real per-thruster RPM.

Hengla is **not** ArduSub and does not identify as it — the heartbeat reports
`MAV_AUTOPILOT_GENERIC` on a submarine frame. What it adds over ArduSub is a companion-computer
command set (`MAV_CMD_SROT_MOVE`) shaped like a ROS 2 action: the Jetson sends *intent*
("forward 3 m", "yaw 90°", "hold 10 s") and the board owns attitude, depth, mixing, braking and
failsafes. What it lacks is a state estimator, position control and replay logging — both sides
of that trade are written up in **[docs/VS_ARDUSUB.md](docs/VS_ARDUSUB.md)**.

Ground station is **Bondor**. BlueOS works for routing, MAVLink2Rest and video, but its
ArduPilot-gated setup pages stay empty by design — see **[docs/BLUEOS.md](docs/BLUEOS.md)**.

Part of the **SROT** control-board suite:

| Repo | What it is |
|---|---|
| **srot-control-board** *(this repo)* | Hengla firmware: ESP32 flight controller, RP2350 thruster co-processor, 2nd board |
| [srot-ground-station](https://github.com/RakibulIslam1/srot-ground-station) | Bondor desktop GCS + ESP32-C3 LoRa bridge |
| [srot-esc-flasher](https://github.com/RakibulIslam1/srot-esc-flasher) | BLHeli 4-way interface — flash Bluejay onto the ESCs |

> **Flash Bluejay first.** Bidirectional DShot — and therefore every RPM-based feature here
> (thruster-health detection, stall detect, thrust trim) — needs **Bluejay**, not stock
> BLHeli_S. Use `srot-esc-flasher` before you fly this.

## Project structure — one project, three MCU targets
```
srot-control-board/
├── platformio.ini            three envs (see below)
├── partitions_hengla.csv     custom partition table (56 KB NVS — see below)
├── src/                       ESP32 flight-controller firmware (Hengla)
│   ├── main.cpp, comms/, control/, drivers/, tasks/
│   ├── pico/main.cpp          RP2350 thruster + RPM co-processor
│   └── second_board/          2nd board: thruster power switch + ESP-NOW voltage TX
├── include/                   config.h, state_types.h
├── shared/                    thruster_link_proto.h · espnow_proto.h · lora_telem_proto.h
├── lib/mavlink/               vendored MAVLink v2 (see its README)
└── docs/ + root docs …
```

## Build & flash
Uses [PlatformIO](https://platformio.org/). Pick the env in the VSCode status bar, or:

| Target | Env | Flash |
|---|---|---|
| **ESP32** flight controller | `esp32doit-devkit-v1` *(default)* | `pio run -t upload` |
| **Pico 2** thruster co-proc | `pico` | `pio run -e pico -t upload` |
| **2nd board** thruster switch + voltage TX | `second-board` | `pio run -e second-board -t upload` |

First Pico flash: hold **BOOTSEL** while plugging in USB (UF2 bootloader).

The 2nd board is what makes the thruster-pack voltage visible to the flight controller — the
ESP32's own ADC reads the *electronics* pack. Until it is flashed and broadcasting (and
`ESPNOW_EN = 1` on the flight controller), the voltage feedforward and the low-thruster-battery
failsafe have no data and stay inert.

> The ESP32 thruster backend is selectable in `include/config.h` (`THRUSTER_BACKEND`): `PICO`
> (default — gives you RPM) or `RMT` (on-board DShot, no Pico, no RPM).

> **Export your parameters before reflashing.** A plain upload preserves NVS, but a build that
> bumps `PARAM_DEFAULTS_VER` rewrites every parameter from its default. Bondor →
> Parameters → Export (the file also carries the `CAL_*` sensor calibration).

> **This build changes the partition table** (NVS 20 KB → 56 KB — the old one physically could not
> hold the parameter set, which is why saves were failing). Flash it once with
> `pio run -t erase` **then** `pio run -t upload`, and re-import your parameters afterwards.
> Full procedure and rationale: [PARAMETERS.md](PARAMETERS.md) and [AUDIT.md](AUDIT.md) R14.

## Documentation
| Doc | What's in it |
|---|---|
| **[HARDWARE.md](HARDWARE.md)** | Complete pin map (every pin → what it connects to), wiring, buses, ESC/power |
| **[ARCHITECTURE.md](ARCHITECTURE.md)** | Dual-core design, the control loop, flight modes, tuning & calibration |
| **[ALGORITHMS.md](ALGORITHMS.md)** | Every control algorithm in full — the math, step by step |
| **[PARAMETERS.md](PARAMETERS.md)** | Every parameter, defaults, units, and the backup workflow |
| **[JETSON_COMMS.md](JETSON_COMMS.md)** | The companion-computer contract — messages, rates, units, sign conventions |
| **[DUBURI_WS_INTEGRATION.md](DUBURI_WS_INTEGRATION.md)** | Porting the duburi_ws ROS 2 stack onto this board |
| **[AUDIT.md](AUDIT.md)** | Firmware audit: findings, evidence, and what was fixed |
| **[docs/VS_ARDUSUB.md](docs/VS_ARDUSUB.md)** | Honest comparison — including where ArduSub is still ahead |
| **[docs/BLUEOS.md](docs/BLUEOS.md)** | What works with BlueOS, what cannot, and how to configure it |
| **[ROADMAP.md](ROADMAP.md)** | Phases, progress log, and the ranked upgrade list |

## Status
Core firmware complete and audit-hardened — see [AUDIT.md](AUDIT.md) for the nine defects found
and fixed in the last pass. Precision control, the Pico RPM co-processor, autotune, LoRa
telemetry and the companion command set are all shipped.

**Not yet validated in water** (all default-off): `THR_TRIM_EN`, `MOT_BAT_V_MAX`, `MAG_YAW_REF`,
`TRIM_EN`. Treat every wet test as a test — `ROADMAP.md` lists what to check.
