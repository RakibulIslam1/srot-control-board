# Hengla vs ArduSub

An honest comparison, written to help decide what to build next rather than to win an argument.
ArduSub is a mature, widely-deployed autopilot with a decade of field use behind it; Hengla is a
focused firmware for one specific vehicle. Each is better at different things, and the second
half of this document matters as much as the first.

Hengla runs on the **SROT** control board: an ESP32 flight core plus an RP2350 thruster
co-processor. ArduSub runs on Pixhawk-class hardware, usually with a Raspberry Pi companion
running BlueOS.

---

## Where Hengla is genuinely better

### 1. High-level motion primitives with an action-shaped contract

`MAV_CMD_SROT_MOVE` takes a verb — forward, back, strafe, turn, dive, arc, hold, style, stop —
and returns an immediate `IN_PROGRESS`, ~3 Hz progress ACKs (0–99), and one terminal
`ACCEPTED`/100. That is exactly the shape of a ROS 2 action, so a companion sends *intent* and
relays a result.

ArduSub has no equivalent. A companion wanting "go forward for 3 seconds" streams
`RC_CHANNELS_OVERRIDE` at ~50 Hz for the duration and implements its own timing, ramping and
stopping in Python. Every one of those messages is a chance for the link to hiccup mid-manoeuvre.

**Consequence:** on Hengla the vehicle keeps executing correctly if the companion stalls for a
second; on ArduSub an RC-override gap is a loss of control. This is the single largest
architectural difference.

### 2. Real per-thruster RPM, and things built on it

Bidirectional DShot with Bluejay-flashed ESCs gives **signed measured RPM for all 8 thrusters**,
streamed as `ESC_STATUS`, with per-thruster stall detection ("Thruster 3 STALLED: dshot=1400
rpm=0").

ArduSub drives PWM ESCs entirely open-loop. It has **no feedback from a thruster at all** — a
fouled prop, a dead ESC or a flooded motor is invisible to the autopilot and is inferred by the
pilot from behaviour.

Built on top of that RPM: the slow thrust-normalisation trim (`THR_TRIM_EN`), which learns a
bounded per-thruster gain so the same command produces the same thrust as the pack drains.

### 3. On-board braking, so timed moves repeat

A translate leg cruises, then actively **brakes** with reverse thrust scaled to the cruise
speed (`MOVE_BRAKE_GAIN`, `MOVE_BRAKE_K`) before reporting complete. ArduSub coasts to a stop on
drag, and how far it coasts depends on speed, trim and current.

Combined with the voltage feedforward, "20% for 5 s" is intended to travel the same distance on
a full or a flat pack. (Both `THR_TRIM_EN` and `MOT_BAT_V_MAX` ship **off** — enable them in
water; see `PARAMETERS.md`.)

### 4. Dual-MCU split keeps the flight loop clean

DShot generation and RPM decoding live on the RP2350's PIO. The ESP32's 500 Hz control loop
never touches bit-timing, and comms work sits on the other core. An ArduSub board generates its
outputs from the same MCU that runs everything else.

### 5. IMU loss degrades instead of disarming

Lose attitude mid-dive and Hengla warns, zeroes the stale rates, holds the last-good attitude
and **stays armed** — for a submarine, cutting the thrusters mid-water is usually worse than
flying on slightly stale data. It also self-heals the BNO085 (re-issues reports, then re-opens
the SH-2 session).

ArduSub's EKF failsafe is more conservative, which is the right call for a vehicle that can fall
out of the sky and the wrong one for a neutrally-buoyant vehicle.

### 6. On-board autotune, including the thrusters

A relay autotune walks the whole cascade (rate roll/pitch/yaw → angle P → depth PID) in about a
minute, safety-monitored, and a separate `MOTOR_TUNE` characterises each thruster individually.
ArduSub has no autotune for a sub.

### 7. Integrated LoRa black-box downlink

A 39-byte CRC-checked frame at ~4 Hz to a second ESP32 that re-emits MAVLink over USB — so
telemetry survives a tether failure with no extra hardware stack. On ArduSub this is a
companion-side integration job.

### 8. Comprehensible

~10k lines you can read in an afternoon, versus ArduPilot's ~700k. When something misbehaves you
can find it. This audit is the demonstration: nine real defects located by reading the code.

---

## Where ArduSub is clearly ahead

This section is not a formality. If a mission needs anything below, ArduSub (or a companion
doing the heavy lifting) is the right answer.

### 1. No state estimator

ArduSub runs **EKF3**: a full 24-state filter fusing IMU, baro, compass, GPS, DVL, optical flow
and range finders, with innovation-based fault detection and lane switching between multiple
IMUs.

Hengla has **no estimator at all**. It consumes the BNO085's internal fusion for attitude and
reads depth straight from the Bar30. There is no velocity estimate, no position estimate, and no
mechanism for detecting that a sensor has gone quietly wrong (as opposed to silent, which *is*
detected).

### 2. No position control or real navigation

ArduSub has `GUIDED`, `AUTO`, `POSHOLD`, `CIRCLE`, waypoint missions with `WPNAV_*` tuning, and
DVL/GPS-driven station keeping.

Hengla's `AUTO` executes **one timed primitive at a time**, dead-reckoned, with no queue.
"Return to the start" is not something it can do. `MISSION_*` upload is handled but minimal.
Anything positional must live on the Jetson.

### 3. No logging for post-dive analysis

ArduSub writes structured dataflash logs at loop rate — every setpoint, every PID term, every
sensor — and the whole ecosystem (`MAVExplorer`, `plotjuggler`, Flight Review) exists to read
them. It is how real tuning problems get diagnosed.

Hengla has Bondor's live charts and CSV recording, which is a GCS-rate sample of selected
signals. **This is the biggest practical gap in the firmware** and is item ④ on the roadmap.

### 4. No sensor redundancy

ArduSub supports multiple IMUs, multiple baros and multiple compasses with voting and failover.
Hengla has one BNO085 and one Bar30 — a single failure of either is a single point of failure,
mitigated only by degrading gracefully.

### 5. No scripting

ArduSub embeds Lua for mission logic, custom failsafes and payload sequencing without a firmware
rebuild. Every behaviour change here is a recompile.

### 6. Field testing

ArduSub has years of use across thousands of vehicles, with the bug reports and edge cases that
implies. Hengla is new, has had nine defects found in one audit pass, and several features
(`THR_TRIM_EN`, `MOT_BAT_V_MAX`, `MAG_YAW_REF`, `TRIM_EN`) are **default-off and not yet
validated in water**. Treat every wet test as a test.

### 7. Ecosystem

QGroundControl's full ArduSub UI, BlueOS's Vehicle Setup pages, parameter metadata with
descriptions and ranges, mature firmware upload tooling. Hengla reports
`MAV_AUTOPILOT_GENERIC`, so those ArduPilot-gated pages stay empty by design — Bondor covers the
same ground for this vehicle, but it is one app, not an ecosystem. See `docs/BLUEOS.md`.

---

## Choosing

| If you need… | Use |
|---|---|
| Companion-driven high-level moves with reliable progress/result | **Hengla** |
| Thruster health, RPM feedback, stall detection | **Hengla** |
| Repeatable timed distances as the pack drains | **Hengla** |
| Waypoint navigation, position hold, DVL/GPS fusion | **ArduSub** |
| Post-dive log analysis for tuning | **ArduSub** |
| Redundant sensors for a vehicle you cannot afford to lose | **ArduSub** |
| A codebase you can fully understand and modify | **Hengla** |

The honest summary: Hengla is better at **being commanded** and at **knowing what its thrusters
are doing**. ArduSub is better at **knowing where it is** and at **surviving things going
wrong**. For an AUV whose intelligence lives on a Jetson, that trade is a reasonable one — the
companion supplies the estimation and planning that Hengla lacks, and gets a much better
actuation interface in return.
