# Hengla vs ArduSub — Why We Choose SROT

**SROT with Hengla is our preferred solution for this project's autonomous AUV missions
and piloted ROV operations.** Its design puts perception and mission decisions on the
high-level compute, keeps real-time vehicle control on the board, and gives both the
ROS workspace and the operator a direct interface to the same vehicle.

The preference is about the complete SROT workflow: mission commands, control-core
isolation, thruster feedback, operator tools and a focused mode set. The sections below
explain the benefit of each choice and the capability available on the current `main`.

## Autonomous missions: send a goal, receive a result

`MAV_CMD_SROT_MOVE` (31000) expresses forward/back, strafe, turn, dive, arc, hold, style
and stop. Hengla owns the motion profile, attitude/depth control and braking, while
the companion handles perception and mission sequencing. Progress arrives through
`COMMAND_ACK IN_PROGRESS` and the command resolves with a terminal result.
Rejections, cancellation and failures must be handled as well as success; see the
[wire contract](../JETSON_COMMS.md).

This is the interface we want for a ROS 2 action client: mission code works with
vehicle intent instead of implementing another attitude/depth controller on Linux.
The companion must still supply a heartbeat at least once per second; the default
link-loss failsafe is 5000 ms. A move does not bypass a failsafe.

ArduSub also supports autonomous missions and companion commands through AUTO and
GUIDED. SROT's specific advantage for this project is its dedicated motion-primitive
contract and the clear board/compute division.
[ArduSub mode documentation](https://ardupilot.org/sub/docs/modes.html).

## ROV operations: select the behaviour the job needs

The everyday controls are direct: `STABILIZE` for attitude-assisted piloting,
`DEPTH_HOLD` for depth-assisted work, `MANUAL` for direct output and `ACRO` for rate
control. `AUTO` executes mission primitives and `SURFACE` handles ascent.

The focused mode set is deliberate. The operator chooses the control behaviour
needed for the task, while the ROS workspace combines primitives into richer missions.
Service modes cover motor direction detection, autotune, motor tune and scripted
manoeuvres. There are 11 enum entries in total; adding mode names is not the measure
of mission capability. See the [complete mode table](../ARCHITECTURE.md).

## Position hold and navigation: a clear integration boundary

The preferred SROT architecture keeps perception, localization and planning on the
high-level compute, with fast control on the board. This gives the ROS workspace
room to develop camera-, DVL- or other sensor-based navigation without moving the
existing attitude/depth loops onto the companion.

**Current implementation:** Hengla holds attitude, heading and depth. It has no
horizontal position/velocity estimator or position controller. `HOLD` commands zero
translation with heading/depth hold; a timed forward leg uses a motion profile, not
measured horizontal displacement. Neither guarantees station keeping in a current.
Closed-loop horizontal position hold needs an implemented, validated feedback path.
The [vision interface](../VISION_API.md) is a specification, not a shipped controller.

ArduSub's POSHOLD and navigation modes require appropriate position/depth inputs.
They are distinct from Hengla's current depth/heading hold.
[ArduSub mode requirements](https://ardupilot.org/sub/docs/modes.html).

## Thruster feedback: make propulsion observable

The RP2350 drives eight Bluejay-flashed ESCs through bidirectional DShot and returns
per-thruster RPM and fault information. Hengla exposes RPM to Bondor and the ROS
workspace, including the companion-compatible `ESC_TELEMETRY_1_TO_4` and
`ESC_TELEMETRY_5_TO_8` messages.

For SROT, this is an integrated propulsion path: configuration, feedback, fault
reporting and operator display are developed together. ArduPilot also supports DShot,
ESC telemetry and bidirectional DShot on supported hardware, so the benefit here is
SROT's integrated implementation rather than exclusive protocol support.
[ArduPilot DShot documentation](https://ardupilot.org/sub/docs/common-dshot-escs.html).

`RPM_LOOP` ships off because a fast RPM setpoint loop is unsuitable for the current
stabilization path. Slow thrust trim and battery feedforward are optional,
default-off features requiring water validation; RPM is not a direct thrust sensor.
See [ALGORITHMS.md](../ALGORITHMS.md) and [PARAMETERS.md](../PARAMETERS.md).

## Motion profiles and braking: keep execution on the board

Timed translation ramps its demand and applies reverse thrust before finishing.
The board owns that timing at the control-loop rate, so Python mission code does
not need to reproduce the ramp and brake sequence.

This provides a useful execution contract for repeatable mission development.
Travel distance still depends on calibration, battery, load and water conditions;
braking is not measured position control. Test compensation features before relying
on them for a particular vehicle.

## Control architecture: give real-time work its own resources

Core 1 runs the ESP32 sensing/control/output tasks at 500 Hz. Core 0 handles
communications and UI, and the RP2350 handles DShot timing and RPM decoding.
The BNO085 supplies fused attitude; the Bar30 supplies depth.

This split is why we choose SROT for this vehicle: communications, perception and
ESC bit timing have explicit owners outside the attitude/depth controller.
Core separation is an architectural benefit; a comparative latency, jitter or
accuracy claim would need measurements on both complete systems.

## Operator tools, tuning and records

Bondor is the primary operator/configuration interface, supported by the separate
LoRa bridge and ESC-flasher repositories. On-board relay autotune and motor tune
serve this hardware directly. Mission logic and payload orchestration can evolve
in the ROS workspace without adding a scripting runtime to the flight controller.

Bondor provides live charts and CSV recording; the firmware also writes selected
flight data to SD at approximately 20 Hz. Full-rate controller replay and expanded
diagnostics remain development work. The hardware currently uses one BNO085 and
one Bar30; software cannot create sensor redundancy that is absent from the board.

## Safety and validation

Safety-monitor disarm takes precedence over failsafe SURFACE, mode execution,
motor-test override and mixing. Leak, low-battery and link-loss handling belong on
the board, alongside input freshness checks and command-result reporting.
The [audit](../AUDIT.md) records fixes and deferred issues; the
[roadmap](../ROADMAP.md) records the remaining implementation and water checks.

These choices make SROT the platform we choose to develop across autonomous and
ROV tasks. Position hold, redundancy and field reliability must be assessed from
implemented hardware, feedback and testing; the architectural preference does not
turn planned capabilities into completed ones.
