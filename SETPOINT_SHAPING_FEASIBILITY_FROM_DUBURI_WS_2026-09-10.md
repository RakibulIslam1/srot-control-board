# Jerk-limited setpoint shaping: we measured it, and we are NOT recommending Ruckig

From `duburi_ws` (Mongla), 2026-09-10. Against **rev 14**.

Our own change map said "port Ruckig". We built it for your board before
proposing it, and the measurements say **do something else**. This document is
the evidence, so the decision is yours on numbers rather than on our summary.

---

## What we were trying to buy

`STABILIZE`'s attitude setpoint **steps**; only the AUTO path ramps. A stepped
setpoint asks the PIDs for an impulse, and the mixer answers by saturating —
which, per our other PR, silently discards part of every axis. Shaping the
setpoint before the loop is the standard fix and it is why your own TDR-adjacent
notes call for "polynomial interpolation with consideration for known physical
limits, preventing thruster saturation."

## What we measured, on your board target

Built with PlatformIO for `esp32doit-devkit-v1`, `-O2`, `gnu++17`.

**It compiles**, after two exclusions that are not obvious:

```
src/ruckig/cloud_client.cpp   ->  #include <httplib/httplib.h>     (their paid cloud service)
src/wrapper/python.cpp        ->  #include <nanobind/nanobind.h>   (python bindings)
```

Neither is needed; both are compiled by default because PlatformIO builds every
source in the library.

**Cost, marginal, against a bare Arduino sketch on the same board:**

| | measured |
|---|---|
| Flash | **+416,088 B (406 KiB)** |
| Static RAM | **+10,324 B (10.1 KiB)** |
| `sizeof(Ruckig<1>)` | 2,408 B |
| `sizeof(InputParameter<1>)` / `OutputParameter<1>` | 480 B / 688 B |

Against your real build (rev 14 measures **935,429 B, 29.7 %** of the 3 MB app
partition) that would be **43.0 %**, leaving ~1.7 MB. **It fits.**

**It does not allocate.** We instrumented `operator new` and ran 999
`update()` calls on a host build:

```
ticks         999
ALLOCATIONS   0  (0 bytes)
```

That clears the obvious blocker for a 500 Hz FreeRTOS loop, and it is the one
thing we most expected to fail.

## ⛔ And then the finding that decides it

**Ruckig is `double` throughout, and the ESP32's Xtensa LX6 FPU is
single-precision only.** Every double operation is emulated in software.

This is not inferred — it is what your own toolchain emitted. Symbols
referenced by one hot-path object file
(`position_third_step1.cpp.o`), via `xtensa-esp32-elf-nm`:

```
__adddf3  __subdf3  __muldf3  __divdf3
__eqdf2   __nedf2   __ltdf2   __ledf2   __gtdf2  __gedf2
```

— the complete libgcc soft-double set, and **zero** `*sf*` (single-precision,
hardware-FPU) helpers. One file contains 58 `double` uses.

So the paper's headline number — **19.8 µs for 7 DoF** — was measured on a
desktop CPU with a hardware double FPU. **It cannot transfer**, and we are not
going to pretend a scaling factor for it.

⚠ **We could not measure the on-target runtime.** That needs a flashed board,
and the only SROT board we have is the one flying the vehicle — we are not
flashing your flight firmware to benchmark a library. If you have a bare ESP32
on the bench, the probe is ~30 lines and we will send it.

## What we recommend instead, and why it is better founded

**ArduPilot's `SCurve`** (`libraries/AP_Math/SCurve.{h,cpp}`) does exactly this
job on flight-controller hardware:

| | Ruckig | ArduPilot `SCurve` |
|---|---|---|
| arithmetic | **double** — soft-emulated here | **float** — hardware FPU |
| flash | **+406 KiB** measured | fixed 23-segment table, scalars |
| allocation | none (verified) | fixed arrays |
| MCU precedent | **none found** | every ArduPilot autopilot — **including an ESP32 port** (`mshingai/ardupilot-esp32` carries `SCurve.h`) |
| solves | time-optimal, multi-DoF **synchronised** trajectories | kinematic shaping of a setpoint |

That last row is the real argument. Ruckig's hard problem is synchronising
several axes onto a time-optimal profile. **We do not have that problem.** We
want one axis' setpoint to stop stepping, and each axis is independently
mixed. Paying 406 KiB and a soft-float tax for time-optimal multi-DoF
synchronisation we will not use is the wrong trade.

If even `SCurve` is more than you want, ArduPilot's per-axis
`shape_vel_accel` / `input_vel_accel` kinematic-shaping helpers are the minimal
form — tens of lines, float, no state beyond the previous target.

**Whatever you choose, our ask is only that it is `float`.** On this chip that
is the difference between the hardware FPU and a libgcc call per operation.

## Honesty about scope

- Compiled and size-measured: **yes**, on your exact board target.
- Allocation behaviour: **yes**, instrumented, host build.
- On-target microseconds: **no** — see above.
- We have **not** implemented anything. This is a feasibility report against a
  recommendation we ourselves wrote, and it reverses it.

Reproduction is two files (`platformio.ini`, `main.cpp`) plus the two exclusions
above; happy to send the probe project.
