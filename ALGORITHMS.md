# SROT — Algorithms in Full Detail

The math and logic behind every control algorithm, step by step. This is the deep-dive
companion to `ARCHITECTURE.md` (which gives the big picture). Symbols: `ω` = body angular
rate (rad/s), `τ` = torque demand (normalized −1..1), `dt` = 0.002 s.

Contents: 1. Attitude estimation · 2. Cascade attitude control · 3. Yaw heading-hold ·
4. Depth control · 5. Model-based control (drag FF / cross-coupling / CoB trim) ·
6. Thrust allocation & shaping · 7. Motor-direction detect · 8. Relay auto-tune ·
9. Pico RPM closed loop · 10. Failsafes & arming.

---

## 1. Attitude estimation (`drivers/bno085.cpp`)

SROT does **not** run its own EKF. The BNO085 fuses accel + gyro internally (Kalman) and
outputs a unit quaternion `q = (qw, qx, qy, qz)` (GAME_ROTATION_VECTOR, mag off) at 400 Hz.

**INT-gated read:** the driver reads only while the BNO's INT line is asserted LOW (a full
SHTP packet is ready), with a 100 ms blind-poll safety net. This avoids reading mid-packet
(the cause of high-rate SHTP resets).

**Quaternion → Euler** (radians), with the `asinf` argument clamped so float rounding on a
near-unit quaternion can't produce NaN:
```
yaw   = atan2(2(qx·qy + qz·qw),  qx²−qy²−qz²+qw²)
s     = clamp(−2(qx·qz − qy·qw) / (qx²+qy²+qz²+qw²),  −1, +1)
pitch = asin(s)
roll  = atan2(2(qy·qz + qx·qw), −qx²−qy²+qz²+qw²)
```
Body rates `ω = (gx, gy, gz)` come straight from the calibrated gyro report (low latency —
this is what the rate loop uses). On a BNO reset the last-good attitude is held for 150 ms.

### One-shot magnetic yaw reference (`control/yaw_ref.cpp`)

`GAME_ROTATION_VECTOR` is the **mag-free** fusion, chosen on purpose: inside a metal hull with
eight thrusters drawing tens of amps, a 9-DOF yaw jumps the instant you throttle up, because
motor current swamps the ~50 µT earth field. The price is a yaw with **no earth reference**.

`MAG_YAW_REF = 1` recovers the reference without ever putting the mag in the loop. Once, at
boot, disarmed and still, over ≥100 samples spanning ≥1 s:

```
# 1. hard/soft iron cal (from CAL_MAG_*)
c = (m − mag_offset) ⊙ mag_scale
# 2. reject implausible fields — 25..65 µT is the whole earth's surface
if |c| ∉ [25, 65] µT: refuse
# 3. TILT-COMPENSATE using the (mag-free, trustworthy) roll/pitch.
#    atan2(my, mx) alone is valid only dead level: a few degrees of bow-down
#    injects an error the same size as the one we are correcting.
Xh = cx·cos(p) + cy·sin(r)·sin(p) − cz·cos(r)·sin(p)
Yh = cy·cos(r) + cz·sin(r)
heading = atan2(−Yh, Xh)
# 4. circular mean of (heading − game_yaw), + declination
offset = wrapPi(atan2(Σ sin Δ, Σ cos Δ) + MAG_DECL)
```

Headings are averaged as **unit vectors, not angles** — a naive mean of samples either side of
±π lands 180° wrong. The same resultant length `R = |Σ(sin,cos)| / n` doubles as the stability
gate (`R < cos(8°)` → refuse); `max − min` would be wrong across the wrap.

`Task_SensorRead` then publishes `yaw = wrapPi(game_yaw + offset)` at the **single** point where
yaw is written, so heading-hold, absolute `TURN`, `ATTITUDE` and `VFR_HUD` cannot disagree.

**Fails safe.** Low mag accuracy, bad field magnitude or unstable samples all refuse, leaving
`offset = 0` — i.e. exactly the pre-existing relative-yaw behaviour. Refusals are sticky (a
retry loop would flap the reported heading); `MAG_ALIGN = 1` re-tries explicitly, and is refused
while armed because dropping the offset mid-dive would step the heading-hold target.

**What it does not fix:** drift. The 6-axis yaw still creeps ~0.5–3 °/min, so a long dive loses
absolute accuracy. Correcting that needs a continuous mag correction — reintroducing exactly the
motor sensitivity this design avoids. Enabling `BNO_USE_MAG` turns the mag *report* on but never
adds the mag to the fusion.

---

## 2. Cascade attitude control (`control/attitude_control.cpp`, `control/pid.h`)

Two nested loops per axis. **Outer (angle) loop** turns an angle error into a desired rate;
**inner (rate) loop** turns a rate error into a torque.

**Outer, roll & pitch:**
```
target_angle = expo(stick) · MAX_LEAN_RAD          # ≈ ±34°, PILOT_EXPO applied
desired_rate = (target_angle − measured_angle) · ATC_ANG_*_P
```
Yaw is handled separately (§3).

**Inner rate PID** (`pid.h`) — the workhorse. For each axis:
```
err    = desired_rate − ω
I     += err · dt ;   I = clamp(I, −IMAX, +IMAX)          # ATC_RAT_*_IMAX
D      = −(ω − ω_prev) / dt                                # derivative ON MEASUREMENT
out    = Kp·err + Ki·I + Kd·D
τ      = clamp(out, −1, +1)
```
Key choices: **derivative-on-measurement** (not on error) → no "derivative kick" when the
setpoint steps; **integrator clamp** (anti-windup) so `I` can't run away at saturation;
**output clamp** to the normalized torque range. `dt` is a fixed 0.002 s.

**Rate feedforward** adds a lead proportional to the *commanded* rate:
```
τ = PID(desired_rate, ω) + ATC_RAT_*_FF · desired_rate
```

**Stick expo** (fine centre resolution, full authority at the ends):
```
expo(x) = (1−e)·x + e·x³        # e = PILOT_EXPO ∈ [0,1]
```

---

## 3. Yaw heading-hold (`attitude_control.cpp` `stabilize()`)

Yaw has two behaviours selected by the stick, with the max rate set by `PILOT_YAW_RATE`
(deg/s → rad/s):

```
yaw_stick = expo(stick_yaw)
if |yaw_stick| > 0.02:                     # active → fine RATE command
    desired_yaw_rate = yaw_stick · max_yaw_rate
    yaw_target = measured_yaw ;  hold = true
else:                                       # centred → HEADING-HOLD
    if capture_pending: yaw_target = measured_yaw; hold = true; capture_pending = false
    err = wrapPi(yaw_target − measured_yaw)                 # wrapped to [−π, π]
    desired_yaw_rate = clamp(err · ATC_ANG_YAW_P, ±max_yaw_rate)
```
On mode entry — and on the **disarmed→armed edge** — `reset()` sets `capture_pending = true`,
so the heading is captured on the **first centred cycle** (hold engages immediately, not only
after the first nudge). `wrapPi` guards NaN (a NaN would spin its `while` loop → watchdog).

### Explicit locks — `holdYaw(yaw_rad)`

Relying on "whatever the loop last latched" is correct in practice but guarantees nothing, so
AUTO sets the target outright:

```
holdYaw(y):  yaw_target = wrapPi(y);  hold = true;  capture_pending = false
```

`movement` raises a one-shot `Demand.yaw_lock` and the control loop forwards it:

- **Every translate leg** (`FWD`/`BACK`/`LEFT`/`RIGHT`, plus `HOLD`/`STOP`/`DIVE`) locks the
  heading it started with, held through cruise *and* braking.
- **A completed `TURN`** locks `s_target_yaw` — the heading it was *commanded* to reach.
  Previously the hold inherited `measured_yaw` at the moment the ±0.03 rad (~1.7°) completion
  test passed, so `TURN 90°` settled anywhere in 88.3–91.7° and then held that error; across a
  sequence of turns it compounded.
- **`ARC` is excluded** — it commands a yaw rate by design.

A yaw stick or `TURN` always overrides the lock (the `|yaw_stick| > 0.02` branch re-latches
every cycle), so the pilot still wins.

### Yaw reference — relative by default

The held heading is only as absolute as the yaw feeding it, and `GAME_ROTATION_VECTOR` yaw is
**relative**: an arbitrary zero that drifts ~0.5–3 °/min. Heading *hold* does not care (it
tracks a target in the same drifting frame, and drift over a 5 s leg is negligible), but
absolute `TURN` and run-to-run repeatability do. `MAG_YAW_REF` fixes the reference with a
one-shot magnetic alignment — see §1 and PARAMETERS.md.

---

## 4. Depth control (`control/depth_control.cpp`)

A single PID drives the vertical (heave) demand. The throttle stick moves the *target* rather
than commanding thrust directly:
```
if |stick| > 0.05:  target −= stick · MAX_CLIMB_MS · dt ;  target = max(target, 0)
heave = PID(−target, −measured_depth, dt)        # DEPTH_P/I/D, out clamp ±1
```
Entering DEPTH_HOLD latches `target = current_depth`.

### The sign convention, spelled out

Get this backwards and the vehicle dives when told to surface, so it is worth being explicit.
Two facts, from `docs/THRUSTER_MAP.md`:

- **Depth is positive DOWN** (metres below the surface).
- **Heave is positive UP** — `throttle +1` maps to `−1` on M5–M8, and a *positive* command on
  those motors pushes the vehicle *down*. Two negatives: positive throttle ascends.

Error and output therefore live in **opposite frames**, which is the trap. The PID is fed
**altitude** (`−depth`) so that error, output, integrator and derivative all share one frame.
Worked through, with the subject stated explicitly each time — this is where the original slip
happened, so no shorthand:

```
err = setpoint − measurement = (−target) − (−measured) = measured − target

measured deeper than target (measured > target):  err > 0 → heave > 0 → ASCEND  ✔ back toward target
measured shallower than target (measured < target): err < 0 → heave < 0 → DESCEND ✔ back toward target
SURFACE failsafe (target = 0, measured = 3 m):    err = +3 → heave > 0 → ASCEND  ✔
```

> This was **inverted** until 2026-07-30: the loop ran `PID(target, measured_depth)`, so a
> target deeper than the current depth produced a *positive* error and hence an *ascend*
> command. `DEPTH_HOLD` was divergent rather than mistuned, and — much worse — the SURFACE
> failsafe (leak / low battery / GCS loss, which sets target 0) drove the vehicle **down**.
> It survived because the Bar30 had not been fitted, so the loop had never run closed. See
> `AUDIT.md` R1.
>
> Negating the *inputs* rather than the output is a readability choice, not a correctness one —
> for this PID the two are provably identical from zero state. Altitude is preferred so that
> error, output, integrator and derivative share one frame, which is precisely the confusion
> that caused the bug.

---

## 5. Model-based control — Phase 2a (`control/feedforward.cpp`)

Applied to the torque/heave demands **after** the mode and **before** the mixer. All gains
default 0/off → identical behaviour until tuned. Grounded in Fossen's AUV equation of motion
`M ν̇ + C(ν)ν + D(ν)ν + g(η) = τ` — each term maps to one part of that equation.

### 5.1 Angular drag feedforward (the `D(ν)ν` nonlinear damping)
Rotational drag is ~quadratic in rate. To hold or change a rate the controller must supply
that drag torque — so **pre-inject it** and the rate PID sees a near-linear plant:
```
roll  += ATC_DRAG_RLL · gx·|gx|
pitch += ATC_DRAG_PIT · gy·|gy|
yaw   += ATC_DRAG_YAW · gz·|gz|
```
Uses the **measured** rates, so it's a real measured-state feedforward (doable now — we have
the gyro). Signed `x·|x|` keeps the correct direction. *(Linear surge/sway/heave drag FF is
Phase 3 — it needs a velocity sensor we don't have.)*

### 5.2 Cross-coupling cancellation (the `C(ν)ν` Coriolis + off-diagonal terms)
A fast yaw induces roll/pitch through hydrodynamic coupling. Rather than wait for the gyro
to see the disturbance and the loop to react, **proactively counter it** from the yaw rate:
```
roll  += XC_YAW2RLL · gz
pitch += XC_YAW2PIT · gz
```
Two empirical gains, tuned by watching roll/pitch during yaw steps — ~80% of full 6×6
decoupling for ~5% of the effort (the full identified matrix needs pool/CFD system-ID).

### 5.3 Centre-of-Buoyancy auto-trim (the `g(η)` restoring term)
A shifted CoB/CG makes the sub sit with a constant tilt the attitude loop fights via a
steady rate-PID integrator. Bleed that steady integrator into a persistent feed-forward trim
so the integrator returns toward zero:
```
if angle-stabilized and TRIM_EN and sticks≈centre:      # "learn" gate
    trim_roll  = clamp(trim_roll  + TRIM_LEAK · I_roll , ±TRIM_MAX)
    trim_pitch = clamp(trim_pitch + TRIM_LEAK · I_pitch, ±TRIM_MAX)
    trim_thr   = clamp(trim_thr   + TRIM_LEAK · I_depth, ±TRIM_MAX)   # depth modes only
roll += trim_roll ; pitch += trim_pitch ; throttle += trim_thr
```
As the trim grows, the angle error shrinks → `I` unwinds → the trim converges to hold the
bias with `I ≈ 0`. **Benefit:** full I-term headroom for real disturbances, no slow windup,
faster recovery, and a drifting trim value is a health signal. **Not a benefit:** it does
*not* save energy — the same thrust holds the sub level whether the number comes from `I` or
the trim; only physical ballast changes that. Learning freezes during large stick input and
resets on mode change (so a maneuver can't corrupt it). Finally all demands are clamped ±1.

---

## 6. Thrust allocation & shaping (`control/mixer.cpp`)

### 6.1 6-DOF allocation
The six body demands `[roll, pitch, yaw, throttle, forward, lateral]` map onto 8 thrusters
by a fixed **SUB_FRAME_VECTORED_6DOF** matrix `M` (row = motor, column = axis):
```
out[m] = Σ_axis  M[m][axis] · demand[axis]
```
M1-4 (horizontal) carry yaw/forward/lateral; M5-8 (vertical) carry heave/roll/pitch.

**Saturation** — if any `|out[m]| > 1`, scale the whole set down uniformly so the *relative*
mix (control authority) is preserved rather than clipping one motor:
```
s = max(1, max_m |out[m]|) ;   out[m] /= s
```

### 6.2 Per-thruster shaping → DShot (`oneToDshot`)
Each normalized output becomes a 3D-DShot value through, in order:
```
n = clamp(out · dir, −1, 1)                 # dir = MOT_n_DIRECTION × detect sign
t = |n|
if t < 0.005:                               # centred → STOP (or MOT_SPIN_ARM idle)
    return NEUTRAL(1048)  (or forward idle at MOT_SPIN_ARM)
thr    = thstExpo(t, MOT_THST_EXPO)         # thrust-curve linearization (below)
shaped = MOT_SPIN_MIN + (1 − MOT_SPIN_MIN)·thr      # lift across the prop deadband
return  n≥0 ? 1049 + shaped·998            # forward band: 1049 (min) … 2047 (max)
            :   48 + shaped·999            # reverse band:   48 (min) … 1047 (max)
```
**Both DShot 3D bands run low→high *within themselves*** — the reverse band is NOT mirrored
about the 1048 neutral gap. Mapping it backwards (`1047 − shaped·999`, which this document
previously described) makes the *smallest* reverse demand ~full reverse and *full* demand a
dead stop: the motor judders from standstill and never spins up.

**Thrust linearization** — propeller thrust ∝ throttle² (`thrust = (1−e)·thr + e·thr²`), so
to make command→thrust linear we invert that quadratic (`thstExpo` solves it):
```
thstExpo(thrust, e) = ((e−1) + √((1−e)² + 4·e·thrust)) / (2e)      # e≥0.001; e→0 = linear
```
This is the same model as ArduPilot `MOT_THST_EXPO`. `MOT_SPIN_MIN` guarantees the smallest
command crosses the prop deadband instead of doing nothing then lurching. 1048 is the
3D-DShot neutral (zero thrust, ESC stays armed) — fixing the old "creep on arm" bug.

---

## 7. Motor-direction detect (`control/calibration.cpp`, mode 20)

For each thruster, a settle→thrust→detect state machine:
```
SETTLE:  hold all motors neutral; wait until gyro is quiet (|ω|² small) for 500 ms
THRUST:  pulse motor m at +0.30 for 200 ms
DETECT:  read the peak gyro response; compare its sign on the dominant axis to the
         mixer's expected angular factor for m  →  MOT_(m)_DIRECTION = ±1
```
The expected response comes from `mixer::motorAngular(m)` (the roll/pitch/yaw row of `M`).
If the measured sign matches, dir = +1; if opposite, dir = −1. The effective direction is
`MOT_n_DIRECTION × detect`, and it is baked into the **published `norm[]`** (in
`task_control_loop.cpp`) so it applies exactly once and reverses a thruster identically on the
raw DShot path **and** the Pico closed-loop RPM path (which derives its signed target from
`norm`). So the ArduSub-style setup reversal works on the Pico backend in both modes.

---

## 8. Relay auto-tune (`control/autotune.cpp`, Åström–Hägglund)

For each loop in turn (rate roll→pitch→yaw, angle roll→pitch→yaw, depth), apply a **relay**
(bang-bang) excitation: output `+A` when the error is negative, `−A` when positive. The
plant limit-cycles; measure the oscillation period `Tu` and amplitude `a`:
```
Ku = 4A / (π·a)                              # ultimate gain
Kp = 0.33·Ku                                 # conservative ("some overshoot") ZN
Ki = Kp / (0.5·Tu)
Kd = Kp · (Tu/3)
```
Results are clamped to sane caps and written to the `ATC_RAT_*`/`ATC_ANG_*`/`DEPTH_*` params;
`ATUNE` reports 0→1 progress and resets on completion. A phase with no motion (disarmed /
wrong direction) is skipped, leaving that gain unchanged. Flash save is deferred to Core 0.

---

## 9. Pico RPM closed loop (`src/pico/main.cpp`, Stage 2)

The Pico receives a signed target RPM per motor and closes a per-motor loop so a given
command means the same **thrust** regardless of battery voltage / load. Best-practice control =
**feedforward + PI + Dynamic Idle + slew** (gains come from the ESP32's `tl_cfg_t` frame, set by
MOTOR_TUNE):
```
meas   = IIR-filter( signedRpm(eRPM/pole_pairs, dir) )       # RPM_FILT alpha
if |target| < RPM_MIN_TARGET:                                # centred stick
     target = RPM_IDLE (fwd)   if RPM_IDLE≥1  (Dynamic Idle: hold min rotation)
     else → neutral 1048, I=0  (stop)
err    = (target − meas) / RPM_MAX
I     += RPM_KI · err ;  I = clamp(I, ±1)
level  = RPM_FF_A·target        # feedforward (dominant, fast)
       + RPM_KP·err + I         # PI trims the residual
level  = slew_limit(level, RPM_SLEW)                         # cap Δ/cycle → no current spikes
DShot  = levelToDshot3D(clamp(level,−1,1))                   # 1048 neutral / bands
```
The **feedforward** does most of the work (open-loop-accurate), so the PI only corrects
disturbances → fast, no lag. **Dynamic Idle** holds a minimum RPM instead of a flat DShot idle,
avoiding restart lag / desync. **Fault
detection** distinguishes two cases from the telemetry:
- **Present** (`TL_ST_PRESENT`): a valid eRPM was decoded within `PRESENT_TIMEOUT_MS` (500 ms).
  No telemetry ever → **not present** = "not detected" (ESC absent, or bidir DShot off) → the
  ESP32 reports `STATUSTEXT "Thruster N not detected"` (WARNING), **no** fault bit.
- **Fault**: present **and** commanded (`level ≠ neutral`) but `|meas| < RPM_FAULT_RPM` for
  `> RPM_FAULT_MS` → set the fault bit → ESP32 raises `"Thruster N fault"` (ERROR) + `ESC_STATUS`.

So a bare bench (no ESCs) reads "not detected", not "fault". Thrust estimation `T ≈ k_T·RPM²`
is plumbed for Phase 2a's model but not yet consumed. A one-shot **beacon beep** (`TL_FLAG_BEEP`,
disarmed only) chirps the ESCs on startup/connect.

---

## 10. Failsafes & arming (`tasks/task_control_loop.cpp`, `control/arming.cpp`)

Each cycle, unless already surfacing:
```
fs = (LEAK_EN & leak) | (FS_BAT_ENABLE & PM1 < FS_BAT_VOLTAGE) | (FS_GCS_ENABLE & GCS lost)
if fs & armed:  mode ← SURFACE                 # controlled ascent to 0 m
```
**Arming:** `arming::canArm()` enforces pre-arm prerequisites (unless `ARMING_CHECK=0`); a
lost prerequisite while armed disarms. The 2nd-board thruster-kill is **display-only** and
never gates arming. **Motor test** requires a ≥2 Hz keep-alive and auto-disarms on a >500 ms
gap. When disarmed, every thruster gets DShot 0 (stopped).

---

## 11. AUTO movement primitives (`control/movement.cpp`, mode 23)

One state machine offloads the Jetson's motion: it turns a high-level verb (forward / back /
strafe / turn / dive / arc / style / stop / hold) into thruster demands, with heading- and
depth-hold running around it in the control loop. Commanded over `MAV_CMD_SROT_MOVE` — see
`JETSON_COMMS.md`.

### 11.1 Voltage-independent cruise → repeatable distance (timer-based)
Translation runs the axis at a **speed** `S ∈ [0, MOVE_CRUISE_MAX]` for a **duration** — the system
is purely time-based, no distance sensor or estimate. Because the Pico closes an **RPM** loop (§9,
enabled via `THR_RPM_CLOSED_LOOP=1`), speed `S` maps to a target RPM `S·THR_MAX_RPM` that the Pico
holds regardless of battery voltage — so "duration × speed" travels the **same distance every run**
(full or low battery), provided the RPM gains are fitted (MOTOR_TUNE, §9/§ MOTOR_TUNE). Start is
ramped: `S_cur ← min(S_cur + MOVE_ACCEL·dt, S)` (no lurch).

### 11.2 On-board braking (no coast / no drift)
At the end of the duration the ESP32 does **not** coast — it applies **reverse thrust** along the
travel axis to null momentum:
```
brake demand = −axis · MOVE_BRAKE_GAIN · S          # opposite the cruise, scaled by speed
brake time   = MOVE_BRAKE_K · |S|   (seconds)       # more speed → longer brake
```
Momentum ∝ the (known, RPM-held) cruise speed, so scaling the brake impulse by `S` stops the AUV in
the same place repeatably. The Jetson never sends a brake.

### 11.3 Smooth dive (no splash)
`DIVE` ramps the depth **setpoint** toward the goal at `MOVE_DEPTH_RATE` (m/s) rather than stepping
it, and `depth::update` (§4) tracks that moving target — a gentle descent/ascent instead of a lurch.
The same ramp smooths the depth hold on every other verb.

### 11.4 Turn — shortest path, rate-limited
`TURN` carries a yaw rate (deg/s, default `MOVE_YAW_RATE`) and a mode: **relative** target
`wrapPi(yaw + Δ°)` or **absolute** target `H°`. It drives `yaw = sign(err)·rate/PILOT_YAW_RATE`
with `err = wrapPi(target − yaw)`, so it always takes the **shortest** direction and locks the new
heading when `|err| < 0.03 rad` (~1.7°). `ARC` is `FWD` + a signed turn rate at once (curved path).

### 11.5 Style, preemption, safety, feedback
`STYLE` delegates to the spin controller (§ stunt: N×360° roll) then re-levels. Every command has a
**timeout** (brakes out) and is **preemptible** — a new command runs a quick brake then takes over.
The safety monitor (tumble > `ST_ANGLE_MAX`, rate > `ST_RATE_MAX`, RPM/NaN) aborts + disarms; dives
are intended, so the depth-runaway check in AUTO guards the **setpoint** (`depth::target()`), not
the depth the leg started at — passing the current depth made the delta identically zero and left
AUTO with no depth protection at all (`AUDIT.md` B9). `progress()` (elapsed/duration, then
brake tail) feeds the `COMMAND_ACK` progress stream (`MV_STATE`/`MV_PROG`/`MV_TYPE` telemetry).

---

*Code map: estimation `drivers/bno085.cpp`; control `control/{attitude_control,depth_control,
pid,feedforward,mixer,calibration,autotune,movement}.cpp`; loop `tasks/task_control_loop.cpp`; Pico
`src/pico/main.cpp`. Parameters: `PARAMETERS.md`. Jetson comms: `JETSON_COMMS.md`. Wiring:
`HARDWARE.md`.*
