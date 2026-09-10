# The mixer's scale-down is computed, used, and then thrown away

From `duburi_ws` (Mongla), 2026-09-10. Against **rev 14**. Two asks, one small
and one that we think is a real defect.

---

## 1. The value exists and nothing can see it

`mixer::mix()` computes `maxabs[2]` — the per-group saturation — and applies it:

```cpp
for (int m = 0; m < NUM_THRUSTERS; ++m) {
    int g = (m < N_HORIZ) ? 0 : 1;
    if (maxabs[g] > 1.0f) out[m] /= maxabs[g];
}
```

Then `mix()` returns and `maxabs` goes out of scope. **Nothing upstream can
tell that a command was cut, or by how much, or which group.** From the
companion the vehicle simply under-performs its demand, and there is no signal
distinguishing "the water is pushing back" from "the allocator discarded a
third of that command".

Ask: two `sendNamed()` values, from a number you already have.

```
MIX_SAT_H   1/maxabs[0]   horizontal group (motors 0..N_HORIZ-1), 1.0 = not saturating
MIX_SAT_V   1/maxabs[1]   vertical group,                          1.0 = not saturating
```

We would take them at the existing slow telemetry rate — this is diagnosis, not
control. `MIX_VERT`/`MIX_VSGN` already exist but are a **sign probe** on a
synthetic full-ASCEND demand, not live saturation, so they do not answer this.

## 2. ⛔ The bigger one: the PIDs cannot see it either, and they wind up

`pid.h` has conditional-integration anti-windup, and its own comment says the
right thing:

> *"The mixer scales every thruster down uniformly on saturation, so saturation
> is common here, not exotic."*

But the condition is keyed on the **PID's own clamp**, not on the mixer:

```cpp
const float out_unsat = _kp*err + _ki*_integ + _kd*_deriv;
const bool  push_high = (out_unsat >=  _outmax) && (err > 0.0f);
const bool  push_low  = (out_unsat <= -_outmax) && (err < 0.0f);
if (!push_high && !push_low) { _integ = constrain(_integ + err*dt, -_imax, _imax); }
```

`attitude_control.cpp:42-44` sets `setLimits(imax, 1.0f)`, so `_outmax = 1.0`.

**A PID can sit well inside its own clamp while the mixer is discarding a third
of its output.** Using your own worked example from `mixer.cpp`:

```
forward = 1.0, yaw = 0.5   ->  motor 2 at -1.5  ->  group scale 0.667

yaw DEMANDED   0.500
yaw DELIVERED  0.333        33.3 % of the yaw command is discarded

out_unsat ~ 0.500  <  _outmax 1.0
    -> push_high == push_low == false
    -> THE INTEGRATOR KEEPS INTEGRATING
```

The yaw error persists (the axis is not achieving its demand), so the integrator
grows against a limit it cannot see:

| persistent yaw error | time to reach `imax` = 0.5 |
|---|---|
| 0.05 rad/s | 10.0 s |
| 0.10 rad/s | **5.0 s** |
| 0.20 rad/s | **2.5 s** |

**Then the forward burst ends.** The mixer stops scaling, the axis regains 1.5×
its authority, and it does so carrying a fully wound integrator. The symptom is
a hull that tracks heading acceptably during a hard translation and then
**snaps in yaw when the translation stops** — which reads as a tuning problem
and is not one.

### The fix is the one ArduPilot already uses

`AP_Motors_Class.h` carries exactly this, and for exactly this reason:

```cpp
struct AP_Motors_limit {
    bool roll, pitch, yaw;                 // "we have reached roll/pitch/yaw limit"
    bool throttle_lower, throttle_upper;
};
```

Those flags exist to protect "against overshoot in lean angle control" — the
same failure. Their allocator publishes *booleans*; **yours already computes a
magnitude**, which is strictly more information.

Minimal shape, entirely inside your code:

```cpp
// mixer.h
struct Saturation { float scale[2]; };   // 1/maxabs per group, 1.0 = clean
Saturation lastSaturation();             // set by mix(), read by the control loop

// pid.h — one more reason to hold, alongside the two that exist
bool authority_lost = (external_scale < 0.98f) &&
                      ((err > 0.0f) == (out_unsat > 0.0f));
if (!push_high && !push_low && !authority_lost) { _integ += err*dt; ... }
```

We are deliberately **not** prescribing where the scale is threaded — you own
the loop and the ordering, and `mix()` runs after the PIDs in the same 500 Hz
tick, so last-tick's value is the natural source and one tick of lag at 500 Hz
is 2 ms.

## 3. Why we are raising it rather than compensating

We cannot see it. Every axis of ours is a demand into `MANUAL_CONTROL` or
`SROT_MOVE`; the scale-down happens two layers below and is not reported. A
host-side workaround would be guessing at a number you already have exactly.

And it is the same shape as three things already fixed here on the same
argument: a disarmed `SROT_MOVE` reporting 100 % (rev 13), `REQUEST_MESSAGE`
ACKing everything (#5), ESC presence computed and dropped one line from the
wire (#10). **A value that exists, is correct, and never leaves.**

## 4. What we did not verify

**We did not compile or run this.** No PlatformIO toolchain our side. The
windup analysis is read from `pid.h`, `attitude_control.cpp:42-44` and
`mixer.cpp` at rev 14, and the arithmetic is your own worked example carried
forward — but we have **not** observed the yaw snap on hardware, because with
no thrusters fitted we cannot. Treat §2 as a code-grounded prediction, not a
measurement. If you want it measured before acting, the cheapest test is a
restrained hull: hold heading, command full forward for 10 s, release, and log
`_integ`.

Once `MIX_SAT_*` exists we will surface it on the companion (a health line and
the run scorecard) so a saturated dive is visible after the fact rather than
argued about.
