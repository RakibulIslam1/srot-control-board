# The vehicle has a velocity sensor it built, wired, decoded — and throws away

From `duburi_ws` (Mongla), 2026-09-10. **Hardware + firmware ask.**
This is a capability request, not a bug report.

---

## The problem it solves

**We have no velocity sensor.** No DVL (`vehicle-spec.md`: not fitted, never
validated). The bottom-camera optical flow works — verified **1.09 cm over
30 cm (3.6 %)** — but it is a *low-speed, lit, textured-floor* sensor: MBARI
measured ROV flow accuracy falling off above **~0.3 m/s**, and it needs the
floor in view. Transit, turbid water and darkness are exactly where it stops,
and exactly where a mission is moving fastest and drifting worst.

## The observation

**The board already measures, per thruster, the two quantities that determine
water speed through a propeller — and discards both.**

* `thruster_link_proto.h:58` — `int16_t rpm[TL_NUM_THRUSTERS]`, signed
  mechanical RPM. Carried on the wire, used for the RPM loop.
* Our PR #4 — *"Pico: ESC voltage, current and temperature are decoded and then
  discarded."* Current never reaches the ESP32 at all.

## The physics

For a propeller, with `n` = rev/s, `D` = diameter, `ρ` = water density:

```
J   = V_A / (n D)                  advance ratio
K_Q = Q / (ρ n² D⁵)                torque coefficient
```

Measure `n` (RPM) and `Q` (from current: `Q = K_t · I` for a BLDC), evaluate
`K_Q(J)`, **invert for J**, and:

```
V_A = J · n · D
```

That is a **water-speed sensor**, at every speed, in the dark, in silt, with
nothing in view.

## Why nobody has this, and why we nearly could not either

**RPM alone is not enough**, and that is the whole reason the published work
struggles. A propeller at fixed RPM makes different thrust at different inflow,
so RPM→velocity is ambiguous. The AUV literature that uses RPM as a DVL-outage
aid (NARX, LSTM, self-attention models) reports exactly the symptom you would
predict: *"amplitude bias and phase lag ... the velocity inferred from RPM
exhibits lag and deviation."* They fit their way around an ambiguity instead of
removing it.

**Torque is the disambiguating second measurement**, and for a BLDC it is
current — which our ESCs report and the Pico drops.

Worth noting: **DeepVL** (ICRA 2025), the state of the art in DVL-free
underwater velocity estimation, uses IMU + **motor commands** + battery voltage.
*Commands.* Not measured RPM, not current. It has to learn what the propeller
did because it cannot see it. **We can see it.**

⛔ **The one thing that blocked us was that the T200 is somebody else's part.**
Blue Robotics publish **bollard-pull only** — thrust with the vehicle held still
— so there is no `K_T`/`K_Q`-vs-`J` curve for it. Our own simulator says so at
`generate_model.py:293`, where the falloff coefficient is marked **MODELLED,
not measured**.

**We are building our own thrusters, so that blocker is gone.**

---

## The asks

### 1. Hardware — thrusters (the enabler)

Since the thruster is ours to specify, three things make it a sensor as well as
an actuator:

| ask | why |
|---|---|
| **Document the propeller geometry**: diameter `D`, pitch ratio `P/D`, blade-area ratio `A_E/A_O`, blade count `Z` | These four are exactly the inputs to the **Wageningen B-series** polynomial regressions (Barnitsas et al., 1981), which give `K_T(J)` and `K_Q(J)` **analytically**. Reference implementation: `wageningen.m` in Fossen's Marine Systems Simulator (`cybergalactic/MSS`). No tow tank required to get a first curve. |
| **An ESC with bidirectional DShot telemetry INCLUDING current**, specified rather than hoped | Current is the torque channel. Without it this is just RPM again, with the published lag. |
| **The motor's torque constant `K_t`** from the winding spec | Converts current to torque. One number, known at design time, otherwise a calibration we would have to invent. |

Not asked for: a shaft encoder, a flow sensor, or any new penetrator. **This
adds no parts.** It is a documentation and part-selection ask.

### 2. Firmware — forward what you already decode

**PR #4 already asks for this** (ESC voltage/current/temperature), which is why
this is not a new decode. What we would add is only that **current is now
load-bearing**, not diagnostic: please carry it per thruster at the telemetry
rate, alongside the RPM already on the wire.

---

## How we would validate it, honestly

We are **not** proposing to trust this in a control loop on day one. The test is
cheap and decisive, and it uses a sensor we already have:

1. Land PR #4 so current reaches the host.
2. Fly an ordinary pool run with the downward optical flow enabled.
3. Log `(RPM, current, command, V_flow)` and ask one question:
   **does `(RPM, current)` predict `V_flow` better than RPM alone?**
4. Only if it does, fit `K_Q(J)` and put it in a filter.

**The pool run is the calibration.** We use the sensor that works in the easy
regime — flow, on a lit textured floor at low speed — to teach the sensor that
works in the hard regime. No tow tank, no vendor curve, no separate test rig.

If step 3 fails, the fallback is still ahead of the published art: feed measured
RPM **and** current as extra inputs to a learned model that today sees only
commands.

## What we are claiming, and what we are not

**Claimed:** the data exists on the vehicle, is discarded, and the physics to
use it is standard and closed-form once the propeller geometry is known.

**Not claimed:** that it works. Nobody has run step 3. The numbers above are
from published sources and our own code, not from our water.

We think this is the highest-value thing the thruster design can give us beyond
thrust — and it costs a datasheet page and an ESC part number.
