# Ask O — one afternoon on a load cell closes four open asks

**To:** the hardware / thruster team (Rakibul Islam).
**Shape:** a bench protocol, not a code change. Nothing here asks anybody to modify firmware.
**Cost:** one rig, one afternoon, one CSV.

---

## 1. Why this is the highest-value hour on the whole thruster programme

We built a closed-loop bench — the board's own control code flying a Fossen 6-DOF plant
derived from this hull's CAD geometry — and swept every actuator property to see which one
the control loop actually cares about. The answer was not the one we expected.

Yaw hold against a 1.5 N·m torque burst, first-order thruster lag τ swept, control loop at
500 Hz throughout:

| τ | peak heading deviation | vs no lag |
|---|---|---|
| 0.00 s | 0.394° | — |
| 0.05 s | 1.026° | 2.6× |
| 0.10 s | 2.030° | 5.2× |
| 0.59 s | 51.49° | **131×** |

For scale: dropping the **control loop from 500 Hz to 50 Hz** — a tenfold change — cost
**21 %**. **Thruster response time beats control rate by one to two orders of magnitude.**

⚠ **And we do not know ours.** Every number in that table is a sweep parameter. `k_n_per_rpm2`
is `null`, `MOT_THST_EXPO = 0.65` is fitted to a T200 we are not flying, and
`REVERSE_EFFICIENCY = 0.77` is a vendor figure for a different propeller. This test replaces
all of it.

---

## 2. ⭐ The same rig answers four separate asks at once

We had been asking for these in different places. They are all the same experiment.

| ask | what this test gives it |
|---|---|
| [M §2.1](pr-m-thruster-requirements-from-control.md) — turns smoothly from zero | the **break-away cost**: the extra lag a stopped propeller has over a turning one. That difference is the entire justification for `MOT_SPIN_MIN` |
| [M §2.2](pr-m-thruster-requirements-from-control.md) — symmetric reverse | forward and reverse plateaus off the same rig. `REVERSE_EFFICIENCY` stops being a vendor number |
| [M §2.4](pr-m-thruster-requirements-from-control.md) — a real thrust curve | the plateau of each step **is** a point on the curve. Sweeping command gives the whole thing, and `k_n_per_rpm2` its first real value |
| [M §7](pr-m-thruster-requirements-from-control.md) — response time | the transient of each step: dead time and τ |

---

## 3. The rig

Nothing exotic. A thruster clamped in a tank or a barrel, a load cell on its mount, the ESC on
the bench supply at the pack voltage we actually fly.

**What to log, as one CSV per step:**

```
time_s, thrust_n [, rpm]
```

| requirement | value | why |
|---|---|---|
| sample rate | **≥ 500 Hz**, 1 kHz preferred | τ is expected around 70–90 ms; we need ~10 samples inside the rise |
| record length | **≥ 5 τ after the step**, so ≥ 0.5 s | our analysis **refuses** a shorter record rather than extrapolating a plateau — §5 |
| pre-step baseline | ≥ 0.1 s of it | so the zero is measured, not assumed |
| supply voltage | logged, and **held** | the vendor's own reverse ratio drifts 0.787 → 0.754 across 12–20 V; a single-voltage curve is not enough |
| water | submerged, and say fresh or salt | a bench-air run measures a different machine entirely |

⚠ **Submerged, not in air.** §1's whole point is the water column: `τ = L·√(ρA/T)`. In air ρ
is 1/800th and the number means nothing.

---

## 4. The steps to run

### 4.1 ⭐ The break-away pair — the one that decides `MOT_SPIN_MIN`

Two steps to the **same** final command, differing only in where they start:

| | from | to |
|---|---|---|
| **A — from stopped** | propeller at rest | 30 % command |
| **B — from running** | propeller already at ~10 % | 30 % command |

**The difference in total lag between A and B is the break-away cost.** If it is near zero, a
stopped propeller costs nothing to start, `MOT_SPIN_MIN` can go to 0, and the 16.2 % output
quantum disappears along with it. If it is large, the floor is doing real work and we stop
asking for it to be removed.

⭐ **This single pair is the most valuable measurement in the document.**

### 4.2 The curve, and the reverse asymmetry

Steps from rest to each of: **10, 20, 30, 40, 50, 60, 80, 100 %**, forward — then the same set
in reverse. Sixteen steps, a few seconds each.

- each **plateau** is a point on the thrust curve → replaces `MOT_THST_EXPO = 0.65`
- forward vs reverse plateau at equal command → replaces `REVERSE_EFFICIENCY = 0.77`
- with rpm logged, plateau thrust against rpm² → **`k_n_per_rpm2`, which is currently `null`**
- each **transient** gives τ at that operating point, which tests `τ ∝ 1/√T`

### 4.3 If there is time: the duct-length check

⭐ Our model says **duct length is the biggest lever on response time**, `τ ∝ L`, and it is a
CAD change rather than a motor or firmware change. If two duct lengths can be printed — say
150 mm and 75 mm — running 4.1 on both tests that prediction directly. If τ does not halve,
our model is wrong and we would very much like to know.

---

## 5. What we do with it — the analysis is written down, not negotiable

`tools/fit_thruster_response.py` in `mongla_ws`. Send the CSV; the fit is:

```
thrust(t) = 0                                      t <  t_dead
thrust(t) = T_ss · (1 − exp(−(t − t_dead)/τ))      t >= t_dead
```

```
python3 tools/fit_thruster_response.py step_A_from_stopped.csv
```

It reports dead time, τ, total lag, 10–90 % rise, steady thrust, **and its own RMS residual**
— so a response that is *not* first-order shows up as a bad fit rather than as a confident
number.

⚠ **Dead time and τ are reported separately and must not be added carelessly.** A first-order
lag can be compensated by a controller; pure dead time cannot. Their sum is the right figure
for *ranking two prototypes*, and the wrong figure for deciding what is fixable in software.

⭐ **The analysis is validated against traces whose answer we chose** — synthetic steps at
τ = 20/79/200/590 ms, with and without load-cell noise, forward and reverse — the same
discipline we apply to the free-decay fit. If it could not recover a known τ from a generated
trace, it would return a plausible one from a real trace and nobody would know.

It **refuses** a record shorter than 4 τ after the step rather than extrapolating a plateau,
because every downstream number — the thrust curve, `REVERSE_EFFICIENCY`, `k_n_per_rpm2` —
would silently inherit that extrapolation.

---

## 6. What we expect to see, so a surprise is recognisable

From momentum theory on the CAD tunnel geometry, at 20 N:

| | predicted τ |
|---|---|
| lateral tunnel, 84 mm bore × 150 mm | **79 ms** |
| vertical tunnel, 84 mm bore × 130 mm | **68 ms** |
| rotor spin-up alone (bell + resin prop) | ~15 ms |

⭐ **That is already about seven times faster than the ~0.59 s reported for a T100-class
unit** — short tunnel thrusters are inherently quick, and the figure that alarmed us is a
large open propeller, not a tunnel.

**If the measurement comes back near 80 ms, our model is good and §4.3's duct-length lever is
real.** If it comes back at 300 ms, something we have not modelled dominates — wall friction,
inlet contraction, ESC ramp limiting — and that is worth far more to us than a confirmation.

⚠ Momentum theory ignores all three of those, so treat §6 as a prediction to be falsified, not
as a target to match.
