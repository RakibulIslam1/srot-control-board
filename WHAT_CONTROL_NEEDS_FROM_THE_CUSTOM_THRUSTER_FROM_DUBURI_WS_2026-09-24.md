# Ask M — what the control layer needs from the custom thruster

**Status:** a requirements ask to the **hardware / thruster design team**, not a firmware PR.
Nothing here asks anybody to change code.

**Why now.** The T200 is a placeholder. Custom thrusters are being designed, and the thruster
is the one component whose properties the control layer cannot work around. Every constant
below that we currently ship is fitted to a T200 datasheet, and every one of them is a
placeholder for the same reason.

**Provenance.** Everything numeric in this document is either (a) read in the firmware source
with file and line, or (b) measured on `tools/control_bench/`, which compiles the board's own
`mixer.cpp`, `attitude_control.cpp`, `feedforward.cpp`, `thrust_trim.cpp` and
`depth_control.cpp` natively and is validated against live-board captures to under one DShot
count of 999. Where a number comes from a vendor datasheet it says so, and that number is a
placeholder.

---

## 1. The measurement that prompted this

Host yaw stick, through `attitude::stabilize` into the mixer, into a motor, on the bench.
Single-shot command, loop otherwise quiet:

| host yaw stick | yaw torque at the mixer | motor output |
|---|---|---|
| 3.0 % | 0.002972 | **0.0 %** |
| 4.0 % | 0.003963 | **0.0 %** |
| 5.0 % | 0.004956 | **0.0 %** |
| 7.5 % | 0.007444 | 16.8 % |

`VISION_YAW_MIN_PCT = 5.0` is the floor our vision servo lifts small yaw demands to. It
produces **zero motor output** — it falls 0.9 % short of the mixer's `0.005` cliff. The first
demand that moves the vehicle at all produces **16.8 %** of full scale.

That cliff is not a mixer bug. It is `MOT_SPIN_MIN` doing exactly what it was written to do:
lift any non-zero demand up to a floor, because a brushless thruster below that floor does not
turn reliably — it cogs, stalls, or sits drawing current. **The floor exists because of the
thruster.** A thruster that turns smoothly from zero removes it.

**So the single most valuable property a custom thruster can have, for control, is not more
thrust. It is resolution near zero.**

---

## 2. What we ask for, ranked

### 2.1 ⭐ Turn smoothly from zero — so `MOT_SPIN_MIN` can go to 0

What makes a floor necessary today:

- **cogging torque** — the detent the rotor has to break out of;
- **static friction in the shaft seal and bearings** — the part that grows with depth rating;
- **ESC startup behaviour** — a sensorless six-step controller has no idea where the rotor is
  until it is already turning, so it has to kick.

Each has a design answer: a skewed or fractional-slot stator for cogging; a low-drag seal for
static friction; and **a sensored or FOC ESC** for startup, which knows rotor position at zero
speed and can produce rated torque there.

What it buys us, concretely:

- the 16.2 % quantum disappears, so a small correction stops being a **lurch**. ⚠ Read
  §6.6 before weighting this: we have since measured the floor's steady-state cost in
  closed loop and it is about a tenth of a degree, which is *not* the problem. The
  problem is the size of the smallest step, not the jitter it leaves behind;
- **INDI becomes possible.** An incremental controller works by commanding small increments.
  An actuator whose smallest non-zero increment is 16.8 % of full scale is not an incremental
  actuator, and INDI on this hull is blocked on exactly this, nothing else;
- the vision servo stops needing a minimum-percentage constant at all.

**This is the ask. If only one thing on this page is designed for, make it this one.**

### 2.2 Symmetric forward and reverse

We currently carry `REVERSE_EFFICIENCY = 0.77` — a T200 property: the propeller is optimised
for one direction and gives about three-quarters of the thrust the other way. Every allocation
we compute has to know the sign of each thruster's demand before it can know what that demand
is worth, and the asymmetry makes the effectiveness matrix demand-dependent.

A symmetric propeller section makes `B` constant, which is worth more to the controller than
the few percent of peak forward thrust it costs. On a vehicle that spends its competition run
station-keeping and correcting, **reverse is not the rare direction.**

### 2.3 Telemetry: RPM **and current**, and do not make shaft speed the primary mode

- **RPM.** `k_n_per_rpm2` is `null` in `hull_geometry.yaml` and stays null until a thruster
  exists to measure. Bidirectional DShot already carries rpm; we need it to be real. Measured
  2026-09-07: 958 CRC-valid `ESC_STATUS` frames with nothing attached, **every rpm exactly 0** —
  so a frame count can never prove a thruster is alive (see [`pr-f`](pr-f-esc-presence-on-the-wire.md)).
- **Current, alongside it.** Fault classification from rpm alone tops out around 78 % in the
  literature; current is what separates a fouled prop from a dead phase. The Pico already
  decodes current and discards it (see [`pr-c`](pr-c-pico-esc-health.md)), so the wire side is
  half-built — the ask here is that the ESC actually measures it.
- ⚠ **Do not make closed-loop shaft speed the primary control mode.** Sørensen's comparison of
  thruster control modes is unambiguous: under wave-induced inflow a shaft-speed controller
  delivers wildly less thrust than a torque controller (order 18 N against 65 N in the reported
  case), because holding rpm constant while inflow changes is precisely the wrong thing to do.
  Shaft speed as *telemetry*: yes, please. Shaft speed as the *setpoint*: no.

### 2.4 A measured bollard thrust curve, at our voltage

`MOT_THST_EXPO = 0.65` is fitted to the T200. It is the curve the firmware uses to linearise
demand into thrust, and it is currently describing a thruster we will not fly.

We need, from the real unit: **thrust against command, forward and reverse, at the battery
voltage we actually run** (the vendor's own reverse ratio drifts 0.787 → 0.754 across 12–20 V,
so a single-voltage curve is not enough), plus **thrust against rpm²** to give `k_n_per_rpm2`
its first real value. A load cell and an afternoon closes two open rows in `measured-bars.md`
and one in `BUGS.md`.

### 2.5 Thrusters as identifiable bodies in the CAD

Reading the Onshape document on 2026-09-23 for `hull_geometry.yaml`, the thruster positions
came out of the tunnel bores and one axial placeholder, because the fitted units in the model
are three A2212 placeholders. Once the real unit exists, **name the bodies** — a consistent
part name per thruster is what lets `tools/pull_hull_geometry.js` re-derive geometry every time
the design moves, instead of us identifying a thruster by where it happens to sit.

And while it is on the list: ⭐ **assign materials.** Onshape reports no material on any part,
so mass, centre of mass and the inertia tensor are not merely unmeasured but *uncomputable*.
That blocks every claim in newtons, the CoB restoring term the firmware already has written,
and INDI's 6×6 effectiveness matrix. It is a dropdown per part, once the hull material is
chosen.

---

## 3. What this does not ask for

**Not more thrust.** Authority is not our limit; resolution is. A thruster with twice the peak
and the same floor makes the problem worse, because the quantum scales with the peak.

⛔ ~~**Not a faster thruster.**~~ **RETRACTED 2026-09-23 — see §7.** We argued that
because a T100-class unit's dead time (~0.59 s) already dwarfs the 2 ms control loop,
response time was not where the win is. We then built a closed-loop bench and measured it,
and the conclusion is the opposite: **thruster lag dominates everything else by one to two
orders of magnitude, and it is the single most valuable property after §2.1.** §7 gives the
design guidance, and the good news is that our tunnel geometry starts seven times ahead of a
T100 already.

**Not a redesign for fault tolerance.** The allocator handles a dead thruster as a limit
change (`u_min = u_max = 0`) and degrades rather than tumbles. We need to *know* the thruster
died — which is §2.3 — not for it to never die.

---

## 4. How we will verify each of these

| ask | how it gets checked |
|---|---|
| §2.1 smooth from zero (see §6.6) | a demand ladder on the bench with `MOT_SPIN_MIN = 0`: the output must be monotone and continuous through the first non-zero step, with no 16.8 % jump |
| §2.2 symmetry | bollard thrust at ±command, matched pairs; `REVERSE_EFFICIENCY` retires when the ratio is within measurement error of 1.0 |
| §2.3 telemetry | rpm and current must move with a *known* mechanical change (a fouled prop, a held shaft) — a non-zero reading is not evidence, per the 958-frame result |
| §2.4 curve | the fitted curve predicts a held-out thrust point; the number lands in `measured-bars.md` with voltage and method |
| §2.5 CAD | `tools/pull_hull_geometry.js` re-runs and reports `materials_assigned: true`, and identifies every thruster by name rather than by position |

---

## 5. Related

- [`pr-i-spin-min-relay.md`](pr-i-spin-min-relay.md) — the firmware-side half of §2.1: the
  relay behaviour measured and filed. If the thruster removes the need for the floor, PR I's
  ask becomes a parameter change rather than a control redesign.
- [`pr-k-the-mixer-is-for-a-different-hull.md`](pr-k-the-mixer-is-for-a-different-hull.md) —
  the mixer is written for eight T200s at 45°; the hull being built has five orthogonal units.
- [`pr-f-esc-presence-on-the-wire.md`](pr-f-esc-presence-on-the-wire.md),
  [`pr-c-pico-esc-health.md`](pr-c-pico-esc-health.md) — the wire side of §2.3.
- `src/mongla_control/config/hull_geometry.yaml` — where `k_n_per_rpm2` waits.

---

## 6. Addendum, 2026-09-23 — the unit is chosen, and it changes the asks

**Reported by the firmware team lead (Rakibul Islam):** the thruster will be a **DJI 2212
920 KV outrunner**, potted into a **3D-printed enclosure** with a **custom resin propeller**,
driven by an ESC running **Bluejay** firmware.

Datasheet, as supplied: 28 × 24 mm, 3S/4S, 8 mm shaft, 56 g, 15–25 A standard / 30 A max,
370 W max, "max thrust 1200 g". ⚠ That thrust figure is **in air, with an 8×4.5 aeroprop, at
14.7 V**. It carries no information about this vehicle. Water loads a propeller by orders of
magnitude more than air, and the number below is the one that matters instead.

### 6.1 ⛔ This motor is torque-limited for this job, and that lands squarely on our floor

Torque constant follows from KV: `Kt = 9.55 / KV = 0.0104 N·m/A`. At the 30 A absolute
maximum that is **0.31 N·m**, and at a sane continuous 15 A it is **0.156 N·m**.

A T200-class thruster absorbs roughly `Q = P / (2πn) ≈ 350 W / (2π × 3500/60) ≈ 0.95 N·m` at
full output. So the 2212 has **on the order of a sixth of that torque continuously**, and the
propeller has to be designed down to it rather than the motor up to the propeller.

⚠ **The control consequence is the one that costs us the competition run, and it is not
"less thrust".** If the propeller is sized so that ordinary cruise and station-keeping sit at
**20 % throttle**, then our measured **16.22 % minimum non-zero output** is *the whole usable
band*. Every correction the vehicle can express is a lurch, and there is no host-side gain that
changes it — `MOT_SPIN_MIN` is a floor, not a slope.

⭐ **So propeller sizing is a control-resolution decision, and it is the cheapest lever on this
page.** Size the resin propeller — diameter and pitch — so that the hull's *cruise* thrust sits
at **50–70 % of throttle**, not at 20 %. Same motor, same ESC, same firmware; the fixed
16.22 % quantum goes from being the entire operating range to being a small part of it. The
propeller is custom, so this costs a design iteration and nothing else.

Concretely, what we need from the propeller design in order to check it:
**the torque the propeller absorbs at the rpm that produces the thrust the hull needs to
cruise.** If that torque exceeds ~0.15 N·m, the propeller is too big for this motor and the
vehicle will live permanently in the bottom of the throttle range.

### 6.2 ⭐ Run the motor **wet**, with no shaft seal — it answers §2.1 directly

§2.1 asked for a thruster that turns smoothly from zero, and named shaft-seal and bearing
static friction as one of the three reasons a floor is needed at all.

A potted outrunner **flooded with water** — windings and stator conformal-coated or
epoxy-potted, rotor and bearings running wet, no dynamic seal anywhere — removes that term
entirely. There is no seal to break out of. It is also the better thermal answer for a motor
that will be run at high current and low rpm, which is exactly the regime §6.1 puts it in.

⚠ Costs, stated honestly: viscous drag from the rotor turning in water (a continuous loss,
but a *smooth* one — it is damping, not stiction, and damping is a term the controller can
model); and bearings must be water-rated (stainless or ceramic, not the stock shielded steel).

**A dry enclosure with a rotating shaft seal is the option to avoid**, because it puts back the
exact static-friction term that forces the floor we are trying to remove.

### 6.3 What Bluejay does and does not give us

**Gives us**, and this closes half of §2.3 for free:

- **bidirectional DShot with rpm telemetry** — the reason `k_n_per_rpm2` can finally be
  measured, and the reason thruster presence stops being unknowable;
- **48 kHz (and higher) PWM**, which is the setting that matters most for smooth running at the
  low duty cycles §6.1 says we will live at;
- **`STARTUP_POWER` and `RAMPUP_POWER`** as explicit settings — the knobs that decide whether
  the motor can be commanded off a standstill without a kick.

**Does not give us**, and this needs a decision:

- ⛔ **current.** Bluejay runs on BLHeli_S-class hardware, which typically has **no current
  shunt at all**. §2.3 asked for current alongside rpm because rpm alone tops out around 78 %
  fault-classification accuracy. If the chosen ESC has no shunt, that ask moves to the hardware
  side: either pick an ESC variant that has one, or accept that per-thruster current comes from
  our own sense board (H-4) after all.
- ⛔ **sensored / FOC startup.** Bluejay is sensorless six-step. §2.1's third reason for the
  floor — the ESC not knowing rotor position at rest — **stands**. Worse, sensorless
  commutation relies on back-EMF, which is weakest at low rpm, so the §6.1 operating point is
  also the point of greatest desync risk. §6.2 and §6.1 are the two mitigations available
  without changing ESC family.

⚠ **Verify before relying on it:** the exact Bluejay build, the ESC hardware it is flashed to,
and whether that hardware has a current shunt. Three facts, one afternoon.

### 6.4 The resin propeller

A photopolymer propeller is brittle and creeps under sustained load; an aeroprop failure is an
annoyance and an underwater one mid-run is the run. We are not asking for a material change —
we are asking for **a strike test and a soak test** before it is trusted: run it against a
deliberate obstruction, and leave a printed blade loaded in water for as long as a competition
day, then re-measure. If it fails either, the finding is cheap now and expensive later.

### 6.5 What this addendum changes in §2

| §  | state after this news |
|---|---|
| 2.1 smooth from zero | **partly answered** — §6.2 (wet, no seal) removes the friction term. Cogging and sensorless startup remain. ⭐ §6.1 (propeller sizing) is now the larger lever |
| 2.2 symmetric reverse | **still open, and now cheaper** — the propeller is custom, so a symmetric section is a design choice rather than a procurement one |
| 2.3 rpm + current | **half answered** — Bluejay gives rpm. Current depends on whether the ESC has a shunt; see §6.3 |
| 2.4 thrust curve | **unchanged and now urgent** — `MOT_THST_EXPO = 0.65` describes a T200. This motor and propeller will not share its shape, and §6.1 cannot be checked without the curve |
| 2.5 CAD bodies + materials | **unchanged** |


---

## 6.6 ⚠ A correction to §2.1, measured — the floor's cost is the STEP, not the jitter

Added 2026-09-23, after building a closed-loop bench: your firmware's control code
compiled natively, flying a Fossen 6-DOF plant built from this hull's own geometry.

§2.1 argues that a thruster which turns smoothly from zero is the single most
valuable property, and it is still the ask. But one of the reasons we gave was
wrong, and it is better that you hear it from us.

**What we implied:** that `MOT_SPIN_MIN`'s floor ruins precision alignment by
leaving the vehicle jittering on target.

**What we measured.** Yaw hold with the floor in the loop limit-cycles at about
**0.1° peak-to-peak**, constant regardless of the size of the step commanded,
against ~0.000° with the floor removed. That is roughly **3.5 mm at 2 m range** —
comfortably inside the pixel tolerances our vision servo already works to. The
result is independent of drag (our largest unknown) to 1e-6, and varies 70× across
the plausible thrust and mass band, so it is an order of magnitude rather than a
number.

**So the steady-state jitter is not the problem.** The problem is what happens
*before* steady state:

| the real cost | measured |
|---|---|
| the smallest non-zero output is a **lurch** | 16.2 % of full scale, from a standing start |
| the usable yaw band is narrow | **4.06 : 1** — 17.4 % minimum to 70.7 % at full stick |
| below the gate there is nothing at all | 0 % output under 2.856 % of stick |

**What this changes for the thruster design: nothing about the direction of the
ask, and something about why.** We are not asking for smooth-from-zero to reduce
jitter. We are asking for it so the vehicle has **small steps available at all** —
so a 2° correction can be a 2° correction instead of either nothing or a lurch,
and so an incremental controller has increments to work with.

⭐ **And it reprioritises §6.1.** If the smallest step is 16.2 % of full scale,
then making full scale *smaller* — by sizing the propeller so cruise sits at
50–70 % throttle rather than 20 % — shrinks the absolute size of every lurch, with
no change to motor, ESC or firmware. That is the cheapest lever on this page and
it is a propeller-sizing decision, not a control one.


---

## 7. ⭐ Designing the custom thruster for RESPONSE TIME — and it is mostly geometry

Added 2026-09-23 from closed-loop measurement. This section **replaces** the "not a faster
thruster" line in §4, which was wrong.

### 7.1 The measurement that changed our minds

Yaw hold against a 1.5 N·m torque burst, our control code on a Fossen plant, first-order
thruster lag swept:

| thruster lag τ | peak heading deviation | vs no lag |
|---|---|---|
| 0.00 s | 0.394° | — |
| 0.05 s | 1.026° | 2.6× |
| 0.10 s | 2.030° | 5.2× |
| 0.30 s | 27.76° | 70× |
| 0.59 s (T100-class) | 51.49° | **131×** |

For comparison, dropping the **control loop from 500 Hz to 50 Hz** — a tenfold change — cost
**21 %**. At τ = 0.10 s, *halving τ* buys about twice what a tenfold loop-rate increase buys.

**Response time is the dominant actuator property. Not thrust, not loop rate.**

### 7.2 What actually sets τ — two terms, and the big one is the duct

**Rotor spin-up**, `τ = J·ω / Q`. For the DJI 2212 at 920 KV, `Kt = 9.55/KV = 0.0104 N·m/A`,
so 0.104 N·m at 10 A. With a bell plus a light resin propeller, `J ≈ 5e-6 kg·m²`:

| rotating inertia | τ to 3000 rpm |
|---|---|
| bell only, 1.5e-6 | 4.5 ms |
| bell + resin prop, 5e-6 | **15 ms** |
| bell + heavy prop, 1.2e-5 | 36 ms |

**Water column**, and this is the one that dominates:

```
    τ = L · √(ρ·A / T)
```

A thruster does not make thrust by spinning — it makes thrust by throwing water backwards,
and the water in the duct has to be accelerated first. Momentum theory gives a slipstream
`v = √(T/ρA)`, and the entrained column `ρAL` has to reach it.

| configuration | bore | duct L | τ at 20 N |
|---|---|---|---|
| **lateral tunnel, as drawn** | 84 mm | 150 mm | **79 ms** |
| **vertical tunnel, as drawn** | 84 mm | 130 mm | **68 ms** |
| half-length duct | 84 mm | 75 mm | 39 ms |
| ring / nozzle only | 84 mm | 40 mm | 21 ms |
| wider bore, same length | 100 mm | 150 mm | 94 ms |

⭐ **We are already seven times faster than a T100 before anyone designs anything**, because
short tunnel thrusters are inherently quick. The 0.59 s figure that scared us is a big open
propeller, not a 130 mm tunnel.

### 7.3 The design levers, ranked

| lever | scaling | what it is worth |
|---|---|---|
| ⭐ **duct length** | `τ ∝ L` | **the biggest one, and it is a CAD change** — not a motor, ESC or firmware change. 150 → 75 mm halves the lag |
| thrust at the operating point | `τ ∝ 1/√T` | 4× the thrust for half the lag — a poor trade on its own |
| bore | `τ ∝ bore` | ⚠ easy to get backwards: a **wider** bore is **slower**, because there is more water to move |
| propeller inertia | `τ ∝ J` (rotor term only) | ~15 ms of a ~80 ms total. Keep the resin prop light, but it is not where the win is |

### 7.4 ⭐ And it does NOT conflict with §6.1

§6.1 asks that the propeller be sized so cruise sits at 50–70 % throttle rather than 20 %,
for command resolution. That is about **maximum** thrust; `τ` depends on the thrust being
produced **at the operating point**. The two asks are independent and can both be satisfied.

### 7.5 What we would ask the mechanical design to do

1. ⭐ **Keep the ducts as short as hydrodynamically acceptable.** Every millimetre of duct is
   directly proportional to lag, and lag is the dominant control term. If a 150 mm tunnel can
   become 100 mm without losing efficiency, that is 26 ms off the vehicle's response.
2. **Keep the propeller light.** Resin already helps; hollow or thin the hub if it is free.
3. **Tell us the real number.** A step-response test — command a step, log rpm and thrust
   against time, fit a first-order lag — is an afternoon with a load cell and it replaces
   every estimate on this page. ⚠ Momentum theory ignores duct-wall friction, the inlet
   contraction, and the break-away of a stopped propeller, so treat the table above as
   RANKING design choices rather than as numbers to quote.
4. **Do not widen the bore to buy thrust** without checking §7.3 — it costs response time
   linearly.
