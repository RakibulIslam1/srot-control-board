# Blue Robotics T200 — thruster profile (ESC settings + firmware constants)

The Bluejay/BLHeli_S defaults are tuned for **5-inch quadcopter motors**. A T200 is a large,
high-inertia, water-loaded motor and needs a different profile. This page records the T200's real
numbers, the ESC settings to use, and the firmware constants that depend on them.

## T200 facts (measured/published)

| Property | Value | Source |
|---|---|---|
| **Poles / pole pairs** | **14 poles → 7 pole pairs** | Blue Robotics staff: *"The T200 has 14 poles and runs at about 3800RPM at full throttle. Electrical RPM is thus 3800*7=22600"* ([BR forum](https://discuss.bluerobotics.com/t/t200-thruster-questions-poles-max-voltage-e-bike-controller/2442)) |
| **Max RPM** | ~3075 @ 12 V · **~3600 @ 16 V** · ~3800 @ 20 V (range 300-3800) | [T200 product page](https://bluerobotics.com/store/thrusters/t100-t200-thrusters/t200-thruster-r2-rp/) |
| Voltage / current / power | 7-20 V · 24 A @ 16 V · 390 W @ 16 V | product page |
| PWM deadband (stock ESC) | 1500 µs ±25 µs | [Thruster Usage Guide](https://bluerobotics.com/learn/thruster-usage-guide/) |
| BR's own ESC | Basic ESC R3 — **BLHeli_S 16.6, 24 kHz**, layout `R_H_15` (dead time 15) | [BR firmware page](https://bluerobotics.com/learn/basic-esc-r3-firmware-files-and-customization/) |

> Blue Robotics officially drive the T200 with **PWM (1100-1900 µs)**, not DShot. Running DShot +
> bidirectional RPM telemetry (as SROT does) is off their supported path — it works, but expect to
> tune the ESC yourself, which is what this page is for.

## ESC settings — apply in esc-configurator

Steps 1-5 are **settings writes only, no reflash**.

| Setting | Bluejay default | **Use for a T200** | Why |
|---|---|---|---|
| **Max Startup Power** | 1020 | **1300** | The big one — see below |
| **Min Startup Power** | 1010 | **1100** | Enough initial torque to break rotor inertia |
| **RPM Power Protection** | 9x | **Off** | Caps power as a function of measured eRPM; a slow-accelerating heavy motor gets choked exactly when it needs power |
| **Demag Compensation** | Low | **High** | Long-demag, low-RPM, high-current load — the classic stutter cure |
| **Motor Timing** | 22.5° (MediumHigh) | **30° (High)** | BLHeli_S manual: high timing gives demagnetisation headroom and "often helps" stutter |
| Brake on stop | Off | Off | A thruster should coast |
| Temperature Protection | Disabled | ~100 °C | Sensible once it runs (see dry-running note) |

### Why startup power is the prime suspect for "judders but won't spin"
Bluejay ramps power from **min** to **max** during startup and, if the motor still hasn't spun at
max, **assumes it is stalled and gives up**. Two things make the v0.21.0 defaults hostile to a T200:

1. **Max Startup Power was *ignored* by a bug in BLHeli_S and Bluejay ≤ 0.20.0.** v0.21.0 fixed that
   — so the cap is now enforced for the first time.
2. **The v0.21.0 defaults were deliberately lowered to 1010/1020** so a stuck 5" quad motor gives up
   quickly and doesn't burn.

A T200 accelerating slowly under load looks exactly like a jammed quad prop to that heuristic.
Symptom: the motor judders and reports only a couple of hundred RPM at a healthy throttle command.

### If it still won't spin
6. **Reflash Bluejay at 24 kHz.** Dead time 30 (layout `?-H-30`) wastes duty resolution at 48 kHz;
   Bluejay's guidance is *"the higher your dead time is, the lower your PWM frequency should be"*.
   BR's own ESC runs 24 kHz.
7. **Drop to Bluejay v0.20.0** (24 kHz). The v0.21.0 release notes state: *"3d mode appears to be
   having performance issues from the bugfixes for startup protection since 0.20 — as such use of
   3d mode cannot be advised."* SROT runs the thrusters in 3D mode, so this applies directly.

> **AM32 is not an option on these ESCs.** AM32 is ARM-only; BLHeli_S hardware is SiLabs EFM8
> (EFM8BB21 here), which runs BLHeli_S/Bluejay only.

> ⚠️ **Dry running:** T200s are water-lubricated and water-cooled. Bench-testing in air is fine in
> short bursts, but sustained high power dry wears the bearings and overheats the motor. Raising
> startup power also removes the anti-stall protection — watch current on the bench.

## Firmware constants that depend on this

| Constant | Where | Value | Note |
|---|---|---|---|
| `THR_POLE_PAIRS` | `src/pico/main.cpp` | **7** | Verified correct for the T200 (14 poles). Mechanical RPM = eRPM / 7. |
| `DEF_RPM_MAX` | `include/config.h` | **3600** | The 16 V ceiling. The old 4000 was unreachable, so the loop ran permanently saturated. |
| `DEF_RPM_FF_A` | `include/config.h` | **0.000278** | Keep at ≈ `1 / RPM_MAX`. |
| `RPM_MAX` (param) | Bondor | 3600 | `task_dshot_rmt` scales stick→target RPM by this **param**, so you can match it to your battery without rebuilding. Set 3000 for 12 V, 3800 for 20 V. |

## Repeatable thrust — why RPM is NOT in the control loop

Throttle does not command thrust, it commands **volts**:

```
RPM    ~ (duty x V_batt) / Kv          thrust ~ RPM^2
```

A T200 at the **same PWM** makes **3.71 kgf at 12 V and 6.7 kgf at 20 V** — a **1.8x spread**
purely from battery state. That is why "20 % for 5 s" travels a different distance on a full
vs flat pack. But measured **thrust / RPM² is invariant to ~3 %** across that whole range, so
RPM is an excellent proxy for delivered thrust.

**The trap:** using RPM as a *setpoint inside the stabilisation loop* makes the vehicle
oscillate — a 1° disturbance produced spin-up → stop → spin-up cycling. That is architectural,
not a tuning failure: thruster dynamics are a slow nonlinear lag (Yoerger/Cooke/Slotine 1990)
and shaft-speed control is documented to induce thrust oscillation in dynamic conditions
(Smogeli/Sørensen 2009). No mainstream firmware does it — ArduPilot, ArduSub, Betaflight and
INAV all use bidirectional-DShot RPM for the **notch filter, telemetry and health only**.

So SROT gets voltage-independent thrust from two mechanisms, **both outside the fast path**:

| | What it does | Speed | Params |
|---|---|---|---|
| **Voltage feedforward** | Pre-scales duty by `1/V` so `duty x V` stays constant. Open-loop → no lag, cannot oscillate. Corrects only what the model predicts. | instant (V low-passed 0.5 Hz) | `MOT_BAT_V_MIN` / `MOT_BAT_V_MAX` |
| **RPM trim** | Compares measured RPM to expected and learns a bounded per-thruster, per-direction gain. Catches everything the model misses (fouling, weak motor, wrong curve) — **and includes the battery effect on its own**. | slow, τ ≈ 4 s | `THR_TRIM_EN` / `THR_TRIM_TAU` / `THR_TRIM_MAX` |

Either alone removes most of the battery dependence; together the feedforward does the bulk
instantly and the trim cleans up the residue.

### Two things to know before enabling them
- **`THR_TRIM_EN` defaults to 0 and must stay 0 with props in air.** An unloaded prop spins far
  faster for the same duty, so learning on the bench drives every gain to its clamp and then
  mis-scales thrust once submerged. Enable it in water.
- **`MOT_BAT_V_MAX` defaults to 0 (compensation off)** and needs the **thruster** pack voltage,
  which arrives from the 2nd board over **ESP-NOW** (`PM2`, `PM2_SRC=2`) — *not* the local ADC,
  which measures the SBC/electronics battery. Until that link is running the compensation stays
  inert. Set `MOT_BAT_V_MAX` to the pack's full-charge voltage (4S LiPo = 16.8 V).

## Low-speed resolution — firmware floor vs ESC startup power

These two solve the *same* problem (getting a loaded prop moving) and must not fight:

| Layer | Setting | Meaning |
|---|---|---|
| ESC | Bluejay **Minimum Startup Power** (e.g. 1060) | ≈1.1 % of the DShot band — where the ESC itself will break the motor away |
| Firmware (raw/test path) | **`MOT_SPIN_MIN`** (0.02) | Smallest active output the mixer will emit |
| Firmware (RPM loop) | **`MOT_SPIN_MIN`** (pushed to the Pico) | Same floor for closed-loop output |
| Firmware (RPM loop) | **`RPM_MIN_TGT`** (30 rpm) | Below this target the motor stops at 3D neutral |

`MOT_SPIN_MIN` used to default to **0.10**, which forced DShot **1173** for a *1 %* command —
measured at **941 rpm** on a T200, roughly a quarter of full speed from the smallest possible
input, with nothing usable underneath. The ESC only needed ~1.1 %. Letting the ESC's own
startup power do the break-away and keeping the firmware floor at **0.02** restores the low
end. `RPM_MIN_TGT` replaced a hard-coded 150 rpm deadband that ate the first 4.2 % of stick.

**If a thruster now hesitates to start**, raise the ESC's Startup Power first (that is what it
is for) before raising `MOT_SPIN_MIN`.

`MOT_THST_EXPO` (0.65) linearises *thrust*, but its slope at zero is ≈2.9×, so it amplifies
small commands in throttle terms. Lower it toward **0.3** if you want finer manual control and
are willing to give up some thrust linearity.

**Changing a `DEF_*` alone does nothing on a board that already has that key in NVS** — NVS survives
a firmware flash and a stored value always wins. Bump `PARAM_DEFAULTS_VER` in `config.h` to make a
build's defaults authoritative, or send `MAV_CMD_PREFLIGHT_STORAGE` with param1 = 2 to reset all
parameters to defaults at runtime. See [`ESC_FLASHING.md`](ESC_FLASHING.md) for the flashing
workflow and the wobble troubleshooting ladder.
