# Bench findings from `duburi_ws` — 2026-08-02, board in the AUV on our dev box

**From:** the companion side. **Board:** `srot-control-board` @ `d386dc8`,
`SROT_FW_BEHAVIOUR_REV 4`, read over USB serial at 115200.
**Conditions:** in the vehicle, disarmed, thrusters powered off, read-only.
**Nothing was armed and nothing was commanded.**

> **🛑 The water test cannot run tomorrow as written.** Two faults below block it, and the
> first one explains your arming blocker completely. Both are visible from a 6-second
> read-only probe, so please confirm them on your side before anything goes near water.

---

## 1. The Bar30 is producing NOISE — and the board reports it HEALTHY

30 `SCALED_PRESSURE2` samples over 6 s, vehicle still on a bench:

```
press_abs      317 .. 874 mbar     (sea level ~1013; you measured 983-990 last session)
WTEMP            6 .. 30 C         (you measured 28.63-28.66, spread 0.03 C)
VFR_HUD.alt   +0.9 .. +6.8 m       (in air)
SCALED_IMU2.temperature  0.15 .. 30.4 C

SYS_STATUS ABSOLUTE_PRESSURE health bit:  SET   <- "healthy"
SCALED_PRESSURE2 and WTEMP:               both still streaming
```

This is **not** an offset and **not** a drift. It is per-sample garbage — the signature of a
bad I2C read or a loose connector, and it matches your own closing note: *"the Bar30 stopped
responding on the last check of this session... I believe this is the sensor connector, not
firmware."* We think you were right, and it has since gone from silent to noisy.

**The part we think is worth a firmware change:** rev 3's protection cannot see this.

You validate each sample against a deliberately wide plausibility band (`[300, 40000]` mbar,
`[-5, 60] °C`), chosen loose on purpose so a judgement call cannot ground the vehicle. **Every
one of those readings is individually inside the band.** So `SCALED_PRESSURE2` keeps streaming,
`WTEMP` keeps streaming, the health bit stays set, and `DEPTH_HOLD`/`AUTO` stay available —
while the derived depth wanders six metres.

A per-sample band is structurally blind to variance. **A jitter/variance check would catch it**
— e.g. reject when peak-to-peak over the last N samples exceeds a few mbar while the vehicle is
stationary. We have added exactly that host-side (`bringup_check --srot` →
`_baro_noise_verdict`, peak-to-peak over a 6 s window) because we get many samples and your
pre-arm check sees one. But the board is the better place for it: **you can refuse `AUTO`; we
can only refuse to arm.**

## 2. Your arming blocker — this is the cause, and it is fully in your source

You reported: *"on arming, props off, nothing commanded, the four VERTICAL thrusters spun to
~3000-3180 RPM while the horizontals sat at a correct 85-170 RPM idle,"* and listed five things
to read out. Here is the readout, taken **disarmed**:

```
DEPTH_ERR   -3.0 .. -6.7 m        (phantom depth from finding 1)
DEPTH_OUT   -1.00                 SATURATED, full scale
DEPTH_CMD   -1.00
MIX_VERT    -1.00     MIX_VSGN 4
```

And `mixer.cpp` closes it. The matrix is **block-diagonal**: motors 5-8 (vertical) are non-zero
only in **roll, pitch, throttle**; motors 1-4 (horizontal) only in yaw/forward/lateral. The
throttle column is `-1` for all four verticals. So a heave demand of `-1.0` becomes `+1.0` on
every vertical and `0` on every horizontal the moment outputs go live:

> **verticals at full, horizontals at idle** — your symptom exactly, with nothing left over.

Against your own five diagnostic questions:

| # | Your question | Answer from the bench |
|---|---|---|
| 1 | Which mode does it arm into? | It sits in **SURFACE** (custom_mode 9). But the mode is not the cause — see 2. |
| 2 | Is the depth target latched on the disarmed→armed edge? | **Yes, and correctly.** `task_control_loop.cpp:294` runs `depth::reset(in.depth)`. This is not the bug. |
| 3 | `DEPTH_ERR` / `DEPTH_OUT` while armed | **Saturated at -1.00 while DISARMED**, before arming is even involved. Above. |
| 4 | `MOT_SPIN_ARM`? | Agreed, not it — it applies to all eight and the horizontals idled correctly. |
| 5 | Why all eight went to 0 on a `+Z` | Still unexplained. We have not reproduced it and did not arm. Please treat as open. |

**So the chain in your handoff now reads:**

```
depth error -> heave demand -> MIXER -> motor output -> MOT_n_DIRECTION -> spin
   POISONED      CLOSED        CLOSED     (untested)      (untested)
```

The controller sign and the mixer sign you verified are both **correct**. That is precisely why
this was so confusing: the demand path is right, so a bad *input* propagates cleanly to full
thrust. Fix the barometer and this symptom should disappear — which is also the cheapest way to
confirm the diagnosis.

**We have added a host-side arm guard** (`SrotFC.check_depth_loop_settled`): we refuse to arm
while `|DEPTH_OUT| >= 0.90`, quoting the numbers. It is a backstop for our own operators, not a
substitute for the board refusing.

## 3. Smaller things, confirmed on the wire

- **`GAIN` still reads 0.500.** `JS_GAIN_DEFAULT` has still not persisted. Every
  `MANUAL_CONTROL` input — Bondor piloting and our `manual()` — is at half authority.
- **`MAGACC` reads 1.0**, matching your known-open note. Harmless per your explanation.
- **`ESC_STATUS` (291) arrives at ~9.9 Hz and pymavlink still cannot decode it** — it shows as
  `UNKNOWN_291`. We read `ESC_TELEMETRY_1_TO_4`/`_5_TO_8` instead, exactly as designed. No
  action wanted; just confirming the workaround holds on real traffic.
- **Both `BATTERY_STATUS` instances arrive at 2 Hz.** **UPDATE, same evening:** the operator
  corrected the PM1 pin number in the SROT parameters, and PM1 now reads **13.95 V** (it was
  1.35 V, which we had both been attributing to the unwired GPIO36 — it was a parameter, not
  the wiring). PM2 reads 14.62 V. Both packs are now sensible.
  This exposed a bug **on our side**, not yours:
  pymavlink caches one message per *msgid*, so our reader was alternating between the two
  packs. Fixed by de-multiplexing on `id`. Flagging it because if you ever add a third
  instance, anyone sampling naively will hit the same trap.
- **Message rates, measured:** `NAMED_VALUE_FLOAT` 28.9 Hz (20 distinct names), `ATTITUDE`
  9.7 Hz, `SCALED_IMU2` 9.7 Hz, `SCALED_PRESSURE2`/`VFR_HUD`/`ESC_TELEMETRY_*` 4.9 Hz,
  `SYS_STATUS`/`BATTERY_STATUS` 2.0 Hz, `POWER_STATUS` 1.0 Hz, `HEARTBEAT` 0.93 Hz.
  Zero `BAD_DATA` frames in 15 s.

## 4. The USB link wedged once, and it may be worth knowing about

Before this session the CH340 enumerated correctly (`1a86:7523`, `ch341` bound, 12 M) but every
`open()` returned **EIO**, from inside our container *and* from the host, with no process
holding the port. Only a physical replug cleared it. That is the same class of symptom you saw
on the ground-station board ("USB CDC dropping, re-enumerating"). We have no diagnosis and are
not asking for one — recording it in case it recurs, because it looks exactly like a dead board
and it is not.

## 5. What we changed on our side this round

| Change | Where |
|---|---|
| `ros2 run duburi_manager connect` — new tool, opens the serial link and prints everything the board sends | `duburi_manager/srot_connect.py` |
| `BATTERY_STATUS` de-multiplexed by instance id; `get_batteries()` added | `SrotFC.note_battery` |
| Arm refused while the depth loop is saturated | `SrotFC.check_depth_loop_settled` |
| Barometer variance check + disarmed depth-loop check in preflight | `bringup_check --srot` |
| Verbose periodic `[SROT ]` telemetry block in the manager | `srot_telemetry_period_s` |
| `FW_BEHAVIOUR_REV` 4 recorded; **`FW_BEHAVIOUR_REV_REQUIRED` stays 2** | `srot_protocol.py` |

**Absence is rendered `--` everywhere, never `0.0`** — your rev-3/4 suppression contract is
honoured end to end.

## 5b. Payload — we now read `SERVO{n}_ROLE` and refuse anything that is not a switch

Your handoff said the channel role is firmware's, not ours, and that we should address
channels 0-15 rather than keep a wiring map. Done, and taken one step further: **we read the
role off the board and refuse to drive a PWM channel at all.**

Read live from the vehicle: **channels 1-8 = `SERVO` (role 1), 9-16 = `SWITCH` (role 2).**

`payload_fire_map` is now plain numbers (`"1:9, 2:10"`), and `fire()` fails **closed** — an
unreadable role refuses, because a timed-out param read is not evidence a channel is safe to
drive. The reasoning is exactly yours: a host-side role map goes stale silently on a re-role,
and the failure mode is driving the manipulator arm during a payload drop.

Verified on the board with nothing armed and nothing actuated: a `fire()` aimed at PCA 3 was
refused and no `DO_SET_SERVO` went out; PCA 9 was accepted by the gate.

**Nothing needed from you here** — this is us conforming to the contract you specified. Worth
knowing only because if you ever change the default role layout, our refusals will follow it
automatically rather than needing a host-side edit.

## 6. What we are asking for

1. **Confirm the Bar30 noise on your side** and treat it as hardware until proven otherwise.
2. **Consider a variance/jitter gate** on the barometer, because the per-sample band cannot see
   this failure and it is the one that reaches the depth loop.
3. **Re-run your arming test only after the barometer is fixed** — we expect the vertical
   spin-up to be gone, and that is the confirmation.
4. Tell us if item 5 in the table (all eight to 0 RPM on `+Z`) reproduces; we have not seen it.

**The water test should not go ahead until 1-3 are resolved.** Everything else — the command
sheet, the host stack, the telemetry — is ready.
