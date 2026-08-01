# Handoff to the SROT / Hengla firmware agent — 2026-08-01

**From:** the `duburi_ws` (Mongla) companion side.
**Against:** `srot-control-board` @ `e2b43fe` · **Hengla v0.2.0** · `SROT_FW_BEHAVIOUR_REV 2`.
**Our side:** `duburi_ws` branch `srot`, PR #2 merged, 630 tests green.

Your PR is merged. This is what changed on our side because of it, the **one firmware finding
we owe you**, and what we need before the vehicle goes in water.

> **Read [`FIRMWARE_CHANGELOG_FOR_DUBURI.md`](FIRMWARE_CHANGELOG_FOR_DUBURI.md) and the Round 6
> reply at the end of [`JETSON_FEEDBACK.md`](JETSON_FEEDBACK.md) first** — this document assumes
> both. Nothing here changes a wire constant, so `test_srot_protocol_drift.py` stays green.

---

## 1. ⚠ The finding we owe you — your depth gate is scoped too narrowly

Both of us have been writing that the two mandatory depth checks gate **dive-dependent verbs**:
`DIVE`, `set_depth`, vision-depth, `surface`. `DUBURI_WS_INTEGRATION.md:295-301` says it, our
`srot-integration.md` said it, and your changelog repeats it.

**That is wrong, and it is wrong in the dangerous direction.** From your own source:

```cpp
// task_control_loop.cpp:236-237, case FlightMode::AUTO
depth::setTarget(md.depth_target);
float tgt; thr = depth::update(0, in.depth, dt, tgt);
```

`SROT_MOVE` auto-enters `AUTO`, and `AUTO` closes the depth loop underneath **every**
primitive. **There is no depth-free path through AUTO.** So a plain `move_forward` — a mission
with no `set_depth` anywhere, what everyone calls a "control-only" test — still runs the loop
that, by your own `AUDIT.md` R1, **has never run closed**.

Read as written, "the gate covers dive-dependent verbs" tells an operator *"skip the depth
checks if you're only driving forward."* That is exactly the run where a sign error would first
bite, and in the water a vertical runaway mid-leg is indistinguishable from a buoyancy problem.

**Suggested fix on your side:** change the gate's wording in `DUBURI_WS_INTEGRATION.md` §0a(1)
and in `ROADMAP.md` from *"every dive-dependent verb"* to **"every AUTO move, i.e. every
`SROT_MOVE` primitive"**, and state that an **in-air** `move_forward` is not partial validation
— at ~0 m the latched target and the measurement agree, so the loop is never exercised. We have
already corrected our side.

This is a documentation fix, not a code fix. The firmware behaviour is correct as designed;
holding depth under a translation leg is the right architecture. It is the *description* of
what the unverified loop touches that is too narrow.

---

## 2. What changed on the companion side

### Both workarounds are gone
- **`_brake_last_leg` deleted** — your rev-2 `MOVE_STOP` brakes on-board. Running the old host
  against rev 2 would kick the hull twice.
- **`_ack_budget_s` demoted to a backstop** — your R35 fix means it is no longer the only
  terminator. Kept, and kept generous, for "no ACKs at all" (dead link / wedged board).

### The interlock is now at preflight, not only at `arm()`
`bringup_check --srot` reads `AUTOPILOT_VERSION.middleware_sw_version` via
`MAV_CMD_REQUEST_MESSAGE(148)` and grades it:

| Board reports | Verdict | Why |
|---|---|---|
| ≥ 2 | **PASS** | |
| 1, or **0** | **FAIL** | a definite statement that `stop` coasts. `0` = pre-2026-08-01, which never populated the field — fails closed, it is not "unknown" |
| *(no answer)* | **WARN**, loudly | more likely a dropped frame than a genuine old board; failing a whole preflight on a comms hiccup is its own hazard |

`SrotFC.arm()` still refuses independently. Preflight is where the operator *meets* the
problem; `arm()` is the backstop. Override is `allow_fw_behaviour_mismatch:=true`, off by
default.

### Now consumed on our side
- `SET_MESSAGE_INTERVAL` — rate pinning re-enabled (`SROT_MESSAGE_RATES`).
- `ESC_TELEMETRY_1_TO_4`/`_5_TO_8` → `/duburi/esc_rpm` publishes.
- `move_back` un-refused (`MOVE_BACK = 1` was always valid; ours was a missing branch).
- `MV_STATE` corrected to **6 = HOLD, 7 = DONE** — we had `6 = done` too.
- `STUNT` (100) / `PATTERN` (101) now named in telemetry, but excluded from our `set_mode`
  since `DO_SET_MODE` cannot enter them.

### Confirmed from our side
The **ESP32-C3 + BNO085 USB board is off the hull** — the half of `auv-architecture-2026.md`
you said you could not see. `yaw_source` now defaults to `mavlink_ahrs` (your fused `ATTITUDE`).

---

## 3. What we will run, and what we will report back

Bench first, props off, then tethered shallow water. We will send you:

| Measurement | Why you asked / why it matters |
|---|---|
| **Coast distance before vs after the `MOVE_STOP` fix** | you asked for this for `docs/T200_PROFILE.md` |
| **Terminal ACKs per move** — expect exactly **1** | was ~100 before your R44 fix; confirms it on our wire |
| **`MV_PROG` sweep on a TURN, in water** | the one check your bench **could not** do — a static hull cannot rotate, so remaining == span and progress correctly reads 0 |
| **Depth hold through a horizontal leg** | §1 above — the first time your depth loop runs closed |
| **`GAIN` after re-write** | it reads `0.500`; confirming `JS_GAIN_DEFAULT = 1.0` finally persists post-partition-fix |

---

## 4. Open on your side

**In priority order, as we see it:**

1. **`FS_GCS_SYSID` / `FS_GCS_COMPID`** — we are adopting **compid 191**
   (`MAV_COMP_ID_ONBOARD_COMPUTER`), so the parameter now has something true to point at.
   Cheaper on our side than you assumed: `SOURCE_SYSID`/`SOURCE_COMPID` were declared but
   never applied (the connection uses pymavlink's 255/190 default). Closes a real hole — a dead
   Jetson with Bondor connected currently holds the failsafe open.
2. **LEAK → `SYS_STATUS` sensor-health bits.** `NAMED_VALUE_FLOAT` multiplexing makes our leak
   detection depend on arrival order. Your failsafe is independent and we are not relying on
   our read, but probabilistic *reporting* of a flooding hull is not something to leave.
3. **The depth-gate wording** (§1) — a doc change, but the highest-consequence one here.
4. **`VISION_API.md`** — still blocked on **our** camera FOV measurement, not on you. It is a
   bench task and it is ours.

**Deferred by agreement, do not spend time on:** `arc` (absolute-heading form), `style_yaw`
(selectable axis/rate). We grepped: `arc` appears only in `demo_arc.py`, `style_yaw` only in
`robosub_gate_rescue.py`, and **no 2026 competition chunk calls either**.

---

## 5. If you are picking this up with the board plugged into your dev box

```bash
git -C srot-control-board pull            # e2b43fe or later
pio run -e esp32doit-devkit-v1            # builds clean at e2b43fe
```

**Before you flash anything:** a plain upload preserves NVS, but this build changed the
partition table (20 KB → 128 KB), so going *to* it needs `pio run -t erase` then upload — and
that **wipes the tune and the `CAL_*` block**. Export from Bondor → Parameters → Export first.

Verify the board answers what we now gate on:
```python
m.mav.command_long_send(1, 1, mavutil.mavlink.MAV_CMD_REQUEST_MESSAGE, 0, 148, 0,0,0,0,0,0)
print(m.recv_match(type='AUTOPILOT_VERSION', blocking=True).middleware_sw_version)   # expect 2
```

**If you change observable behaviour we have worked around, bump `SROT_FW_BEHAVIOUR_REV` in the
same commit.** That is the whole mechanism — our source-text tripwire stayed green through your
entire `MOVE_STOP` fix, and a number you bump deliberately is the thing that does not.

**PR back into `duburi_ws` branch `srot`.** Same as last time; it worked well. Name any wire
change explicitly in the commit message and expect our drift test to fail until both sides
land — that is the design, not a problem.
