# Executable task list for the Hengla firmware agent — 2026-08-01

**You have the board plugged into your dev box. This is the run sheet.**

**From:** the `duburi_ws` (Mongla) companion side · branch `srot` @ `41318e7`, 631 tests green.
**Against:** this repo @ `ced6dc9` · **Hengla v0.2.0** · `SROT_FW_BEHAVIOUR_REV 2`.

Work top to bottom. **§0 is a five-minute read-only check and everything else depends on it.**
When you are done, **open a PR into `duburi_ws` branch `srot`** — same as last time, it worked
well.

> Background, if you want the reasoning rather than the steps:
> [`DUBURI_WS_HANDOFF_2026-08-01.md`](DUBURI_WS_HANDOFF_2026-08-01.md) (what we changed and why),
> [`JETSON_FEEDBACK.md`](JETSON_FEEDBACK.md) Round 6 reply (our answers to your three questions).

---

## §0 — Confirm the board on your bench (READ-ONLY, do this first)

Everything below assumes the flashed board is at behaviour rev 2. Confirm it rather than
trusting the build you *think* is on there:

```python
from pymavlink import mavutil
m = mavutil.mavlink_connection('COM19', baud=115200)   # your port
m.wait_heartbeat()

m.mav.command_long_send(1, 1, mavutil.mavlink.MAV_CMD_REQUEST_MESSAGE, 0, 148, 0,0,0,0,0,0)
av = m.recv_match(type='AUTOPILOT_VERSION', blocking=True, timeout=3)
print('behaviour rev =', av.middleware_sw_version)          # MUST print 2
```

| Result | Meaning | Do |
|---|---|---|
| `2` | current | continue to §1 |
| `1` | pre-2026-08-01 behaviour | **reflash — see §0a**, this board coasts on `MOVE_STOP` |
| `0` | firmware older than the field itself | **reflash — see §0a** |
| *nothing* | not answering `REQUEST_MESSAGE(148)` | your build predates `b213e50`; **reflash** |

Also read back, while you are connected — this one has been wrong on the hardware all along:

```python
# GAIN rides NAMED_VALUE_FLOAT; loop until you see name == 'GAIN'
```
It reads **`0.500`** today. `JS_GAIN_DEFAULT = 1.0` never persisted through the full-NVS era
(R14), so **`MANUAL_CONTROL` has been running at half authority** — every pilot input through
Bondor, and our `manual()` streamed primitive. Re-write it and confirm it reads `1.0`.

*(It does **not** affect `SROT_MOVE`: `pilotGain()` is applied only in the `MANUAL_CONTROL`
handler at `mav_commands.cpp:650-657`; `movement.cpp` takes `s_speed` straight from p3. We
checked, so you do not have to.)*

### §0a — If you must reflash, the order is not optional

The partition table changed (NVS 20 KB → 128 KB), so this needs `erase` **then** upload, and
that **wipes the tune AND the `CAL_*` block**:

```
1. Bondor -> Parameters -> Export        <- the ONLY copy of CAL_* that survives
2. pio run -t erase && pio run -t upload
3. Bondor -> Parameters -> Import
4. re-write JS_GAIN_DEFAULT = 1.0, confirm streamed GAIN reads 1.0
5. re-run §0 -- behaviour rev must now read 2
```

---

## §1 — ⚠ Doc fix: the depth gate is scoped too narrowly (highest consequence here)

**This is the one finding we owe you, and it is the only item on this list that can hurt
someone.** It is a wording change, not a code change — the firmware behaviour is correct.

Both repos have been writing that the two mandatory depth checks gate **dive-dependent verbs**:
`DIVE`, `set_depth`, vision-depth, `surface`. Your own source disagrees:

```cpp
// task_control_loop.cpp:236-237, case FlightMode::AUTO
depth::setTarget(md.depth_target);
float tgt; thr = depth::update(0, in.depth, dt, tgt);
```

`SROT_MOVE` auto-enters `AUTO`, and `AUTO` closes the depth loop underneath **every**
primitive. **There is no depth-free path through AUTO.** So a plain `move_forward` — a
"control-only" test, no `set_depth` anywhere — still runs the loop that `AUDIT.md` R1 says has
**never run closed**.

Read as written, the current wording tells an operator *"skip the depth checks if you're only
driving forward"* — which is exactly the run where a sign error bites first, and in water a
vertical runaway mid-leg is indistinguishable from a buoyancy problem.

**Change, in `DUBURI_WS_INTEGRATION.md` §0a(1) and `ROADMAP.md`:**
- from *"gates every dive-dependent verb"* → **"gates every AUTO move, i.e. every `SROT_MOVE`
  primitive, `move_forward` included"**
- add: **an in-air `move_forward` is NOT partial validation** — at ~0 m the latched target and
  the measurement agree, so the loop is never exercised.

Already corrected on our side (`README.md`, `srot-integration.md`, `CLAUDE.md`).

---

## §2 — `FS_GCS_SYSID` / `FS_GCS_COMPID` — **unblocked, our half is shipped**

You said this could not be built while the companion and your LoRa bridge were both `255/190`.
**We have moved to `255/191`** (`MAV_COMP_ID_ONBOARD_COMPUTER`) — shipped in `41318e7`,
`srot_protocol.SOURCE_COMPID = 191`, applied on the srot path only.

So the parameter now has something true to point at. What we ask for:

- `FS_GCS_SYSID` / `FS_GCS_COMPID` params naming the source whose liveness the failsafe tracks.
- Default them to **255 / 191** so the shipped behaviour is correct out of the box.
- Keep the current "any non-self heartbeat" path as the fallback when they are unset/zero, so
  an operator with an older companion is not stranded.

**Ordering is safe in both directions and we checked before shipping:** you count any heartbeat
whose id is not your own (`msg.compid != MAV_COMPONENT_ID || msg.sysid != MAV_SYSTEM_ID`,
`mav_commands.cpp:687`), so `191` feeds today's failsafe exactly as `190` did. Nothing breaks
while you build this.

**The failure it closes:** a dead Jetson with Bondor still connected currently holds the GCS
failsafe open, so the vehicle station-keeps at depth when it should surface.

---

## §3 — LEAK off `NAMED_VALUE_FLOAT` → `SYS_STATUS` health bits

pymavlink caches exactly **one message per msgid**. With `MV_STATE` / `LEAK` / `WTEMP` / `GAIN`
all multiplexed onto `NAMED_VALUE_FLOAT`, whichever arrived last wins — so whether we ever
observe a leak depends on arrival order. **Leak detection is currently probabilistic.**

Your failsafe is independent (`task_control_loop.cpp:607`) and `arming.cpp:32` refuses to arm
into a leak, so the *vehicle* is protected and we are not relying on our read. But probabilistic
*reporting* of a flooding hull is not something to leave in place.

Move it to a `SYS_STATUS` sensor-health bit: a latched state that cannot be overwritten by a
temperature reading, which is what a leak actually is.

---

## §4 — Optional, if you have time after §1–§3

- **`MV_PROG` on TURN in water** — you flagged this as unproven because a static bench cannot
  rotate. We will measure it and send the trace; nothing needed from you unless it looks wrong.
- **Bump `SROT_FW_BEHAVIOUR_REV` to 3** if any of §2/§3 changes observable behaviour we might
  have worked around. That mechanism is the whole reason our source-text tripwire failure
  cannot repeat — use it liberally.

**Explicitly NOT wanted, do not spend time on:** `arc` (absolute-heading form) and `style_yaw`
(selectable axis/rate). We grepped: `arc` appears only in `demo_arc.py`, `style_yaw` only in
`robosub_gate_rescue.py`, and **no 2026 competition chunk calls either.**

---

## §5 — What changed on the companion side (so your PR does not re-fix it)

| Change | State |
|---|---|
| `_brake_last_leg` | **deleted** — your rev-2 `MOVE_STOP` brakes on-board |
| `_ack_budget_s` | demoted to a backstop after your R35 fix |
| Behaviour-rev interlock | at **preflight** (`bringup_check --srot`) **and** in `arm()`; `0`/`1` fail closed, no-answer warns |
| `SET_MESSAGE_INTERVAL` | consumed — `SROT_MESSAGE_RATES`, ATTITUDE ~11 → ~50 Hz |
| `ESC_TELEMETRY_1_TO_4`/`_5_TO_8` | consumed — `/duburi/esc_rpm` publishes |
| `move_back` | un-refused (`MOVE_BACK = 1`; ours was a missing branch) |
| `MV_STATE` | corrected to **6 = HOLD, 7 = DONE** (we had `6 = done` too) |
| `STUNT`/`PATTERN` | named in telemetry, excluded from `set_mode` (`DO_SET_MODE` cannot enter them) |
| compid | **255/191** on srot |
| `yaw_source` | defaults to `mavlink_ahrs` — the USB ESP32-C3 + BNO085 is **off the hull**, confirming the half of `auv-architecture-2026.md` you could not see |

---

## §6 — When you PR back into `duburi_ws` branch `srot`

- **Name any wire change explicitly in the commit message.** Expect
  `test_srot_protocol_drift.py` to fail until both sides land — that is the design, not a
  problem.
- **Bump `SROT_FW_BEHAVIOUR_REV` in the same commit** as any behaviour change a partner may have
  worked around. Our source-text grep stayed green through your entire `MOVE_STOP` fix; a
  number you bump deliberately is the thing that does not.
- Keep `srot_protocol.py` as our single copy — if you change a constant, change it there too
  and say so.
- Say what you could **not** verify. Last time that was the `rclpy` suites and it was the most
  useful line in the PR: we ran them and they passed, but only because you flagged it.

**After your PR merges we go to water.** The gate is §1's two bench checks, and everything else
on this list is either shipped or scoped so it does not block that.
