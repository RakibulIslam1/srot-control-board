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

---

# §7 — Payload: make the BOARD enforce the role contract (added 2026-08-03)

**Why this is on your list and not ours.** The agreed companion contract is: *duburi_ws
activates a channel; the board fires it if it is high/low configured, and REJECTS it if it
is PWM-configured for the on-board arm; either way the companion gets a success/failure
acknowledgement.* We have implemented our half. **Your half does not exist yet**, and the
gap is not cosmetic:

```cpp
// mav_commands.cpp:475-481, as shipped in rev 4
if ((int)g_params.servo_role[ch] == 2) { g_state.aux.switch_on[ch] = (p[1] >= 1500.0f); }
else                                   { g_state.aux.servo_us[ch]  = (uint16_t)p[1]; }  // ← MOVES THE ARM
g_state.aux.dirty = true;
return MAV_RESULT_ACCEPTED;                                                             // ← always
```

`DO_SET_SERVO` on a role-1 channel **moves the manipulator arm and reports success.** There
is no result value that distinguishes "fired the payload" from "swung the arm" from "did
nothing because the channel is disabled" from "the PCA9685 is unplugged". Right now the only
thing preventing a mission `fire()` from driving the arm mid-drop is a **host-side** role
read we do before every shot — which is us doing your safety interlock over a lossy link.

### §7A — `MAV_CMD_SROT_PAYLOAD_FIRE`, a NEW command id (please: 31001)

**Do not add role-rejection to `DO_SET_SERVO` (183).** Moving a servo on a role-1 channel is
a legitimate operation — it is how the arm and the joystick work — so rejecting it there
would break arm control. A separate id is what lets "payload fire" have stricter rules than
"set a servo", which is the actual requirement.

| param | meaning |
|---|---|
| `param1` | channel, **1..16**, same 1-based convention as `DO_SET_SERVO`, same n as `SERVO{n}_ROLE` |
| `param2` | pulse milliseconds; `0` = board default (suggest 800 ms) |

| condition | result | note |
|---|---|---|
| `role == 2` (switch) | `MAV_RESULT_ACCEPTED` | latch ON, **board-side one-shot timer**, auto-clear to OFF |
| `role == 1` (PWM/arm) | `MAV_RESULT_DENIED` + `STATUSTEXT` | **this is the contract**; safe here because arm/joystick never use this id |
| `role == 0` (disabled) | `MAV_RESULT_DENIED` | today this is a silent ACCEPTED-and-nothing-happens |
| channel out of 1..16 | `MAV_RESULT_DENIED` | matches `:462` |
| `mtx_aux` miss | `MAV_RESULT_TEMPORARILY_REJECTED` | matches your existing convention at `:464` |

**The board-side auto-clear is the point, not a convenience.** Today the host must send the
OFF itself, and if that command is lost the coil stays energised forever — see §7B.

Bump `SROT_FW_BEHAVIOUR_REV` to **5** in the same commit. We gate on the rev, never by probing
with a fire, so an un-bumped rev means we keep using the old path and your work goes unused.

### §7B — No failsafe de-energises a payload channel

`switch_on[]` is a plain bool that nothing ever resets (`state_types.h:242-247`); only a
reboot clears it. Grep confirms `safety_monitor.cpp`, `arming.cpp` and
`task_control_loop.cpp` never touch `AuxState`. **A leak, a disarm, or GCS loss leaves an
energised solenoid energised** while the vehicle surfaces. Please clear all role-2 channels
on the leak latch and on the failsafe auto-disarm paths.

### §7C — `pca9685_aux::healthy()` cannot fail

`begin()` sets `s_ok = true` with no I2C probe and no ACK check (`pca9685_aux.cpp:13-21`), so
`healthy()` always returns true and the `if (!s_ok || !s_pwm) return;` guard at `:26` can
never fire. **A physically disconnected expander ACKs exactly like a working one.** Please
read back `MODE1` after `setPWMFreq`, re-probe periodically in the 30 Hz service, and surface
it (a `SYS_STATUS` sensor bit, or a `NAMED_VALUE_FLOAT "AUXOK"`). Then we can report
NOT_READY instead of a confident FIRED.

### §7D — optional: any payload readback at all

`g_state.aux` is never streamed. With §7C that is survivable; without it there is no way for
anyone to know a channel actually latched, or to notice one stuck ON.

---

## Two things we fixed on our side that touch your wire

1. **`ACK_TEMPORARILY_REJECTED` was 3 in `srot_protocol.py`; it is 1.** Verified against your
   own `lib/mavlink/common/common.h:1151`. `3` is `MAV_RESULT_UNSUPPORTED`. So every real
   mutex-miss you returned was read by us as non-terminal and burned the full ACK budget into
   a bogus `TIMEOUT` + brake, and every `UNSUPPORTED` you returned was reported to the
   operator as "board busy, safe to retry". **No wire change on your side** — we were
   misreading a correct answer. `test_srot_protocol_drift.py` now parses `MAV_RESULT_*`
   straight out of your `common.h` so this cannot drift again.

2. **`config.h:245` is misleading and cost us real time.** It says *"Edit `PCA_CH_ROLE` below
   to match how your expander is wired"*, but `PCA_CH_ROLE_INIT` (`:268-271`) has **no
   consumer anywhere** — roles come only from `SERVO{n}_ROLE`. It reads like a fixed
   1-8/9-16 wiring rule, which is exactly the folklore we just removed from our side. Please
   delete it or mark it dead.

---

# §8 — MOTOR_DETECT fixed by duburi_ws (2026-08-06) — NEEDS FLASH + IN-WATER RUN

**This one is ours, not a request.** The change is written, builds clean
(`esp32doit-devkit-v1`, RAM 24.3 %, Flash 29.6 %) and is **not flashed** — the board is in
a hull mid-testing and flashing is your call with it in hand.

### What went wrong on the vehicle

In-water session: every joystick axis reversed in MANUAL, and STABILIZE flipped the hull.
Root cause was a globally inverted thrust sense **plus** two thrusters (1 and 8) inverted
*relative to their groups*. The operator reversed all eight `MOT_n_DIRECTION`, which fixed
the global sense but — necessarily — could not fix a *relative* asymmetry: multiplying
every element by −1 cannot change which elements differ from each other. STABILIZE then
span in yaw (motor 1 carries yaw) and autotune reported `RollRate … CLAMP` /
`PitchRate FAIL: period unstable` (motor 8 carries roll+pitch, halving both and
cross-coupling heave into them).

### The two firmware faults

Both in `calibration.cpp`, `CalRoutine::MOTOR_DETECT`, DETECT phase.

**1. It overwrote where it had to compose.** `driveTestMotor()` sets `test_override`, and
`task_control_loop.cpp:789` drives that through `oneToDshot(test_throttle, in.dir[motor])`
— so the pulse goes through the *existing* direction, and what you measure is an
**agreement**, not an absolute direction. With `c = CAL_MDIRn`, `p = MOT_n_DIRECTION`,
`s` = the thruster's intrinsic sign:

```
agreement      = c·p·s
OVERWRITE  c' = c·p·s          -> effective c'·p = c·s  -> result = c
COMPOSE    c' = c·(c·p·s) = p·s -> effective c'·p = s    -> result = +1
```

The old form converges **only if `c` was already +1**, and otherwise locks the existing
error in permanently — no number of detect runs can fix a thruster that starts inverted.
That is exactly why motors 1 and 8 stayed wrong on this hull while the other six were
fine. Composing is correct from any starting state in one pass, and is idempotent.

**2. An inconclusive detect wiped the calibration.** `dir` was initialised to `1` and
stored unconditionally, so a run that measured nothing — in air, or a restrained hull,
where 0.30 throttle never crosses the 0.05 rad/s gate — silently reset all eight thrusters
to `+1`, reported SUCCESS, and was persisted to flash. A calibration routine must not
destroy a good calibration as its failure mode. It now leaves the stored value untouched
and calls `finish(CalResult::FAIL)`, which also keeps `persist_pending` clear so a failed
run cannot reach flash.

`SROT_FW_BEHAVIOUR_REV` bumped **5 → 6**. Nothing for the companion to adapt to (duburi_ws
never runs MOTOR_DETECT), but it changes what an operator should expect from Bondor: a
detect can now legitimately report FAIL, and that is the routine working correctly.

### What we ask of you

1. **Review the reasoning above**, particularly the compose algebra. It was worked through
   analytically and verified against the observed hull behaviour, but it was **not**
   reviewed by a second pair of eyes before writing — please treat it as needing yours.
2. **Flash it**, then run **MOTOR_DETECT in water, armed**. It should converge every
   thruster in one pass. After that nobody should ever hand-set `MOT_n_DIRECTION` again —
   that param goes back to being a pure operator override, which is what it is for.
3. Note the erase/NVS caveat still applies: flashing loses the `CAL_*` block, so take a
   Bondor param export first. Losing `CAL_MDIR` is now harmless *provided* step 2 is run.

### Interim state on the vehicle (rev 5, unflashed)

We set `MOT_1_DIRECTION = -1` and `MOT_8_DIRECTION = -1` live and confirmed by readback,
giving a uniform effective `[-1] × 8`. **Runtime only, deliberately not saved** — a power
cycle reverts it. That keeps the hull flyable now; the firmware fix is what makes it stop
happening.
