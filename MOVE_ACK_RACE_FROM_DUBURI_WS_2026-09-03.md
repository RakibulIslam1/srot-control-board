# ⛔ A `SROT_MOVE` can report COMPLETE at 100 % while it is still running

**Repo:** `srot-control-board` · **Severity: highest we have filed.**
**File:** `src/comms/mav_stream.cpp` — `snapshot()` at `:53-86`, `updateMove()`
at `:623`
**Found by:** reading the firmware end to end from the `duburi_ws` side,
2026-09-03. Not observed in the field yet — which is the reason to fix it now
rather than after it is.

---

## The defect

`snapshot()` caches last-good values for the thruster block, and the comment
says exactly why:

```cpp
// mtx_thrusters is contended by two 500 Hz tasks, so a 3 ms miss is not rare.
// Keep the LAST good values on a miss — the previous behaviour left the
// default-constructed Snap in place and transmitted all-zero RPM, which made a
// perfectly healthy thruster read 0 in the GCS at random.
```

**The `mtx_control` block two blocks above has no such cache, and no `else`:**

```cpp
{
    StateLock lk(g_state.mtx_control, pdMS_TO_TICKS(5));
    if (lk.ok()) {
        const ControlState& c = g_state.control;
        s.armed = c.armed;
        s.mode  = (uint8_t)c.mode;
        ...
        s.mv_active = c.mv_active; s.mv_type = c.mv_type; ...
    }
    // no else — the default-constructed Snap stands
}
```

`Snap`'s member initializers are `armed = false`, `mode = 0`,
**`mv_active = false`**.

Now `updateMove()`:

```cpp
bool done = (s.mv_done_seq == s_seq) || (s_seen_active && !s.mv_active);
if (done) {
    sendMoveAck(s, MAV_RESULT_ACCEPTED, 100);
    sendNamed(now, "MV_STATE", 0);
    sendNamed(now, "MV_PROG",  1.0f);
    s_resolved = true;
    return;
}
```

So: once a move has been seen active, **a single missed 5 ms lock on a mutex
the 500 Hz control loop takes several times per cycle** makes `s.mv_active`
read its default `false`, `done` becomes true, and the board emits a terminal
`MAV_RESULT_ACCEPTED` with progress 100.

`s_resolved = true` latches it. Every later call returns at `:603`. **The move
is still running and the board will never correct itself.**

## Why this is the same failure you fixed in rev 13

Rev 13's note, verbatim:

> *"The movement state machine never consulted the arm state, so a move sent
> while disarmed ran its whole profile with the thrusters silent and finished
> `MAV_RESULT_ACCEPTED` at 100%. Measured: a 3 m FORWARD 'completed' in 3.5 s
> having moved nothing. **Consumers that sequence legs on those ACKs
> (duburi_ws) would advance an entire mission on a dead hull.**"*

Same terminal ACK, same 100 %, same consequence — except here the hull is not
dead, it is **still under way**. Our action server resolves on that ACK and
sequences the next leg, so the mission issues its next command into a vehicle
that is still executing the previous one. On a 20 kg hull with no position
estimate, that is unrecoverable state: neither side knows how far it went.

The `mv_done_seq` term does not save it — on the same lock miss `mv_done_seq`
also reads its default `0`, so that half of the `||` is false too.

## The fix

Mirror what the thruster block already does. Roughly:

```cpp
{
    static ControlState s_ctl_last;      // or just the fields Snap needs
    static bool s_ctl_valid = false;
    StateLock lk(g_state.mtx_control, pdMS_TO_TICKS(5));
    if (lk.ok()) { s_ctl_last = g_state.control; s_ctl_valid = true; }
    if (s_ctl_valid) {
        s.armed = s_ctl_last.armed;
        s.mode  = (uint8_t)s_ctl_last.mode;
        s.mv_active = s_ctl_last.mv_active;
        ...
    }
}
```

Holding the last good value is correct here for the same reason it is correct
for the thrusters: the quantity did not change, our *view* of it lapsed. And
`mv_active` going false is a **transition** the ACK machine must not infer from
a failed read.

If you would rather not cache, the narrower fix is to make the resolve require
two consecutive observations of `!mv_active`, or to trust only the
`mv_done_seq` term. We have no preference — the cache is closest to the code
already there.

## Two more consequences of the same missing `else`

**1. The heartbeat that confirms an arm can say DISARMED.**
`sendHeartbeatNow()` is called synchronously right after arm, disarm, mode
change and `SROT_MOVE` (`mav_commands.cpp:238-250, 313, 319, 362`). On a lock
miss that frame reports `armed = false, mode = 0 (STABILIZE)` — a correct-CRC
frame stating the opposite of what just happened, immediately after the command
whose result the operator is watching for.

**2. `ATTITUDE` and `VFR_HUD.heading` ship `0, 0, 0`** for that frame, from the
`mtx_sensors` block which has the same shape.

## What we will do on our side regardless

Require two consecutive agreeing frames before acting on a terminal move ACK,
and cross-check arm state rather than trusting one heartbeat. That is a
workaround for a race we cannot see from here, so we would rather not keep it —
which is the usual reason to raise it with you instead of patching around it.

**Please bump `SROT_FW_BEHAVIOUR_REV`** when this lands. We gate on the rev,
and this changes observable behaviour we are about to work around.
