# PR D — `REQUEST_MESSAGE` ACKs everything and sends seven things

**Repo:** `srot-control-board` · **Severity:** medium (protocol correctness), with
two absences behind it that are architectural
**Evidence:** live probe against `Hengla v0.2.0`, behaviour rev 14, on
`/dev/ttyUSB0` at 115200, 2026-09-03. Tool: `tools/srot_mav_probe.py` in
`duburi_ws` (`--messages`), which is in the PR branch and reproduces this in
~45 s.

---

## The defect

`MAV_CMD_REQUEST_MESSAGE` (512) returns `MAV_RESULT_ACCEPTED` for **every**
message id and emits **seven**.

```
requested   190 message ids, one-shot, 220 ms apart
emitted       7   30 ATTITUDE · 74 VFR_HUD · 77 COMMAND_ACK · 116 SCALED_IMU2
                 137 SCALED_PRESSURE2 · 148 AUTOPILOT_VERSION · 251 NAMED_VALUE_FLOAT
ACCEPTED    183   including HEARTBEAT, SYS_STATUS, PARAM_VALUE, GPS_RAW_INT …
                 and ids -1 and -2, which are not messages at all
```

Accepting `-1` is the part that makes this unambiguous. There is no reading of
the spec under which the board can produce message `-1`, so this is not a
question of which messages are supported — the parameter is never examined.

`mav_commands.cpp:567` falls through to `MAV_RESULT_ACCEPTED` without
consulting `p1`.

## Why we are raising it rather than working around it

Because it is the exact failure shape this system has already paid for twice,
and both times the fix was made here:

* pre-rev-13, a disarmed `SROT_MOVE` returned success and did not move. You
  fixed it in rev 13 precisely because *we* would advance a mission on a dead
  hull.
* a `MAV_CMD_SROT_VISION` (31001) call returns `UNSUPPORTED` — correctly — and
  that correct refusal is what let us build the host side against a consumer
  that does not exist yet, without ever believing it worked.

`REQUEST_MESSAGE` is currently the first behaviour. A companion that asks for
`SYS_STATUS` on demand gets ACCEPTED, waits, times out, and has no way to tell
"this firmware cannot send that" from "the link dropped it". We will guess, and
we will guess wrong in the direction of blaming the link.

## Requested change

Return `MAV_RESULT_UNSUPPORTED` when `p1` is not a message this build can
produce. The allowlist is already implicit in whatever `REQUEST_MESSAGE` does
emit; making it explicit is a switch and a default:

```cpp
case MAV_CMD_REQUEST_MESSAGE: {
    const uint32_t id = (uint32_t)p[0];
    switch (id) {
        case MAVLINK_MSG_ID_ATTITUDE:
        case MAVLINK_MSG_ID_VFR_HUD:
        case MAVLINK_MSG_ID_SCALED_IMU2:
        case MAVLINK_MSG_ID_SCALED_PRESSURE2:
        case MAVLINK_MSG_ID_AUTOPILOT_VERSION:
        case MAVLINK_MSG_ID_NAMED_VALUE_FLOAT:
            mav_stream::sendOnce(id);
            return MAV_RESULT_ACCEPTED;
        default:
            return MAV_RESULT_UNSUPPORTED;   // we cannot make that one
    }
}
```

Cost: one switch. No wire change, no new message, no behaviour-revision bump
needed on our side — we read the ACK, and an honest `UNSUPPORTED` is
immediately more useful than a dishonest `ACCEPTED`.

---

## Two absences found by the same probe

Reported here rather than as separate issues because they were measured in the
same run and they trade off against each other.

### 1. The SD log cannot be retrieved over the link

```
LOG_REQUEST_LIST (117)        -> silent, no LOG_ENTRY
FILE_TRANSFER_PROTOCOL (110)  -> silent
AUTOPILOT_VERSION.capabilities = 8206
   = MAVLINK2 | COMMAND_INT | MISSION_INT | PARAM_FLOAT
   FTP (32) is NOT advertised, and the probe confirms it is not there
```

`src/drivers/sd_log.{h,cpp}` writes a packed `Record` every 50 ms with a
`"SROTLOG1"` header. That log exists and **cannot be got off the vehicle
without pulling the card.** Between dives, at a competition, that is the
difference between a log that gets read and one that does not.

Either half solves it, and we would rather have whichever is cheaper for you:

* **`LOG_REQUEST_LIST` / `LOG_REQUEST_DATA`** — the standard three-message
  protocol, no FTP stack, ~150 lines. Advertise nothing new.
* **MAVFTP** — more code, but you also get param and config file transfer.

We will write the host side either way; `duburi_ws` already has the .tlog
recorder and replay tool that this would feed into (see PR B).

### 2. There is no common clock

```
TIMESYNC (111) -> silent, no reply
SYSTEM_TIME (2) -> never sent
```

Board time is `time_boot_ms`; host time is wall clock. Nothing relates them, so
a host log and a board log of the same dive **cannot be put on one axis** —
which is most of the value of having both. `TIMESYNC` is a request/response
pair with two int64 fields:

```cpp
case MAVLINK_MSG_ID_TIMESYNC: {
    mavlink_timesync_t ts; mavlink_msg_timesync_decode(&msg, &ts);
    if (ts.tc1 == 0) {                       // a request; answer it
        mav_stream::sendTimesync(micros() * 1000LL, ts.ts1);
    }
    break;
}
```

That is the whole implementation. The requester computes offset and round-trip
delay from `ts1`, `tc1` and its own receive time.

**Ranking, if you only take one thing from this PR:** the `UNSUPPORTED` fix is
five minutes and removes a class of wrong diagnosis. `TIMESYNC` is ten lines
and makes two logs into one timeline. Log download is the biggest and the one
we can most easily live without for another round.

---

## What already works, so you know what we are building on

Measured in the same run, all good:

| probe | result |
|---|---|
| `REQUEST_AUTOPILOT_CAPABILITIES` (520) | ACCEPTED, `AUTOPILOT_VERSION` arrives |
| `GET_MESSAGE_INTERVAL` (510) | ACCEPTED, `MESSAGE_INTERVAL` arrives |
| `PARAM_REQUEST_LIST` (21) | **239 parameters** streamed |
| `MISSION_REQUEST_LIST` (43) | `MISSION_COUNT` returned |
| `SET_MESSAGE_INTERVAL` (511) | ACCEPTED; `ATTITUDE` held at 43.5 Hz measured |

`GET_MESSAGE_INTERVAL` working is worth calling out: it means a companion can
**verify a stream rate took** instead of assuming. We are adding that on our
side this round — it is the same "set it, then read it back" discipline that
caught a mode change silently not applying.

`UNSUPPORTED` on `REQUEST_PROTOCOL_VERSION` (519), `DO_SEND_BANNER` (42428),
`REQUEST_FLIGHT_INFORMATION` (264) and `REQUEST_CAMERA_INFORMATION` (521) is
correct and needs no change — those are the negative controls that make the
183 false ACCEPTEDs above meaningful rather than an artefact of our probe.
