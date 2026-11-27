# Vision uplink, stage 1 — accept `LANDING_TARGET`, echo it, actuate nothing

From the `duburi_ws` (Mongla) companion side, 2026-09-10.
Re-derived against **Hengla v0.2.0, behaviour rev 14** (`config.h:805`).
Where this disagrees with `VISION_API.md`, the source won — see §5.

---

## 1. What we are asking for

One `case` in the message switch, six fields of state, and four telemetry
names. **No control, no new command id, no mode change, no arming path.**

```cpp
// src/comms/mav_commands.cpp — beside case MAVLINK_MSG_ID_MANUAL_CONTROL (:782)
case MAVLINK_MSG_ID_LANDING_TARGET:
    onLandingTarget(msg);
    break;
```

```cpp
static void onLandingTarget(const mavlink_message_t& msg) {
    mavlink_landing_target_t lt;
    mavlink_msg_landing_target_decode(&msg, &lt);

    // ⛔ THIS PATH DOES NOT INHERIT YOUR NaN GUARD. The blanket isfinite()
    // sweep is in the COMMAND path (mav_commands.cpp:290-295, before
    // `switch (command)`), so it covers COMMAND_LONG/INT and nothing else.
    // angle_x/angle_y/size_x/size_y are float on the wire and a companion bug
    // can legally send NaN. A NaN bearing stored now is a NaN error term the
    // day stage 2 closes the loop -- and NaN fails every comparison silently,
    // so a staleness check would read "fresh" and a limit check would read
    // "in range".
    if (!isfinite(lt.angle_x) || !isfinite(lt.angle_y) ||
        !isfinite(lt.size_x)  || !isfinite(lt.size_y)) {
        return;                       // drop, do not store, do not count
    }

    StateLock lk(g_state.mtx_control);
    if (!lk.ok()) return;             // skip a cycle, as everywhere else
    g_state.control.vis_bearing  = lt.angle_x;
    g_state.control.vis_elev     = lt.angle_y;
    g_state.control.vis_size     = sqrtf(fabsf(lt.size_x * lt.size_y));
    g_state.control.vis_range    = (lt.distance > 0.0f) ? lt.distance : 0.0f;
    g_state.control.vis_stamp_ms = millis();   // AT RECEIPT, on Core 0
    g_state.control.vis_seq++;
    g_state.control.vis_valid    = true;
}
```

Telemetry, through the `sendNamed()` you already have (`mav_stream.cpp:335`):

```
VIS_BRG   rad, + = right          VIS_ELV   rad, + = below
VIS_SIZ   rad, angular size       VIS_AGE   s, (millis() - vis_stamp_ms)/1000
```

`VIS_AGE` is the one that matters most — see §3.

## 2. Why this is worth doing now rather than with stage 2

**Because it is the only way either side finds out it is wrong before a
thruster turns.** We have a producer and you have no consumer; the moment those
meet under control, a disagreement about sign, units or staleness becomes hull
motion. With the echo we can put your `VIS_BRG` beside our `angle_x` on one
plot and settle every convention **at zero risk**, disarmed, on a bench.

That is the same staging that worked for `SROT_MOVE`: `MAV_CMD_SROT_VISION`
(31001) returns `UNSUPPORTED` today, correctly, and that honest refusal is what
let us build the host side against a consumer that does not exist without ever
believing it worked.

**Two things changed this week that make it timely.**

**(a) Our bearing was wrong, and we fixed it.** A flat port refracts, so the
pinhole model recovers the ray angle *inside* the housing while the vehicle
needs the direction in the water: `sin(air) = n·sin(water)`. Measured on our
forward calibration (fx 851.2, cx 675.4):

```
px from cx   air (what we sent)   true water bearing      error
     100       6.70 deg             5.02 deg            +1.68 deg  (+33.4 %)
     300      19.41                14.44                +4.97      (+34.5 %)
     640      36.94                26.80               +10.14      (+37.8 %)
```

Near the axis `sin x ≈ x`, so it is not an edge effect — it was the refractive
index applied to the whole angular scale. **Had you implemented ingest before
we found this, your gains would have been tuned against a bearing a third too
large.** Fixed companion-side; nothing changes on your wire.

**(b) The link has no room today, and PR #17 is the fix.** Measured
read-only with the hull idle and nothing commanded: **5969 B/s of 11520 —
51.8 % of capacity**, leaving ~93 `LANDING_TARGET`/s before the outbound half.
Stage 1 at 10–25 Hz fits; a 500 Hz loop does not. We would merge #17 first.

## 3. `VIS_AGE`, and the one field we would like defined

Your staleness timer is the whole safety mechanism on this path, and **sending
nothing is our loss signal** — a detector that sees nothing stops sending, and
you age the last bearing out. We never re-send a stale sample to "hold" a
target: on the wire that is indistinguishable from a live one and it defeats
exactly the mechanism that protects the hull.

⚠ But our sample can already be *coasted* — a tracker-predicted box bridging a
detector gap. Its age from your side is the time since we sent it; its true age
is the time since a real detection. Without a `coasted` flag and a true gap age
those two get multiplied together and the target is **double-decayed, roughly
2× too fast** — coast dead inside 0.4 s. We raised this in PR #2; it costs
nothing today and becomes a latent bug the moment staleness is implemented. We
currently carry both in `LANDING_TARGET.position_valid` / `.q[0]`, and are
happy to move them wherever you prefer.

## 4. What this does NOT ask for

- **No `SROT_VISION`.** Stage 1 needs no new command id, so it is **not blocked
  on the 31001 double-allocation** we raised in PR #2. That decision can take
  its time.
- **No `movement::Type::VISION`.** For the record when it does come: the enum
  ends `STYLE, ARC` (`movement.h:26`) and the dispatch documents
  `wire type + 1 = movement::Type` (`mav_commands.cpp:327`), so VISION would be
  enum **11**, wire **10**, and the bound `wire > 9` becomes `wire > 10` **in
  the same commit** or the verb is dead on arrival.
- **No control, no mode entry, no arming interaction.** `vis_valid` is written
  and read by nothing.

## 5. Corrections to `VISION_API.md` (our document, our errors)

Re-derived against rev 14 — these are stale:

| `VISION_API.md` says | actually |
|---|---|
| `:199` → the wire bound at `mav_commands.cpp:285` | **`:327`** |
| `:206` → mutex-miss at `mav_commands.cpp:287` | **`:328-334`** |
| `angle_x = ex · HFOV/2` | wrong at frame centre by **1.264 deg** (our `cx` is 22.68 px off-axis). We send `atan2` + refraction; document only |

We will send a documentation PR correcting these separately so this one stays
reviewable.

## 6. Honesty about what we did and did not verify

**We did not compile this.** There is no PlatformIO toolchain on the companion
or the dev box, so the code above is written against your patterns
(`onManualControl` for ingest, `sendNamed` for telemetry, `StateLock` for the
write) and **has not been built**. Treat it as a precise specification rather
than a patch — we would rather hand you something honest than something that
looks tested.

What we *have* verified, live on the board 2026-09-10: 19 message ids are
handled and `LANDING_TARGET` is not among them, so what we send today is parsed
and dropped; the link utilisation above; and our own bearing maths against the
calibration.

We will run the echo comparison and post the plot the day this lands.
