# PR G — the bottom camera is a velocity sensor now. The board has nowhere to put it.

**Repo:** `srot-control-board` — ESP32 only
**Severity:** medium — it is the missing input for anything that holds a
POSITION rather than an attitude
**Evidence:** zero hits for `OPTICAL_FLOW`, `ODOMETRY`, `VISION_SPEED` or any
velocity ingest across `src/` and `include/`; `mav_commands.cpp:747-782` is the
full set of messages you accept

---

## What changed on our side

We turned the downward camera into a bottom-track velocity sensor. Measured
against a tape, real slides, `h = 0.72 m`:

| axis | measured | truth | error |
|---|---|---|---|
| lateral | 30.13 cm | 30 | +0.13 |
| forward | 31.09 cm | 30 | +1.09 |
| back | 31.04 cm | 30 | +1.04 |

**Max error 1.09 cm on 30 cm — 3.6 %**, against Nortek's 0.5–1 % for a real DVL
and 0.89–1.88 % ATE for published monocular VO in turbid water. A synthetic
control through the same optics with exact ground truth reads 0.02 cm.

**Stated honestly, because it bounds what you should trust:**
- These are **in air**, on a bench. A flat port changes the effective focal
  length (we measure 63.8° air / 46.7° water), and the in-water figure is
  validated but not yet tape-checked in a pool.
- A hand slide's own precision is ~±1 cm, so **this is an upper bound on our
  error, not the sensor's error** — the operator sits inside the error bar.
- It degrades on low-texture floor, on haloing from vehicle lighting, and
  caustics can satisfy a brightness-constancy objective and yield a confident
  wrong velocity. Our estimator gates on residual, inlier count and rotation
  fraction and **refuses** rather than guessing.

## The ask

**Accept `OPTICAL_FLOW_RAD` (msgid 106).** We are not asking you to invent a
message — this one already carries exactly what we produce:

| field | what we put in it |
|---|---|
| `integrated_x` / `integrated_y` | flow over the interval, radians |
| `integrated_xgyro` / `ygyro` / `zgyro` | body rotation over **the same interval** |
| `integration_time_us` | the interval |
| `distance` | height above the floor, metres |
| `quality` | 0–255, and **`0` means no valid flow** |

Two properties worth keeping when you implement it, both from ArduPilot's own
EKF3 handling:

1. **The gyro must be accumulated over the SAME interval as the flow**, or
   de-rotation is wrong by `f·ω·Δt`. Our producer's signature already enforces
   this; please do not resample the gyro at ingest.
2. **Gate it.** ArduPilot gates flow on tilt, height validity, innovation and a
   rate limit. A flow reading taken while the hull is rotating hard is mostly
   ego-motion, and we measured de-rotation making it *worse* above ~1.1 rad/s.
   `quality = 0` is the absence signal — please treat it as "no data", never as
   "zero velocity". (Same rule as `BARO_HEALTH` and the all-zero ESC block.)

**If you would rather have body velocity directly, ask for `ODOMETRY`** and we
will send that instead. **Do not use `VISION_SPEED_ESTIMATE`** — it is not
supported in the ecosystem we would be matching, and we would rather not build
against a message that will not travel.

## Why it belongs on your side eventually, not ours

Today we close this loop on the Pi. That is fine for a transit and wrong for a
hold: a station-keep is a 500 Hz job and the companion cannot make that
guarantee — the same argument your own `VS_ARDUSUB.md` makes for keeping the
inner loop on the board.

With velocity ingested you get the input for position hold, and `MOVE_*`
primitives stop being purely timed. **We are not asking for a position
estimator** — horizontal position is unobservable without a velocity sensor,
which is precisely the gap this fills — and we are not asking for it this
round. The ask is the ingest and the gating.

## No rush

Per Fahim, batch this with the rest. Nothing on our side is blocked: the
producer is built and the host loop closes today.

---

## A reference handler, in your own style

**Deliberately NOT sent as a patch to your source.** Unlike the ESC-presence
one-liner, this needs a new state field *and* a consumer, and where it lands is
your architecture call — a handler that writes into something nothing reads
would be worse than no code. So: a sketch of exactly what we mean, shaped like
`onManualControl()`, whose "CLAMP FIRST" discipline is the right one here too.

```cpp
// mav_commands.cpp — beside case MAVLINK_MSG_ID_MANUAL_CONTROL
case MAVLINK_MSG_ID_OPTICAL_FLOW_RAD:
    onOpticalFlowRad(msg);
    break;

static void onOpticalFlowRad(const mavlink_message_t& msg) {
    mavlink_optical_flow_rad_t f;
    mavlink_msg_optical_flow_rad_decode(&msg, &f);

    // GATE FIRST, and treat every rejection as ABSENCE, never as zero velocity.
    // quality == 0 is the sender saying "no valid flow" -- it is NOT "not moving".
    // A still hull and a covered lens produce the same numbers otherwise, and we
    // have already been bitten by that shape (BARO_HEALTH, and the all-zero ESC
    // block that duburi_ws measured).
    if (f.quality == 0)                       return;   // sender says invalid
    if (f.integration_time_us == 0)           return;   // no interval, no rate
    if (f.distance <= 0.0f)                   return;   // no height => no scale

    const float dt = f.integration_time_us * 1e-6f;

    // De-rotation. The gyro fields MUST be the rotation accumulated over THIS
    // SAME interval -- do not resample your own gyro here, or the correction is
    // wrong by f*omega*dt. duburi_ws measured that error dominating above about
    // 1.1 rad/s, where de-rotating made the estimate WORSE than leaving it alone.
    const float fx = (f.integrated_x - f.integrated_xgyro) / dt;   // rad/s
    const float fy = (f.integrated_y - f.integrated_ygyro) / dt;

    // Reject rotation-dominated intervals rather than trusting a small
    // difference of two large numbers.
    if (fabsf(f.integrated_xgyro) > ROT_FRACTION_MAX * fabsf(f.integrated_x) &&
        fabsf(f.integrated_ygyro) > ROT_FRACTION_MAX * fabsf(f.integrated_y))
        return;

    // Body-frame velocity: angular rate x height. Sign/axis convention is YOURS
    // to fix -- we will match whatever you choose, but please state it in the
    // header, because a silent sign flip here steers the hull AWAY from target
    // and looks exactly like a tuning problem.
    const float vx = fx * f.distance;
    const float vy = fy * f.distance;

    StateLock lk(g_state.mtx_nav, pdMS_TO_TICKS(2));
    if (!lk.ok()) return;                     // never block the 500 Hz loop
    g_state.nav.vel_x    = vx;
    g_state.nav.vel_y    = vy;
    g_state.nav.vel_ms   = millis();          // STALENESS: a consumer must age this
    g_state.nav.vel_qual = f.quality;
}
```

**The one property we would ask you to keep whatever the shape:** a consumer
must be able to tell *stale* from *slow*. `vel_ms` is there so a hold that stops
receiving flow decays its authority instead of holding a remembered velocity —
the same reason your own `MANUAL_FRESH_MS` / `MANUAL_DECAY_MS` exist.
