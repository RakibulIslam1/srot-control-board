# PR A — `VISION_API.md` is unblocked: the FOV, the bearing model, and an id clash

**Repo:** `srot-control-board` · **Blocks:** the `LANDING_TARGET` ingest
**Status of the blocker you named:** *"Still blocked on us, not you: Camera
FOV."* (`JETSON_FEEDBACK.md`) — **cleared**, below.

---

## 1. The FOV, measured

`duburi_ws` @ `8049de9`. Blue Robotics low-light USB camera, the mission optic.

| | value |
|---|---|
| horizontal FOV, **in air** | **63.82°** ± 0.7 |
| horizontal FOV, **in water** (flat port) | **46.72°** ± 0.7 |
| intrinsics @ 1280×720 | `fx 1027.87  fy 1033.86  cx 617.32  cy 373.02` |
| method | OpenCV `calibrateCameraRO` (Strobl & Hirzinger), 25 views |
| validation | k-fold **held-out**, plus an independent tape-measure check |

Two notes on how it was arrived at, because both cost us time and either could
cost you some:

- **`calibrateCameraRO`, not `calibrateCamera`.** A printed target is not flat
  enough; OpenCV's own documentation says so and points at the object-releasing
  method. Our first four answers (835.7 / 969.9 / 1011.2 / 1027.9 px) each had a
  green-looking residual.
- **Training RMS cannot compare the two methods.** RO adds ~3N parameters, so it
  always wins on training error. It has to be scored held-out, against
  `newObjPoints` rather than the ideal grid.

The water figure is Snell through the flat port, `2·asin(sin(θ_air/2)/1.333)`,
not a second calibration. **Use the water number** — 27 % narrower is the
difference between a search pattern that sees the prop and one that does not.

## 2. Your linear bearing model is wrong at the centre of the frame

`VISION_API.md` gives `angle_x = ex · HFOV/2`. That assumes the optical axis
passes through the middle of the sensor. On this camera it does not:

```
cx = 617.32 on a 1280-wide frame  ->  22.68 px LEFT of centre

target at the exact frame centre:
    linear model   angle_x =  0.000 deg
    pinhole        angle_x = +1.264 deg
```

**1.264° of standing bias at the aim point**, which is exactly where a torpedo
shot is decided. Use the pinhole form, which needs no FOV at all:

```
angle_x = atan2(px - cx, fx)
angle_y = atan2(py - cy, fy)
```

We compute this companion-side and send radians, so **nothing changes on your
side** — this is a correction to the document, so that the next person deriving
a bearing from it does not reintroduce the bias.

## 3. ⚠ `31001` IS ALLOCATED TWICE, IN THIS REPO, BY US

This is the item to action first, because it is a wire constant and both
claimants are still unimplemented — so today both return `UNSUPPORTED` and the
clash is invisible.

| document | asks for | for |
|---|---|---|
| `VISION_API.md` §3.1 | `MAV_CMD_SROT_VISION` = **31001** | the vision primitive |
| `TASKS_FROM_DUBURI_WS.md` §7A | `MAV_CMD_SROT_PAYLOAD_FIRE` — *"please: 31001"* | payload role enforcement |

Both were written by us, months apart, and neither noticed the other. Whichever
lands first silently claims the id; the second would then either collide or be
quietly renumbered, and a renumbered command is precisely the class of change
the frozen-wire-constants invariant exists to prevent.

**Proposed allocation** — the space is empty, so this costs nothing now and is
very expensive later:

```
31000  SROT_MOVE          (shipped)
31001  SROT_PAYLOAD_FIRE  (TASKS §7A -- the safety interlock; give it the low id)
31002  SROT_VISION        (VISION_API §3.1)
31010..31014  USER_1..5   (shipped: stunt x3, pattern, autotune)
```

Payload takes 31001 because it is the one with a *safety* argument behind it —
`DO_SET_SERVO` on a role-1 channel currently moves the manipulator arm and
reports success — and because our vision path works today through
`MANUAL_CONTROL`, so `SROT_VISION` is an optimisation where payload-fire is an
interlock. We are happy either way; what matters is that it is decided in one
place before either is written.

## 4. Two fields the spec is missing, and the bug they cause

`LANDING_TARGET` has `x`, `y`, `z` spare and we currently send `0.0, 0.0, 0.0`.
Please define:

- **`x` = `coasted`** (0/1) — the target is a Kalman prediction of the locked
  track, not a fresh detection.
- **`y` = gap age in seconds** — how long since the last *real* detection.

Without them the board can only age a target from its own receipt time, so a
coasted target is **double-decayed**: once by our coast authority, once by your
staleness. Our ladder is `_freshness` 0.10 s < `coast_s` 0.40 < `lost_grace_s`
0.80 < declared-loss 1.00, so a target we still consider live at 0.6 s would be
long dead on your side — roughly 2× too fast, coast gone inside 0.4 s.

It costs nothing today (we already send the message; the board drops it) and
becomes a latent bug the moment staleness is implemented.

## 5. What is already built on our side

- `LANDING_TARGET` producer, **default off**, measured at ~1.9 kB/s at 25 Hz
- bearings from the calibration above, agreeing with an independent computation
  to **0.003°**
- the selection policy stays with us exactly as §2 proposes: class filter,
  control-side confidence floor, largest-area acquisition, continuity lock,
  Kalman coast, and payload gating on frame identity

We are not asking for `SROT_VISION` soon. The vision loop closes today through
`MANUAL_CONTROL` in `STABILIZE` and is measured end to end. What we are asking
for is **the id allocation in §3**, because that is cheap now and irreversible
later.

---

## 6. Answering your open questions (`VISION_API.md` §9)

You answered Q1 (subtype) and Q2 (gyro rate for yaw damping) in
`FIRMWARE_CHANGELOG_FOR_DUBURI.md` and we agree with both — Q2 especially, since
differentiating a 20 Hz bearing on our side would be strictly worse than a
500 Hz rate you already have. Three were left open. Our positions, so they stop
being open:

**Q3 — a persistent "vision hold" that outlives one command: yes, eventually,
but not first.** Every mission we run today is phrased as a bounded command with
a deadline, and that is deliberate: a bounded verb has an ACK, and the ACK is
what sequences the mission. A hold that outlives its command needs a second
mechanism to end it and a defined behaviour when the target is lost, which is
new failure surface for a capability nothing currently asks for. The one case
that genuinely wants it is a manipulator task, and we have no manipulator on the
vehicle. **Suggest deferring until there is one**; `p4` (hold seconds) covers
every task in the 2026 set.

**Q4 — range: agreed, do not use our monocular distance.** We can send it and we
do not trust it for control. If the board ever wants real range, a known-size
target plus angular size is the better construction, and we would rather send
you the angular size (which we measure directly and well) than a number derived
from a depth network we cannot validate underwater. **We will keep `distance`
unset rather than send something plausible-looking.** A field that is present
and wrong is worse than one that is absent, which is the same rule your
`DEPTH_ERR` suppression follows.

**Q5 — `target_num`: please treat it as a CLASS id, from a frozen table.** This
is the one we have a concrete need on. We currently send a hardcoded `0` from
the runtime while `tools/srot_uplink_check.py` sends `d.class_id` — our own two
senders disagree about what the field means, which is our bug and we are fixing
it. The fix needs the mapping to be a **frozen, append-only table** shared by
both repos, exactly like `SERVOn_FUNCTION` and `movement::Type`:

- **append-only**, because inserting a value silently renames every class after
  it — the same hazard your `SROT_SERVO_FUNC_*` comment warns about
- **0 reserved for "unspecified"**, so an unconfigured sender reads as
  unassigned rather than as whatever class happens to be first
- mirrored in `srot_protocol.py` on our side and in the spec on yours

We only ever track one target at a time and expect that to continue, so we are
**not** asking for multi-target. We would just like `target_num` to mean
something stable if you ever key behaviour on it (e.g. a different standoff for
a torpedo hole than for a gate).

If you would rather it stayed a pure instance counter, say so and we will send
`0` deliberately and document it as reserved — either is fine, but the two
senders on our side need one answer.
