# PR K — the mixer allocates for a hull that is not the one being built

**Target:** `srot-control-board` · **Rank: this outranks everything else we have filed.**
Not because it is urgent this week, but because every other control ask assumes the mixer
describes the vehicle, and on the hull now in fabrication it does not.

This is a measurement, not an opinion, and it is the first time the discrepancy has been
quantified rather than noted.

---

## 1. What the mixer says

`src/control/mixer.cpp:26` — `M[NUM_THRUSTERS][6]`, `SUB_FRAME_VECTORED_6DOF`:

```
//  roll  pitch  yaw   thr   fwd   lat
{  0,    0,     1,    0,   -1,    1 },  // 1 FR horiz      four horizontal
{  0,    0,    -1,    0,   -1,   -1 },  // 2 FL horiz      thrusters at 45 deg,
{  0,    0,    -1,    0,    1,    1 },  // 3 RR horiz      each carrying yaw,
{  0,    0,     1,    0,    1,   -1 },  // 4 RL horiz      surge AND sway
{  1,   -1,     0,   -1,    0,    0 },  // 5 FR vert
{ -1,   -1,     0,   -1,    0,    0 },  // 6 FL vert
{  1,    1,     0,   -1,    0,    0 },  // 7 RR vert
{ -1,    1,     0,   -1,    0,    0 },  // 8 RL vert
```

Eight thrusters. The four horizontals sit at 45°, which is why each one appears in the
`yaw`, `fwd` **and** `lat` columns at once. That is a correct vectored-6DOF allocation —
for a vectored-6DOF vehicle.

## 2. What the hull is

Measured 2026-09-23 from the Onshape GLB export (`docs/assets/cad/hull.mgla`), by finding
the bodies carrying the documented 84 mm bore and taking each one's longest extent as its
tunnel axis:

| tunnel | bore axis | station z | pair separation |
|---|---|---|---|
| lateral aft / fwd | **X** | −0.1732 / +0.1734 m | 346.6 mm |
| vertical aft / fwd | **Y** | −0.2603 / +0.2601 m | 520.4 mm |

Four **orthogonal** tunnels, all within 6 mm of the centreline, distributed along Z only —
plus one axial unit in the nose. Five thrusters, not eight. No 45°: every tunnel points
down exactly one body axis.

The hull bounding box from the same mesh is **176.1 × 172.1 × 702.0 mm**, which reproduces
the figure in our own documentation exactly. That agreement is why we trust the rest.

## 3. The consequence, as a rank

Building the geometric allocation matrix from those four tunnels — `B[0:3] = axis`,
`B[3:6] = r × axis`:

```
              lateral_aft  lateral_fwd  vertical_aft  vertical_fwd
    Fx sway        1.0000       1.0000        0.0000        0.0000
    Fy heave       0.0000       0.0000        1.0000        1.0000
    Fz surge       0.0000       0.0000        0.0000        0.0000
    Mx pitch       0.0000       0.0000        0.2603       -0.2601
    My yaw        -0.1732       0.1734       -0.0000        0.0000
    Mz roll       -0.0003      -0.0003        0.0005        0.0002
```

**Rank 4 of 6.** The tunnels give sway, heave, pitch and yaw. They give **no surge at
all** — the axial nose unit is the vehicle's only forward thrust, and roll is unactuated
by design.

⛔ **Run the firmware's `M` against this hull and the columns do not describe it.** The
mixer's `fwd` column asks all four horizontals for surge; on this hull the horizontals are
the *lateral* pair and produce none. The mixer's `lat` column asks the same four for sway
with a 45° component; here sway comes from two tunnels at unity. Roll is a real column in
`M` (motors 5-8 at ±1) and is unactuated on this hull.

## 4. What we are asking for

We are **not** asking you to guess our geometry. We are asking for the mixer to stop being
a compiled-in constant for one frame:

1. **A frame selector**, so `vectored_6dof` is one option rather than the only one — and a
   second entry for a straight orthogonal frame (tunnels on body axes, one axial).
2. ⭐ **Better, if you are willing: make `M` a function of GEOMETRY rather than a literal.**
   Per thruster, a position and an axis, with `B[0:3] = axis` and `B[3:6] = r × axis`
   computed once at boot. That is ~25 multiply-accumulates for eight thrusters, done once,
   and it makes a hull change a parameter change instead of a firmware change. It also
   expresses a dead thruster as a limit rather than requiring a new matrix — which is what
   every constrained-allocation method in the literature needs.
3. **Please keep the per-group saturation scaling exactly as it is.** We measured it on the
   bench and it works: adding heave to a saturating horizontal command left the horizontal
   outputs bit-identical. Your comment records why the global `maxabs` was wrong, and the
   fix holds. Whatever replaces `M`, that behaviour should survive.

## 5. What we will bring

- The measured geometry, in the frame above, as a file
  (`src/mongla_control/config/hull_geometry.yaml`) rather than prose.
- ⚠ The axial unit is currently marked **UNKNOWN** in it, because it is absent from the GLB
  export. Its position, axis and count are being read off the CAD directly. We are not
  sending you a matrix with a guessed column in it.
- A host-side bench that runs **your** `mixer.cpp` compiled natively against captured board
  data, so a proposed matrix can be checked before it is flashed. It currently reproduces
  the board exactly on the one capture where the attitude loop was quiet, and to within 6
  counts of 999 elsewhere.

## 6. What this does NOT say

It does not say the current mixer is wrong. It is right for the **8-thruster vectored
competition vehicle**, and that vehicle exists. It says the two vehicles are different, and
that the firmware currently has no way to express the second one.

Our own documentation has carried the warning — *"the hull in CAD is not the hull the mixer
was written for"* — for some time. This is the first time it has been measured, and the
measurement is what makes it actionable.

---

*Evidence: `docs/assets/cad/hull.mgla`, packed from the Onshape export by
`tools/pack_hull.py`; body centroids measured 2026-09-23; hull bounding box cross-checked
against `CLAUDE.md`. Firmware read at `src/control/mixer.cpp:26-42` and `include/config.h:117`.*
