#pragma once

// =============================================================================
//  control/frames — WHERE THE THRUSTERS ARE. Geometry, not algebra.
//
//  One entry per thruster: a position in the body frame (metres, from the frame
//  origin) and a unit axis along which POSITIVE output pushes the vehicle. That
//  is the whole vehicle model. `allocator` turns it into the 6xN wrench matrix
//  and its inverse, at boot, so a hull change is a table edit rather than a
//  hand-derived matrix.
//
//  WARNING: WHY THIS FILE IS SEPARATE FROM allocator.cpp. Data and algorithm have
//  different review rules. A number in here is checked against a CAD model or a
//  tape measure; a line in there is checked against linear algebra. Mixing them
//  is how `mixer.cpp` ended up with a hard-coded matrix that nobody could tell
//  was for a different vehicle -- the matrix WAS the geometry, so there was
//  nothing to compare it against.
//
//  Caution: BODY FRAME, and it is the CAD's, stated once here so nothing downstream
//  has to guess:
//        +X  starboard   (the lateral tunnels' bore)
//        +Y  forward      (the long, 702 mm axis; the axial unit lies on it)
//        +Z  up           (the vertical tunnels' bore)
//  and therefore, by r x axis:
//        Mx = PITCH      My = ROLL       Mz = YAW
//  This is a right-handed frame with +Z up, so a POSITIVE heave force ascends,
//  which is the same convention `depth_control` works in.
//
//  WARNING: WARNING: THE VERTICAL SIGN CONVENTION IS NOT THE LEGACY MIXER'S, AND THIS IS THE
//  ONE THING THAT CAN SINK THE VEHICLE ON THE FIRST DIVE WITH FRAME_CLASS != 0.
//
//  `mixer.cpp`'s literal matrix gives every vertical motor a throttle column of
//  -1, because on that vehicle A POSITIVE MOTOR COMMAND PUSHES THE HULL DOWN
//  (docs/THRUSTER_MAP.md). Two negatives then combine to make "positive throttle
//  ascends", which is the convention `depth_control` closes its loop in.
//
//  This table has no such inversion: +Z is up, positive output pushes ALONG the
//  axis, so a positive vertical output ASCENDS directly. One negation, not two.
//
//  The two paths therefore disagree about the polarity of the vertical pair, and
//  a depth loop wired the wrong way round is POSITIVE FEEDBACK: measured deeper
//  than target -> command descend -> deeper. It does not oscillate, it dives.
//
//  WARNING: BEFORE ANY ARMED DIVE ON FRAME_CLASS != 0: set MOT_n_DIRECTION for the
//  vertical pair so that a positive command ascends, and CONFIRM it with the
//  Motor Test tab and a hand on the hull -- not by reasoning about this comment.
//  `depth::preview()` exists precisely so the sign can be read off telemetry
//  while disarmed; use it. The reason this is a comment and not code is that only
//  the vehicle knows which way its props are on.
//
//  Caution: THE SIGNS OF THE AXES ARE A DECISION, NOT A MEASUREMENT. Which end of Y is
//  the bow, and whether the axial unit is a tractor or a pusher, are settled
//  against the vehicle -- in water, one thruster at a time -- not from a part
//  list. Everything a sign error does here is invisible on a bench and violent
//  in water. `MOT_n_DIRECTION` remains the per-thruster correction; this table
//  is what the vehicle IS, not how it happens to be wired.
// =============================================================================

#include <stddef.h>

namespace frames {

// The maximum any frame in this file uses. Sized to NUM_THRUSTERS so the
// allocator's fixed buffers match the rest of the firmware; a frame may use
// fewer, and `Frame::n` is the truth.
#ifndef ALLOC_MAX_THRUSTERS
#define ALLOC_MAX_THRUSTERS 8
#endif

struct Thruster {
    const char* name;
    float pos[3];    // m, body frame, from the frame origin
    float axis[3];   // unit vector; positive output pushes ALONG it
};

struct Frame {
    const char*      name;
    int              n;
    const Thruster*  t;
};

// ── FRAME 1: MONGLA-5 — the hull in fabrication ──────────────────────────────
//
// Four free-flooding ⌀84 mm tunnels on the centreline plus one axial unit in the
// nose. Five thrusters, and the geometry gives 5 of 6 DOF: ROLL IS UNACTUATED,
// because every thruster's line of action passes through the long axis. That is
// a property of the hull, so `allocator` COMPUTES it from this table rather than
// taking anyone's word for it.
//
// PROVENANCE. Measured 2026-09-25 off the Onshape GLB export of Part Studio 1,
// from each body's exact per-primitive accessor bounds (NOT a decimated render
// mesh -- an earlier pass used one and produced 346.6 / 520.4 mm for the two
// pair separations, which is wrong by 3.4 and 1.4 mm; those figures are
// retracted). Hull bounding box from the same read: 176.10 x 172.09 x 702.00 mm,
// fineness 3.99.
//
//   part            CAD station     what it is
//   Part 11 / 1     +-175.00 mm     the lateral tunnels   -> sway + yaw
//   Part 12 / 2     +-259.50 mm     the vertical tunnels  -> heave + pitch
//   A2212           +330.60 mm      the axial motor       -> surge
//   Part 3           +351.00 mm     its ⌀88 mm duct ring (exit side)
//
// Separations are therefore 350.00 mm and 519.00 mm EXACTLY, which is what the
// vehicle documentation has said all along.
//
// Caution: THE AXIAL UNIT IS 8.06 mm OFF THE Z CENTRELINE, and that is not noise -- it
// is a real pitch moment of 0.00806 N.m per newton of surge. The +-1 mixer could
// not express it at all, so surge used to arrive as an uncorrected pitch-up
// disturbance for the attitude loop (and then for the CoB auto-trim) to absorb.
// Here it is a column entry, and the vertical pair cancels it for free: 0.0156
// of one thruster each at full surge.
static const Thruster MONGLA5_T[5] = {
    // name        position (m)                 axis (unit)
    { "lat_aft",  { 0.0f, -0.17500f,  0.0f     }, { 1.0f, 0.0f, 0.0f } },
    { "lat_fwd",  { 0.0f,  0.17500f,  0.0f     }, { 1.0f, 0.0f, 0.0f } },
    { "vert_aft", { 0.0f, -0.25950f,  0.0f     }, { 0.0f, 0.0f, 1.0f } },
    { "vert_fwd", { 0.0f,  0.25950f,  0.0f     }, { 0.0f, 0.0f, 1.0f } },
    { "axial",    { 0.0f,  0.33060f,  0.00806f }, { 0.0f, 1.0f, 0.0f } },
};

// ── FRAME 2: VECTORED-8, as GEOMETRY ─────────────────────────────────────────
//
// The competition vehicle: four horizontal T200s at 45 degrees and four vertical.
// Caution: THIS IS NOT THE FRAME THE VEHICLE FLIES ON TODAY. Today's path is the
// literal +-1 matrix in `mixer.cpp`, preserved bit-identically as FRAME_CLASS 0,
// and it stays the default. This entry exists so the same vehicle can be flown
// through the geometric allocator and the two compared on a bench, because a
// +-1 matrix and a geometric one are NOT the same map: the +-1 form overstates
// surge and sway by 1/cos(45 deg) = 41 % and carries no moment arms at all.
//
// WARNING: THE ARM LENGTHS BELOW ARE PLACEHOLDERS AND MUST BE MEASURED BEFORE USE.
// 0.220 / 0.120 / 0.150 m is a plausible BlueROV2-Heavy-like layout, not this
// vehicle. `allocator::begin()` refuses this frame unless FRAME_ARMS_OK is set,
// precisely so a guess cannot fly by default. Replace them with tape-measure
// numbers and delete the guard.
static const Thruster VECTORED8_T[8] = {
    { "h_FR", {  0.120f,  0.220f, 0.0f }, {  0.7071f, -0.7071f, 0.0f } },
    { "h_FL", { -0.120f,  0.220f, 0.0f }, {  0.7071f,  0.7071f, 0.0f } },
    { "h_RR", {  0.120f, -0.220f, 0.0f }, { -0.7071f, -0.7071f, 0.0f } },
    { "h_RL", { -0.120f, -0.220f, 0.0f }, { -0.7071f,  0.7071f, 0.0f } },
    { "v_FR", {  0.150f,  0.150f, 0.0f }, {  0.0f, 0.0f, 1.0f } },
    { "v_FL", { -0.150f,  0.150f, 0.0f }, {  0.0f, 0.0f, 1.0f } },
    { "v_RR", {  0.150f, -0.150f, 0.0f }, {  0.0f, 0.0f, 1.0f } },
    { "v_RL", { -0.150f, -0.150f, 0.0f }, {  0.0f, 0.0f, 1.0f } },
};

// FRAME_CLASS parameter values. 0 is deliberately NOT in this table: it is the
// legacy literal matrix in `mixer.cpp` and must stay reachable without any of
// this code running, so that adopting the allocator can never be the reason a
// working vehicle stops working.
enum Class : int {
    CLASS_LEGACY_VECTORED8 = 0,   // mixer.cpp's +-1 matrix, unchanged
    CLASS_MONGLA5          = 1,
    CLASS_VECTORED8_GEOM   = 2,   // needs measured arms — see above
};

inline const Frame* get(int cls) {
    static const Frame f1 = { "mongla5",       5, MONGLA5_T   };
    static const Frame f2 = { "vectored8_geom", 8, VECTORED8_T };
    switch (cls) {
        case CLASS_MONGLA5:        return &f1;
        case CLASS_VECTORED8_GEOM: return &f2;
        default:                   return nullptr;   // 0 = legacy, handled elsewhere
    }
}

}  // namespace frames
