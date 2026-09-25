#pragma once

// =============================================================================
//  control/allocator — geometric thrust allocation
//
//  Given a wrench the vehicle WANTS, solve for the thruster outputs that produce
//  it. This is the opposite direction to `mixer::mix()`, which applies a fixed
//  +-1 pattern, and the difference is not cosmetic:
//
//    * a +-1 pattern carries NO MOMENT ARMS, so its six "axes" are not
//      commensurate quantities. On the mongla-5 hull, a demand of 1.0 in the yaw
//      column produces 0.350 N.m per newton of thrust while a demand of 1.0 in
//      the sway column produces 2.000 N. Those differ by 5.71x, and every PID
//      gain tuned against them silently absorbed that factor. Here each axis is
//      normalised by its own measured authority (see `authority()`), so a demand
//      of 1.0 means "the most this hull can do on this axis" on every axis.
//
//    * a +-1 pattern cannot express an off-axis thruster. The mongla-5 axial unit
//      sits 8.06 mm off the Z centreline, so surge carries a real pitch moment;
//      under the old mixer that arrived as an unexplained pitch disturbance.
//
//  ⛔ WHAT THIS DELIBERATELY DOES NOT DO: tolerate a dead thruster. An earlier
//  version carried `rebuild(avail)` and an `avail` mask on `allocate()`. Both are
//  DELETED, because on this hull they cannot help and nothing could have called
//  them:
//    * the mongla-5 matrix is 5x5 and exactly determined, so removing a column
//      removes an AXIS -- there is no better allocation to re-solve for;
//    * nothing on the vehicle can detect a dead thruster yet. ESC presence is
//      measured on the Pico and never reaches the wire (our upstream PR F /
//      their #10), so the mask had no possible source;
//    * and the flight path only ever passed `nullptr`.
//  So it was a wide interface serving a capability that did not exist. When #10
//  lands and presence is readable, add it THEN, with the consumer in hand. The
//  geometric fact it was there to express -- that this hull has no redundancy --
//  is asserted directly against the matrix in the test, which is a stronger claim
//  than a function nobody called.
//
//  ⛔ UNACTUATED AXES ARE REFUSED, NOT ROUNDED TO ZERO. On mongla-5 every
//  thruster's line of action passes through the long axis, so no combination
//  produces roll. That is computed from the geometry, never declared. A roll
//  demand comes back in `refused` so a caller cannot believe the vehicle is
//  holding an attitude it physically cannot hold.
//
//  ⚠ THE UNITS ARE NOT NEWTONS AND THIS MODULE WILL NOT PRETEND OTHERWISE.
//  Output is a signed fraction of ONE thruster's full output, -1..1 -- the same
//  quantity `mixer::toDshot()` already consumes. Converting to force needs a
//  thrust coefficient that has been measured for this thruster, and there is no
//  such number in this repo. The wrench rows are therefore force-like and
//  moment-like in CONSISTENT units: exact in direction and in ratio, silent
//  about magnitude. That is enough to allocate correctly and not enough to
//  predict a trajectory, and the distinction matters.
//
//  ⚠ NO ARDUINO, NO PARAMS, NO GLOBALS IN THE MATH. This file compiles on a host
//  with nothing but <math.h>, which is what makes `test/test_allocator` possible.
//  A 500 Hz allocator that can only be checked by flying it is not checkable.
// =============================================================================

#include "control/frames.h"

namespace allocator {

// Wrench row order, fixed. Named for what they physically are so a diagnostic
// can say WHICH authority ran out instead of printing an index.
enum Axis : int {
    AX_SWAY = 0,   // Fx
    AX_SURGE,      // Fy
    AX_HEAVE,      // Fz
    AX_PITCH,      // Mx
    AX_ROLL,       // My
    AX_YAW,        // Mz
    AX_COUNT
};

extern const char* const AXIS_NAME[AX_COUNT];

// How a saturating demand is given up. Both are defensible and they handle
// differently, so it is a parameter rather than a decision baked into the code.
enum Priority : int {
    // Scale the WHOLE demand uniformly until it fits. Preserves the ratio between
    // every axis, which is what `mixer::mix()` does today per mechanical group, so
    // this is the default: adopting the allocator must not change how the vehicle
    // handles on the same stick input.
    PRIO_UNIFORM = 0,
    // Attitude (roll/pitch/yaw) is met in full, and translation gets the headroom
    // that is left. Correct when holding an attitude matters more than getting
    // there -- which is most of a mission, and all of a vision servo. The cost is
    // legible and severe: on a two-thruster pair a full yaw demand consumes the
    // pair, so sway goes to zero rather than to half. That honesty is the point.
    PRIO_ATTITUDE = 1,
};

struct Result {
    bool  ok;                  // false = begin() never succeeded; out[] is all zero
    float scale_attitude;      // 1.0 = attitude demand met in full
    float scale_translation;   // 1.0 = translation demand met in full
    int   limiting_thruster;   // index of the thruster that ran out first, or -1
    // Axes that carry a non-zero demand this frame but cannot be produced by the
    // geometry at all. Bit i = Axis i. ⛔ Never silently zero -- report.
    unsigned refused_mask;
};

// Build the frame's wrench matrix and its inverse. Call once at boot, and again
// whenever FRAME_CLASS changes. Returns false and leaves the allocator disabled
// if the geometry is unusable (rank-deficient in a way that leaves a requested
// axis doubly-determined, a non-unit axis, a frame with no thrusters).
//
// `cls` is a frames::Class. Class 0 (legacy) is NOT handled here -- it has no
// geometry -- and begin() returns false for it by design, so the caller keeps
// using mixer::mix().
bool begin(int cls);

const char* frameName();
int   thrusterCount();

// Which axes this geometry cannot produce, as a bitmask over Axis. Computed from
// the matrix, so it follows the table automatically.
unsigned unactuatedMask();

// The largest single-axis wrench this frame can produce with every |output| <= 1.
// This is the number that makes the six demand channels comparable: on mongla-5
// it is sway 2.000, surge 1.000, heave 2.000, pitch 0.519, yaw 0.350, roll 0. A
// hull whose axes differ by 5.7x is a hull whose per-axis gains must differ by
// 5.7x -- so this is reported on the wire at boot by mixer::selectFrame(), which
// is the only way an operator or the companion ever gets to see it.
float authority(int axis);

// Allocate. `demand[AX_COUNT]` is normalised, -1..1, where 1.0 means "all the
// authority this hull has on that axis". `out` receives thrusterCount() values in
// -1..1; entries past that are zeroed.
//
// Deterministic and allocation-free: one N x 6 multiply plus at most two linear
// passes over the thrusters. No matrix work happens here -- that is all in
// begin() -- so this is safe at 500 Hz.
Result allocate(const float demand[AX_COUNT], int priority,
                float out[ALLOC_MAX_THRUSTERS]);

// The wrench a set of outputs actually produces, in the same units `authority()`
// reports.
//
// ⚠ NO FLIGHT CONSUMER, ON PURPOSE, AND THAT IS NOT AN ORPHAN. This is the
// correctness oracle the native test allocates against: you cannot check an
// allocator without computing the wrench its outputs produce, and asserting
// against recorded outputs instead would only prove the code still does what it
// did. It is also the readback an anti-windup needs (their #20) -- when that
// lands it has a caller. Kept BECAUSE the test needs it, not in case something
// might.
void achieved(const float out[ALLOC_MAX_THRUSTERS], float wrench[AX_COUNT]);

// Column j of the wrench matrix — what thruster j produces at unit output.
// Exposed for MOTOR_DETECT, which needs the expected angular response and must
// no longer read it out of a hard-coded mixer column.
void thrusterWrench(int j, float wrench[AX_COUNT]);

}  // namespace allocator
