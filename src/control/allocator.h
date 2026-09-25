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
//    * a +-1 pattern cannot express a DEAD thruster. Here availability is an
//      input: `allocate()` masks the column on the flight loop, and `rebuild()`
//      re-solves the geometry without it OFF the flight loop. The two differ and
//      the difference is the honest part -- see `rebuild()`.
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

// Re-solve the allocation for a changed set of healthy thrusters.
//
// ⛔ NOT FROM THE FLIGHT LOOP. This inverts a matrix. Call it from the task that
// notices the health change, then let the next allocate() use the result.
//
// ⚠ AND ON A HULL WITH NO REDUNDANCY, RE-SOLVING MAY NOT HELP. mongla-5's five
// thrusters produce exactly five axes (determinant 0.1817, condition number
// 5.71), so losing one does not reduce the quality of the solution -- it removes
// an axis. rebuild() then returns false, because there is no allocation, and the
// previous one stays in force. A caller that treats false as "keep flying" is
// wrong; the correct response is to surface. Passing null restores full health.
bool rebuild(const bool* avail);

bool  ready();
int   frameClass();
const char* frameName();
int   thrusterCount();

// Which axes this geometry cannot produce, as a bitmask over Axis. Computed from
// the matrix, so it follows the table automatically.
unsigned unactuatedMask();

// The largest single-axis wrench this frame can produce with every |output| <= 1.
// This is the number that makes the six demand channels comparable, and it is
// worth reading on a bench: on mongla-5 it is sway 2.000, surge 1.000, heave
// 2.000, pitch 0.519, yaw 0.350, roll 0. A hull whose axes differ by 5.7x is a
// hull whose per-axis gains must differ by 5.7x, and now that is visible.
float authority(int axis);

// Allocate. `demand[AX_COUNT]` is normalised, -1..1, where 1.0 means "all the
// authority this hull has on that axis". `avail` may be null (all healthy), else
// avail[i] false removes thruster i. `out` receives thrusterCount() values in
// -1..1; entries past that are zeroed.
//
// Deterministic and allocation-free: one N x 6 multiply plus at most two linear
// passes over the thrusters. No matrix work happens here -- that is all in
// begin() -- so this is safe at 500 Hz.
Result allocate(const float demand[AX_COUNT], const bool* avail,
                int priority, float out[ALLOC_MAX_THRUSTERS]);

// The wrench a set of outputs actually produces, in the same units `authority()`
// reports. This is the readback the anti-windup needs and the wire does not
// carry: the difference between this and the demand is precisely the charge a
// PID integrator must NOT accumulate.
void achieved(const float out[ALLOC_MAX_THRUSTERS], float wrench[AX_COUNT]);

// Column j of the wrench matrix — what thruster j produces at unit output.
// Exposed for MOTOR_DETECT, which needs the expected angular response and must
// no longer read it out of a hard-coded mixer column.
void thrusterWrench(int j, float wrench[AX_COUNT]);

}  // namespace allocator
