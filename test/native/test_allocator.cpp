// =============================================================================
//  test/native/test_allocator — the allocator, checked on a host.
//
//  ⛔ WHY THIS EXISTS. A 500 Hz allocator whose only test is a water test is not
//  tested; it is trusted. `allocator.cpp` deliberately includes nothing but
//  <math.h> and <string.h> so it compiles here, and every number asserted below
//  was derived independently (exact rational arithmetic on the CAD geometry)
//  rather than recorded from a run of this code. A test that records what the
//  code does cannot tell you the code is wrong.
//
//      ./tools/run_native_tests.sh
//
//  Each check prints PASS/FAIL and the numbers, because a bare "OK" from a
//  numerical test is worth very little when the question is whether a moment arm
//  is 0.175 m or 0.1732 m.
// =============================================================================

#include "control/allocator.h"
#include <cstdio>
#include <cmath>
#include <cstring>

static int g_fail = 0;

static void check(bool cond, const char* what) {
    printf("  [%s] %s\n", cond ? "PASS" : "FAIL", what);
    if (!cond) g_fail++;
}

static void near(float got, float want, float tol, const char* what) {
    const bool ok = fabsf(got - want) <= tol;
    printf("  [%s] %-44s got %+10.6f  want %+10.6f\n",
           ok ? "PASS" : "FAIL", what, (double)got, (double)want);
    if (!ok) g_fail++;
}

using namespace allocator;

// ── the geometry, restated independently ─────────────────────────────────────
//
// Derived by hand from the CAD stations, NOT read from frames.h. If somebody
// edits the table, this disagrees and that is the whole point.
//   lateral pair  at y = +-0.1750, axis +X  -> Fx = 1,        Mz = -/+0.1750
//   vertical pair at y = +-0.2595, axis +Z  -> Fz = 1,        Mx = -/+0.2595
//   axial         at y = +0.3306, z = +0.00806, axis +Y -> Fy = 1, Mx = -0.00806
// reduced determinant = (1*-0.175 - 1*0.175) * 0.519 = -0.18165
static void test_geometry() {
    printf("\nFRAME 1 (mongla-5): geometry -> wrench matrix\n");
    check(begin(frames::CLASS_MONGLA5), "begin(CLASS_MONGLA5) succeeds");
    check(thrusterCount() == 5, "five thrusters");
    check(strcmp(frameName(), "mongla5") == 0, "frame name");

    float w[AX_COUNT];
    thrusterWrench(0, w);   // lat_aft at y = -0.1750
    near(w[AX_SWAY],  1.0f,      1e-6f, "lat_aft  Fx");
    near(w[AX_YAW],   0.1750f,   1e-6f, "lat_aft  Mz (yaw arm)");
    near(w[AX_SURGE], 0.0f,      1e-6f, "lat_aft  Fy");
    thrusterWrench(1, w);   // lat_fwd at y = +0.1750
    near(w[AX_YAW],  -0.1750f,   1e-6f, "lat_fwd  Mz (opposite sign)");
    thrusterWrench(2, w);   // vert_aft at y = -0.2595
    near(w[AX_HEAVE], 1.0f,      1e-6f, "vert_aft Fz");
    near(w[AX_PITCH],-0.2595f,   1e-6f, "vert_aft Mx (pitch arm)");
    thrusterWrench(4, w);   // axial at y = +0.3306, z = +0.00806
    near(w[AX_SURGE], 1.0f,      1e-6f, "axial    Fy");
    near(w[AX_PITCH],-0.00806f,  1e-7f, "axial    Mx  (the 8.06 mm offset IS a pitch moment)");
    near(w[AX_YAW],   0.0f,      1e-7f, "axial    Mz");
}

// ⛔ ROLL. Unactuated must be COMPUTED, and a roll demand must be REFUSED.
static void test_roll_is_refused() {
    printf("\nROLL is unactuated, and that is computed from the table\n");
    check(begin(frames::CLASS_MONGLA5), "begin");
    check((unactuatedMask() & (1u << AX_ROLL)) != 0, "roll flagged unactuated");
    check((unactuatedMask() & ~(1u << AX_ROLL)) == 0, "and nothing else is");
    near(authority(AX_ROLL), 0.0f, 1e-9f, "roll authority is exactly zero");

    float d[AX_COUNT] = {0}, out[ALLOC_MAX_THRUSTERS];
    d[AX_ROLL] = 0.8f;
    Result r = allocate(d, nullptr, PRIO_UNIFORM, out);
    check(r.ok, "allocate() still succeeds");
    check((r.refused_mask & (1u << AX_ROLL)) != 0, "roll demand REPORTED as refused");
    float sum = 0.0f;
    for (int j = 0; j < 5; ++j) sum += fabsf(out[j]);
    near(sum, 0.0f, 1e-9f, "...and produced no thruster output at all");
}

// The number that makes the six channels comparable. Exact values:
//   sway  1/0.5     = 2.000    heave 1/0.5     = 2.000
//   surge 1/1.0     = 1.000
//   pitch 1/(1/0.519) -> 0.519 yaw   1/(1/0.35) -> 0.350
static void test_authority() {
    printf("\nAUTHORITY per axis -- the 5.71x the +-1 mixer hid\n");
    check(begin(frames::CLASS_MONGLA5), "begin");
    near(authority(AX_SWAY),  2.0f,   1e-5f, "sway");
    near(authority(AX_SURGE), 1.0f,   1e-5f, "surge");
    near(authority(AX_HEAVE), 2.0f,   1e-5f, "heave");
    near(authority(AX_PITCH), 0.519f, 1e-5f, "pitch  (= the 519 mm separation)");
    near(authority(AX_YAW),   0.350f, 1e-5f, "yaw    (= the 350 mm separation)");
    const float ratio = authority(AX_SWAY) / authority(AX_YAW);
    near(ratio, 5.714286f, 1e-4f, "sway:yaw authority ratio");
    printf("        ^ a hull whose axes differ by %.2fx needs per-axis gains that differ by %.2fx\n",
           (double)ratio, (double)ratio);
}

// A unit demand on ONE axis must saturate exactly one thruster at exactly 1.0 --
// no headroom left unused, and nothing over the limit. This is the property the
// authority normalisation exists to create.
static void test_unit_demand_is_exactly_full() {
    printf("\nA demand of 1.0 on one axis uses exactly all of that axis\n");
    check(begin(frames::CLASS_MONGLA5), "begin");
    const int axes[5] = { AX_SWAY, AX_SURGE, AX_HEAVE, AX_PITCH, AX_YAW };
    for (int a = 0; a < 5; ++a) {
        float d[AX_COUNT] = {0}, out[ALLOC_MAX_THRUSTERS];
        d[axes[a]] = 1.0f;
        Result r = allocate(d, nullptr, PRIO_UNIFORM, out);
        float mx = 0.0f;
        for (int j = 0; j < 5; ++j) mx = fmaxf(mx, fabsf(out[j]));
        char buf[64];
        snprintf(buf, sizeof(buf), "%s: max|out| == 1", AXIS_NAME[axes[a]]);
        near(mx, 1.0f, 1e-5f, buf);
        // and it produced the wrench it was asked for, to within nothing
        float wr[AX_COUNT];
        achieved(out, wr);
        snprintf(buf, sizeof(buf), "%s: achieved == authority", AXIS_NAME[axes[a]]);
        near(wr[axes[a]], authority(axes[a]), 1e-4f, buf);
        // no scale-down was needed
        snprintf(buf, sizeof(buf), "%s: no saturation scaling", AXIS_NAME[axes[a]]);
        near(r.scale_attitude, 1.0f, 1e-5f, buf);
    }
}

// ⭐ THE COUPLING THE OLD MIXER COULD NOT EXPRESS. Pure surge on this hull needs
// the vertical pair to cancel the axial unit's pitch moment: 0.00806 / 0.519 =
// 0.01553 of one thruster each, in opposite directions. Under the +-1 mixer that
// moment was simply left on the vehicle.
static void test_surge_pitch_coupling() {
    printf("\nPure SURGE needs a pitch correction the +-1 mixer never made\n");
    check(begin(frames::CLASS_MONGLA5), "begin");
    float d[AX_COUNT] = {0}, out[ALLOC_MAX_THRUSTERS];
    d[AX_SURGE] = 1.0f;
    allocate(d, nullptr, PRIO_UNIFORM, out);
    near(out[4], 1.0f,      1e-5f, "axial at full");
    near(out[2], -0.015530f, 1e-5f, "vert_aft trims");
    near(out[3], +0.015530f, 1e-5f, "vert_fwd trims the other way");
    float wr[AX_COUNT];
    achieved(out, wr);
    near(wr[AX_PITCH], 0.0f, 1e-6f, "net pitch from a pure surge command: ZERO");
    printf("        ^ without this, full surge leaves %.5f N.m/N of uncorrected pitch\n", 0.00806);
}

// Priority is a parameter because the two behaviours are both defensible and they
// differ sharply. UNIFORM must reproduce the old per-group feel; ATTITUDE must
// give the attitude loop the whole pair when it asks for it.
static void test_priority_modes() {
    printf("\nSATURATION: uniform vs attitude-first\n");
    check(begin(frames::CLASS_MONGLA5), "begin");
    float d[AX_COUNT] = {0}, out[ALLOC_MAX_THRUSTERS];
    d[AX_SWAY] = 1.0f; d[AX_YAW] = 1.0f;   // the lateral pair, asked for twice over

    Result u = allocate(d, nullptr, PRIO_UNIFORM, out);
    near(u.scale_attitude, 0.5f, 1e-5f, "uniform: everything halved");
    near(out[0], 1.0f,  1e-5f, "uniform: lat_aft at the rail");
    near(out[1], 0.0f,  1e-5f, "uniform: lat_fwd stopped");
    float wr[AX_COUNT]; achieved(out, wr);
    near(wr[AX_YAW] / authority(AX_YAW), 0.5f, 1e-4f, "uniform: HALF the yaw it asked for");

    Result a = allocate(d, nullptr, PRIO_ATTITUDE, out);
    near(a.scale_attitude,    1.0f, 1e-5f, "attitude-first: yaw met in full");
    near(a.scale_translation, 0.0f, 1e-5f, "attitude-first: sway yields entirely");
    achieved(out, wr);
    near(wr[AX_YAW] / authority(AX_YAW), 1.0f, 1e-4f, "attitude-first: ALL the yaw");
    near(wr[AX_SWAY], 0.0f, 1e-5f, "attitude-first: no sway");
    printf("        ^ both are correct. The pair cannot do both, and neither mode pretends it can.\n");
}

// ⛔ MECHANICAL INDEPENDENCE. PR #25 asked for the per-group behaviour to survive
// whatever replaced the matrix. It does, and for a better reason than before: the
// solution operator is block-diagonal because the GEOMETRY is, so a saturating
// surge cannot reach the lateral pair. Checked by driving surge to the rail and
// asserting the lateral outputs are bit-identical.
static void test_groups_stay_independent() {
    printf("\nA saturating SURGE must not cost any YAW authority\n");
    check(begin(frames::CLASS_MONGLA5), "begin");
    float d1[AX_COUNT] = {0}, o1[ALLOC_MAX_THRUSTERS];
    d1[AX_YAW] = 0.6f;
    allocate(d1, nullptr, PRIO_UNIFORM, o1);
    float d2[AX_COUNT] = {0}, o2[ALLOC_MAX_THRUSTERS];
    d2[AX_YAW] = 0.6f; d2[AX_SURGE] = 1.0f;
    allocate(d2, nullptr, PRIO_UNIFORM, o2);
    check(o1[0] == o2[0] && o1[1] == o2[1], "lateral pair BIT-IDENTICAL with surge added");
    printf("        (%.9f, %.9f) vs (%.9f, %.9f)\n",
           (double)o1[0], (double)o1[1], (double)o2[0], (double)o2[1]);
}

// A dead thruster is a limit, not a new matrix. With one lateral tunnel gone the
// hull loses a DOF outright -- there is no redundancy to fall back on -- and the
// allocator must say so rather than produce a plausible wrong answer.
static void test_dead_thruster() {
    printf("\nA DEAD THRUSTER: no redundancy exists, so say so\n");
    check(begin(frames::CLASS_MONGLA5), "begin");
    float d[AX_COUNT] = {0}, out[ALLOC_MAX_THRUSTERS];
    bool avail[ALLOC_MAX_THRUSTERS] = { false, true, true, true, true };  // lat_aft dead
    d[AX_YAW] = 1.0f;
    Result r = allocate(d, avail, PRIO_UNIFORM, out);
    check(r.ok, "allocate() succeeds");
    near(out[0], 0.0f, 1e-9f, "the dead thruster is commanded nothing");
    float wr[AX_COUNT];
    achieved(out, wr);
    printf("        yaw asked %.3f, got %.3f; sway side-effect %.4f\n",
           (double)authority(AX_YAW), (double)wr[AX_YAW], (double)wr[AX_SWAY]);
    check(fabsf(wr[AX_SWAY]) > 1e-3f,
          "⛔ and yaw now comes with unavoidable SWAY -- one pair, one thruster");
    printf("        ^ the determinant is 0.1817 with a condition number of 5.71: exactly\n"
           "          determined, zero redundancy. Any single loss costs a whole DOF.\n");
}

// rebuild() is the off-loop re-solve, and on this hull it must REFUSE: five
// thrusters produce exactly five axes, so removing one removes an axis and there
// is no allocation to fall back to. Refusing is the answer; pretending is not.
static void test_rebuild_refuses_when_a_dof_is_lost() {
    printf("\nrebuild() after a thruster loss: REFUSE, do not improvise\n");
    check(begin(frames::CLASS_MONGLA5), "begin");
    bool dead_lat[ALLOC_MAX_THRUSTERS] = { false, true, true, true, true };
    check(!rebuild(dead_lat), "one lateral tunnel lost -> rebuild() returns false");
    near(authority(AX_YAW), 0.350f, 1e-5f, "previous solution retained, not corrupted");
    check(rebuild(nullptr), "restoring full health rebuilds cleanly");
    near(authority(AX_YAW), 0.350f, 1e-5f, "and authority is unchanged");
    // The redundant frame is the opposite case: it CAN re-solve around a loss.
    check(begin(frames::CLASS_VECTORED8_GEOM), "begin(vectored8_geom)");
    bool dead_h[ALLOC_MAX_THRUSTERS] = { false, true, true, true, true, true, true, true };
    check(rebuild(dead_h), "⭐ eight thrusters CAN re-solve around one loss");
    printf("        yaw authority after the loss: %.4f (was 0.9625)\n", (double)authority(AX_YAW));
    printf("        ^ THIS is what redundancy buys, and mongla-5 has none of it.\n");
}

// Whatever the demand, no output may leave [-1, 1]. Swept rather than argued.
static void test_bounds_sweep() {
    printf("\nBOUNDS: no demand, however hostile, can command past the rail\n");
    check(begin(frames::CLASS_MONGLA5), "begin");
    float worst = 0.0f;
    long n = 0;
    const float V[5] = { -1.0f, -0.37f, 0.0f, 0.61f, 1.0f };
    for (int a = 0; a < 5; ++a)
    for (int b = 0; b < 5; ++b)
    for (int c = 0; c < 5; ++c)
    for (int e = 0; e < 5; ++e)
    for (int f = 0; f < 5; ++f)
    for (int p = 0; p < 2; ++p) {
        float d[AX_COUNT] = { V[a], V[b], V[c], V[e], 0.0f, V[f] };
        float out[ALLOC_MAX_THRUSTERS];
        allocate(d, nullptr, p, out);
        for (int j = 0; j < 5; ++j) worst = fmaxf(worst, fabsf(out[j]));
        ++n;
    }
    printf("        %ld demands swept, worst |output| = %.9f\n", n, (double)worst);
    check(worst <= 1.0f + 1e-6f, "every output inside [-1, 1]");
    // NaN must not propagate to a single thruster.
    float d[AX_COUNT] = { 0.5f, NAN, 0.2f, 0.1f, 0.0f, -0.3f };
    float out[ALLOC_MAX_THRUSTERS];
    Result r = allocate(d, nullptr, PRIO_UNIFORM, out);
    bool clean = true;
    for (int j = 0; j < 5; ++j) if (!std::isfinite(out[j])) clean = false;
    check(clean, "a NaN demand reaches NO thruster");
    check((r.refused_mask & (1u << AX_SURGE)) != 0, "...and is reported refused");
}

// The placeholder-arm frame must refuse to be trusted silently. It is allowed to
// BUILD (the algebra is fine) -- the point is that its numbers are not measured,
// which is a fact about the table and is recorded there, not here. What this
// checks is that it is at least self-consistent and full rank, so a measured
// table dropped in its place will work.
static void test_vectored8_geom_builds() {
    printf("\nFRAME 2 (vectored-8 as geometry): builds, full 6-DOF rank\n");
    check(begin(frames::CLASS_VECTORED8_GEOM), "begin(CLASS_VECTORED8_GEOM)");
    check(thrusterCount() == 8, "eight thrusters");
    check(unactuatedMask() == 0, "all six axes actuated (roll IS available here)");
    printf("        authority: sway %.3f surge %.3f heave %.3f pitch %.3f roll %.3f yaw %.3f\n",
           (double)authority(AX_SWAY), (double)authority(AX_SURGE), (double)authority(AX_HEAVE),
           (double)authority(AX_PITCH), (double)authority(AX_ROLL), (double)authority(AX_YAW));
    printf("        ⛔ PLACEHOLDER ARMS. Measure them before flying this frame.\n");
    // Minimum-norm property: the redundant frame must produce the wrench EXACTLY.
    float d[AX_COUNT] = {0}, out[ALLOC_MAX_THRUSTERS], wr[AX_COUNT];
    d[AX_YAW] = 0.5f; d[AX_SURGE] = 0.5f;
    allocate(d, nullptr, PRIO_UNIFORM, out);
    achieved(out, wr);
    near(wr[AX_YAW],   0.5f * authority(AX_YAW),   1e-4f, "redundant solve is EXACT in yaw");
    near(wr[AX_SURGE], 0.5f * authority(AX_SURGE), 1e-4f, "...and in surge");
    near(wr[AX_ROLL],  0.0f, 1e-5f, "...with no roll leaked");
}

// Class 0 is the legacy literal matrix and has no geometry. begin() must refuse
// it, so that adopting this module cannot silently take over the vehicle that is
// flying today.
static void test_legacy_class_is_refused() {
    printf("\nFRAME_CLASS 0 stays with mixer.cpp\n");
    check(!begin(frames::CLASS_LEGACY_VECTORED8), "begin(0) returns false");
    check(!ready(), "allocator stays disabled");
    float d[AX_COUNT] = {0}, out[ALLOC_MAX_THRUSTERS];
    d[AX_YAW] = 1.0f;
    Result r = allocate(d, nullptr, PRIO_UNIFORM, out);
    check(!r.ok, "allocate() refuses");
    near(out[0], 0.0f, 1e-9f, "and commands nothing");
    check(!begin(99), "an unknown class is refused too");
}

int main() {
    printf("=== allocator: native tests ===\n");
    test_geometry();
    test_roll_is_refused();
    test_authority();
    test_unit_demand_is_exactly_full();
    test_surge_pitch_coupling();
    test_priority_modes();
    test_groups_stay_independent();
    test_dead_thruster();
    test_rebuild_refuses_when_a_dof_is_lost();
    test_bounds_sweep();
    test_vectored8_geom_builds();
    test_legacy_class_is_refused();
    printf("\n=== %s ===\n", g_fail == 0 ? "ALL PASS" : "FAILURES PRESENT");
    return g_fail == 0 ? 0 : 1;
}
