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
    Result r = allocate(d, PRIO_UNIFORM, out);
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
        Result r = allocate(d, PRIO_UNIFORM, out);
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
    allocate(d, PRIO_UNIFORM, out);
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

    Result u = allocate(d, PRIO_UNIFORM, out);
    near(u.scale_attitude, 0.5f, 1e-5f, "uniform: everything halved");
    near(out[0], 1.0f,  1e-5f, "uniform: lat_aft at the rail");
    near(out[1], 0.0f,  1e-5f, "uniform: lat_fwd stopped");
    float wr[AX_COUNT]; achieved(out, wr);
    near(wr[AX_YAW] / authority(AX_YAW), 0.5f, 1e-4f, "uniform: HALF the yaw it asked for");

    Result a = allocate(d, PRIO_ATTITUDE, out);
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
    allocate(d1, PRIO_UNIFORM, o1);
    float d2[AX_COUNT] = {0}, o2[ALLOC_MAX_THRUSTERS];
    d2[AX_YAW] = 0.6f; d2[AX_SURGE] = 1.0f;
    allocate(d2, PRIO_UNIFORM, o2);
    check(o1[0] == o2[0] && o1[1] == o2[1], "lateral pair BIT-IDENTICAL with surge added");
    printf("        (%.9f, %.9f) vs (%.9f, %.9f)\n",
           (double)o1[0], (double)o1[1], (double)o2[0], (double)o2[1]);
}

// ⛔ THIS HULL HAS NO REDUNDANCY, asserted against the GEOMETRY rather than
// against a fault-tolerance API.
//
// An earlier version proved this through `rebuild(avail)` and an `avail` mask on
// `allocate()`. Both are gone: nothing on the vehicle can detect a dead thruster
// (ESC presence never reaches the wire -- their #10), the flight path only ever
// passed nullptr, and on a 5x5 exactly-determined matrix re-solving cannot help
// anyway. Testing a function nobody calls is testing dead code.
//
// The claim that actually matters survives, and it is stronger stated this way:
// remove ANY single column from B and the rank drops, i.e. an axis is LOST. On
// the 8-thruster frame it does not. That difference is the whole argument for
// redundancy, and it is a property of the geometry, not of any code path.
static int rank_of(const float* rows, int nr, int nc) {
    // Gaussian elimination with partial pivoting; counts independent rows.
    float m[AX_COUNT * ALLOC_MAX_THRUSTERS];
    for (int i = 0; i < nr * nc; ++i) m[i] = rows[i];
    int rank = 0;
    for (int c = 0; c < nc && rank < nr; ++c) {
        int piv = -1; float best = 1e-6f;
        for (int r = rank; r < nr; ++r) {
            const float v = fabsf(m[r * nc + c]);
            if (v > best) { best = v; piv = r; }
        }
        if (piv < 0) continue;
        for (int k = 0; k < nc; ++k) { float t = m[rank*nc+k]; m[rank*nc+k] = m[piv*nc+k]; m[piv*nc+k] = t; }
        for (int r = 0; r < nr; ++r) {
            if (r == rank) continue;
            const float f = m[r * nc + c] / m[rank * nc + c];
            if (f == 0.0f) continue;
            for (int k = 0; k < nc; ++k) m[r * nc + k] -= f * m[rank * nc + k];
        }
        ++rank;
    }
    return rank;
}

// Build the 6 x N matrix from the public per-thruster wrench, optionally dropping
// column `skip`. Uses only what a caller can see, so it cannot accidentally test
// allocator internals.
static int rank_without(int skip, int n) {
    float rows[AX_COUNT * ALLOC_MAX_THRUSTERS] = {0};
    int nc = 0;
    for (int j = 0; j < n; ++j) {
        if (j == skip) continue;
        float w[AX_COUNT];
        thrusterWrench(j, w);
        for (int i = 0; i < AX_COUNT; ++i) rows[i * (n - (skip >= 0 ? 1 : 0)) + nc] = w[i];
        ++nc;
    }
    return rank_of(rows, AX_COUNT, nc);
}

static void test_no_redundancy_is_a_property_of_the_geometry() {
    printf("\nREDUNDANCY: mongla-5 has none, and the 8-thruster frame does\n");
    check(begin(frames::CLASS_MONGLA5), "begin(mongla5)");
    const int full5 = rank_without(-1, 5);
    printf("        full rank = %d of 6 (roll unactuated)\n", full5);
    check(full5 == 5, "all five thrusters -> rank 5");
    bool every_loss_costs_an_axis = true;
    for (int j = 0; j < 5; ++j) {
        const int r = rank_without(j, 5);
        printf("        drop thruster %d -> rank %d %s\n", j, r,
               r < full5 ? "  <- an AXIS is lost" : "");
        if (r >= full5) every_loss_costs_an_axis = false;
    }
    check(every_loss_costs_an_axis,
          "⛔ EVERY single-thruster loss costs a whole DOF -- zero redundancy");

    check(begin(frames::CLASS_VECTORED8_GEOM), "begin(vectored8_geom)");
    const int full8 = rank_without(-1, 8);
    check(full8 == 6, "eight thrusters -> full 6-DOF rank");
    bool survives_any_loss = true;
    for (int j = 0; j < 8; ++j) if (rank_without(j, 8) < full8) survives_any_loss = false;
    check(survives_any_loss, "⭐ and it keeps all six axes after ANY single loss");
    printf("        ^ THAT is what redundancy buys. mongla-5 has none of it, so a\n"
           "          thruster failure is a surface-now event, not a degrade-and-continue one.\n");
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
        allocate(d, p, out);
        for (int j = 0; j < 5; ++j) worst = fmaxf(worst, fabsf(out[j]));
        ++n;
    }
    printf("        %ld demands swept, worst |output| = %.9f\n", n, (double)worst);
    check(worst <= 1.0f + 1e-6f, "every output inside [-1, 1]");
    // NaN must not propagate to a single thruster.
    float d[AX_COUNT] = { 0.5f, NAN, 0.2f, 0.1f, 0.0f, -0.3f };
    float out[ALLOC_MAX_THRUSTERS];
    Result r = allocate(d, PRIO_UNIFORM, out);
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
    allocate(d, PRIO_UNIFORM, out);
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
    check(thrusterCount() == 0, "allocator stays disabled (no thrusters reported)");
    float d[AX_COUNT] = {0}, out[ALLOC_MAX_THRUSTERS];
    d[AX_YAW] = 1.0f;
    Result r = allocate(d, PRIO_UNIFORM, out);
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
    test_no_redundancy_is_a_property_of_the_geometry();
    test_bounds_sweep();
    test_vectored8_geom_builds();
    test_legacy_class_is_refused();
    printf("\n=== %s ===\n", g_fail == 0 ? "ALL PASS" : "FAILURES PRESENT");
    return g_fail == 0 ? 0 : 1;
}
