// =============================================================================
//  control/allocator — implementation. See the header for the rationale.
//
//  THE WHOLE ALGORITHM, in the order it runs:
//
//   begin()      B[6][N]  = per thruster: axis as a force row, r x axis as a
//                           moment row. 6N multiply-adds, once.
//                keep[]   = the rows B can actually produce (a row of all zeros
//                           is an unactuated axis -- computed, never declared).
//                Ainv     = the solution operator, N x 6, built by solving
//                           B_reduced u = e_k for each kept axis k. Square and
//                           full rank -> exact unique solution. Wider than tall
//                           -> minimum-norm solution via B'(BB')^-1, which is the
//                           least total output that produces the wrench exactly.
//                auth[]   = 1 / max_i |Ainv[i][k]|, the most axis k can do before
//                           some thruster hits 1. This is what makes the axes
//                           commensurate.
//
//   allocate()   u = Ainv * (demand .* auth), split into an attitude part and a
//                translation part, then scaled to fit. No matrix work.
//
//  ⛔ THE ONE THING TO GET RIGHT: every loop here is bounded by a compile-time
//  constant and there is no dynamic allocation, no recursion, and no iteration
//  whose count depends on the data. begin() is off the flight loop; allocate() is
//  on it, and it is straight-line code.
// =============================================================================

#include "control/allocator.h"
#include <math.h>
#include <string.h>

namespace allocator {

const char* const AXIS_NAME[AX_COUNT] = {
    "sway", "surge", "heave", "pitch", "roll", "yaw"
};

// A row is unactuated when no thruster contributes to it at all. Tolerance is in
// the units of the matrix: metres for the moment rows, dimensionless for the
// force rows. 1e-6 is far below any real moment arm (the smallest real one in the
// tree is the axial unit's 8.06 mm) and far above float noise in a cross product
// of numbers this size.
static const float UNACT_TOL = 1e-6f;

// Below this the pivot is not a pivot. Same reasoning as above: a real frame's
// reduced matrix has determinant 0.18 and condition number 5.7, so anything near
// this threshold means the table describes a hull that cannot be controlled, and
// refusing to fly it is the correct outcome.
static const float PIVOT_TOL = 1e-9f;

static bool     s_ready = false;
static int      s_cls   = -1;
static const char* s_name = "none";
static int      s_n     = 0;

static float    s_B[AX_COUNT][ALLOC_MAX_THRUSTERS];       // wrench matrix
static float    s_A[ALLOC_MAX_THRUSTERS][AX_COUNT];       // solution operator
static float    s_auth[AX_COUNT];
static unsigned s_unact = 0;

// ── a small dense linear solver ───────────────────────────────────────────────
//
// Gauss-Jordan with partial pivoting on an m x m system with `rhs` right-hand
// sides solved simultaneously. m <= 6 here, so this is a few hundred flops and
// runs once. Deliberately NOT a library: the whole point of doing it here is that
// the numbers and the failure mode are both inspectable.
//
// Returns false on a singular pivot, having touched nothing the caller relies on.
static bool solveInPlace(float* a, int m, float* b, int rhs) {
    for (int c = 0; c < m; ++c) {
        // partial pivot: the largest remaining |a[r][c]|
        int piv = c;
        float best = fabsf(a[c * m + c]);
        for (int r = c + 1; r < m; ++r) {
            float v = fabsf(a[r * m + c]);
            if (v > best) { best = v; piv = r; }
        }
        if (best < PIVOT_TOL) return false;
        if (piv != c) {
            for (int k = 0; k < m; ++k)   { float t = a[c * m + k];   a[c * m + k]   = a[piv * m + k];   a[piv * m + k]   = t; }
            for (int k = 0; k < rhs; ++k) { float t = b[c * rhs + k]; b[c * rhs + k] = b[piv * rhs + k]; b[piv * rhs + k] = t; }
        }
        const float inv = 1.0f / a[c * m + c];
        for (int k = 0; k < m; ++k)   a[c * m + k]   *= inv;
        for (int k = 0; k < rhs; ++k) b[c * rhs + k] *= inv;
        for (int r = 0; r < m; ++r) {
            if (r == c) continue;
            const float f = a[r * m + c];
            if (f == 0.0f) continue;
            for (int k = 0; k < m; ++k)   a[r * m + k]   -= f * a[c * m + k];
            for (int k = 0; k < rhs; ++k) b[r * rhs + k] -= f * b[c * rhs + k];
        }
    }
    return true;
}

static void cross3(const float a[3], const float b[3], float out[3]) {
    out[0] = a[1] * b[2] - a[2] * b[1];
    out[1] = a[2] * b[0] - a[0] * b[2];
    out[2] = a[0] * b[1] - a[1] * b[0];
}

// Build s_A (and s_auth) from s_B, using only the thrusters marked available.
// Split out of begin() because it is also what a thruster failure needs: the
// geometry has not changed, only which columns may be used.
//
// ⚠ NOT CALLED FROM THE FLIGHT LOOP. It solves a linear system. A failure has to
// be handled off-loop (recompute, then publish), which is also the honest place
// to decide whether the vehicle should still be flying at all.
static bool build(const bool* avail) {
    memset(s_A, 0, sizeof(s_A));
    for (int k = 0; k < AX_COUNT; ++k) s_auth[k] = 0.0f;

    // Which axes are producible with the columns we are allowed to use?
    int keep[AX_COUNT], m = 0;
    s_unact = 0;
    for (int i = 0; i < AX_COUNT; ++i) {
        float mx = 0.0f;
        for (int j = 0; j < s_n; ++j) {
            if (avail && !avail[j]) continue;
            float v = fabsf(s_B[i][j]);
            if (v > mx) mx = v;
        }
        if (mx > UNACT_TOL) keep[m++] = i;
        else                s_unact |= (1u << i);
    }
    if (m == 0) return false;

    // The usable columns, in order.
    int col[ALLOC_MAX_THRUSTERS], nc = 0;
    for (int j = 0; j < s_n; ++j) if (!avail || avail[j]) col[nc++] = j;
    if (nc < m) return false;   // fewer thrusters than producible axes: underdetermined

    // Br is m x nc. Solve for the outputs that produce each unit axis wrench.
    //
    //   nc == m : Br u = e_k has one solution. Solve directly.
    //   nc >  m : infinitely many. Take the minimum-norm one, u = Br'(Br Br')^-1 e_k,
    //             which is the least total thruster output that hits the wrench
    //             exactly -- the right default, because thrust costs current and
    //             every newton spent opposing another thruster is wasted.
    float G[AX_COUNT * AX_COUNT];      // Br Br'  (m x m), or Br itself when square
    float R[AX_COUNT * AX_COUNT];      // rhs: the m x m identity, solved in place
    for (int r = 0; r < m; ++r)
        for (int c = 0; c < m; ++c) R[r * m + c] = (r == c) ? 1.0f : 0.0f;

    if (nc == m) {
        for (int r = 0; r < m; ++r)
            for (int c = 0; c < m; ++c) G[r * m + c] = s_B[keep[r]][col[c]];
        if (!solveInPlace(G, m, R, m)) return false;
        // R is now Br^-1 (m x m): row = usable-column index, col = kept-axis index.
        for (int c = 0; c < m; ++c)
            for (int k = 0; k < m; ++k) s_A[col[c]][keep[k]] = R[c * m + k];
    } else {
        for (int r = 0; r < m; ++r)
            for (int c = 0; c < m; ++c) {
                float s = 0.0f;
                for (int j = 0; j < nc; ++j) s += s_B[keep[r]][col[j]] * s_B[keep[c]][col[j]];
                G[r * m + c] = s;
            }
        if (!solveInPlace(G, m, R, m)) return false;   // R = (Br Br')^-1
        for (int j = 0; j < nc; ++j)
            for (int k = 0; k < m; ++k) {
                float s = 0.0f;
                for (int r = 0; r < m; ++r) s += s_B[keep[r]][col[j]] * R[r * m + k];
                s_A[col[j]][keep[k]] = s;
            }
    }

    // Authority: the largest wrench on one axis before some thruster reaches 1.
    for (int k = 0; k < m; ++k) {
        const int ax = keep[k];
        float mx = 0.0f;
        for (int j = 0; j < s_n; ++j) {
            float v = fabsf(s_A[j][ax]);
            if (v > mx) mx = v;
        }
        s_auth[ax] = (mx > PIVOT_TOL) ? (1.0f / mx) : 0.0f;
    }
    return true;
}

bool begin(int cls) {
    s_ready = false;
    s_cls = cls;
    s_name = "none";
    s_n = 0;
    memset(s_B, 0, sizeof(s_B));
    memset(s_A, 0, sizeof(s_A));

    const frames::Frame* f = frames::get(cls);
    if (!f || f->n <= 0 || f->n > ALLOC_MAX_THRUSTERS) return false;
    s_name = f->name;
    s_n = f->n;

    for (int j = 0; j < s_n; ++j) {
        const float* a = f->t[j].axis;
        const float  n = sqrtf(a[0] * a[0] + a[1] * a[1] + a[2] * a[2]);
        // ⛔ A non-unit axis is a table error, not something to normalise away
        // quietly: it usually means somebody wrote a force vector where a
        // direction belongs, and the magnitude they intended is then lost.
        if (!(n > 0.9f) || !(n < 1.1f)) return false;
        float unit[3] = { a[0] / n, a[1] / n, a[2] / n };
        float mom[3];
        cross3(f->t[j].pos, unit, mom);
        s_B[AX_SWAY][j]  = unit[0];
        s_B[AX_SURGE][j] = unit[1];
        s_B[AX_HEAVE][j] = unit[2];
        s_B[AX_PITCH][j] = mom[0];
        s_B[AX_ROLL][j]  = mom[1];
        s_B[AX_YAW][j]   = mom[2];
    }

    if (!build(nullptr)) return false;
    s_ready = true;
    return true;
}

bool rebuild(const bool* avail) {
    if (s_cls < 0 || s_n <= 0) return false;
    // Keep the previous solution if the new one is not viable: a hull that has
    // just lost an axis is not improved by also losing its allocator.
    float A_save[ALLOC_MAX_THRUSTERS][AX_COUNT];
    float auth_save[AX_COUNT];
    unsigned unact_save = s_unact;
    memcpy(A_save, s_A, sizeof(s_A));
    memcpy(auth_save, s_auth, sizeof(s_auth));
    if (build(avail)) return true;
    memcpy(s_A, A_save, sizeof(s_A));
    memcpy(s_auth, auth_save, sizeof(s_auth));
    s_unact = unact_save;
    return false;
}

bool        ready()         { return s_ready; }
int         frameClass()    { return s_cls; }
const char* frameName()     { return s_name; }
int         thrusterCount() { return s_ready ? s_n : 0; }
unsigned    unactuatedMask(){ return s_unact; }

float authority(int axis) {
    if (!s_ready || axis < 0 || axis >= AX_COUNT) return 0.0f;
    return s_auth[axis];
}

void thrusterWrench(int j, float wrench[AX_COUNT]) {
    for (int i = 0; i < AX_COUNT; ++i)
        wrench[i] = (s_ready && j >= 0 && j < s_n) ? s_B[i][j] : 0.0f;
}

void achieved(const float out[ALLOC_MAX_THRUSTERS], float wrench[AX_COUNT]) {
    for (int i = 0; i < AX_COUNT; ++i) {
        float s = 0.0f;
        if (s_ready) for (int j = 0; j < s_n; ++j) s += s_B[i][j] * out[j];
        wrench[i] = s;
    }
}

// The largest s in [0,1] with |a + s*t| <= 1, given |a| <= 1. Exact, branchless
// apart from the sign test, and it is the whole of the priority scaling.
static float headroom(float a, float t) {
    if (fabsf(t) < 1e-9f) return 1.0f;               // this thruster does not care
    const float lim = (t > 0.0f) ? (1.0f - a) / t : (-1.0f - a) / t;
    return (lim < 0.0f) ? 0.0f : (lim > 1.0f ? 1.0f : lim);
}

Result allocate(const float demand[AX_COUNT], const bool* avail,
                int priority, float out[ALLOC_MAX_THRUSTERS]) {
    Result r;
    r.ok = false;
    r.scale_attitude = 0.0f;
    r.scale_translation = 0.0f;
    r.limiting_thruster = -1;
    r.refused_mask = 0;

    for (int j = 0; j < ALLOC_MAX_THRUSTERS; ++j) out[j] = 0.0f;
    if (!s_ready) return r;

    // ⛔ REFUSE, DO NOT ROUND. An unactuated axis with a live demand on it is a
    // caller believing something false about the vehicle, and the only useful
    // response is to say so. A non-finite demand is refused the same way rather
    // than propagated: one NaN here would reach every thruster.
    float d[AX_COUNT];
    for (int i = 0; i < AX_COUNT; ++i) {
        const float v = demand[i];
        if (!isfinite(v)) { d[i] = 0.0f; r.refused_mask |= (1u << i); continue; }
        if ((s_unact & (1u << i)) && fabsf(v) > 1e-6f) { d[i] = 0.0f; r.refused_mask |= (1u << i); continue; }
        d[i] = (v < -1.0f) ? -1.0f : (v > 1.0f ? 1.0f : v);
    }

    // Two partial solutions, so the scaling can give one of them precedence.
    // Available thrusters only: an unavailable column has a zero row in s_A when
    // build() was told about it, but a caller may pass `avail` per frame without
    // having rebuilt, so mask here too rather than trusting that.
    float ua[ALLOC_MAX_THRUSTERS] = {0}, ut[ALLOC_MAX_THRUSTERS] = {0};
    for (int j = 0; j < s_n; ++j) {
        if (avail && !avail[j]) continue;
        ua[j] = s_A[j][AX_PITCH] * d[AX_PITCH] * s_auth[AX_PITCH]
              + s_A[j][AX_ROLL]  * d[AX_ROLL]  * s_auth[AX_ROLL]
              + s_A[j][AX_YAW]   * d[AX_YAW]   * s_auth[AX_YAW];
        ut[j] = s_A[j][AX_SWAY]  * d[AX_SWAY]  * s_auth[AX_SWAY]
              + s_A[j][AX_SURGE] * d[AX_SURGE] * s_auth[AX_SURGE]
              + s_A[j][AX_HEAVE] * d[AX_HEAVE] * s_auth[AX_HEAVE];
    }

    if (priority == PRIO_ATTITUDE) {
        // Attitude first, in full if it fits.
        float mx = 0.0f; int lim_j = -1;
        for (int j = 0; j < s_n; ++j) {
            const float v = fabsf(ua[j]);
            if (v > mx) { mx = v; lim_j = j; }
        }
        float sa = 1.0f;
        if (mx > 1.0f) { sa = 1.0f / mx; r.limiting_thruster = lim_j; }
        for (int j = 0; j < s_n; ++j) ua[j] *= sa;

        // Translation gets whatever is left, per thruster, worst case wins.
        float st = 1.0f;
        for (int j = 0; j < s_n; ++j) {
            const float h = headroom(ua[j], ut[j]);
            if (h < st) { st = h; if (sa >= 1.0f) r.limiting_thruster = j; }
        }
        for (int j = 0; j < s_n; ++j) out[j] = ua[j] + st * ut[j];
        r.scale_attitude = sa;
        r.scale_translation = st;
    } else {
        // Uniform: one scale for everything, which is what mixer::mix() does today.
        float mx = 0.0f; int lim_j = -1;
        for (int j = 0; j < s_n; ++j) {
            out[j] = ua[j] + ut[j];
            const float v = fabsf(out[j]);
            if (v > mx) { mx = v; lim_j = j; }
        }
        float s = 1.0f;
        if (mx > 1.0f) { s = 1.0f / mx; r.limiting_thruster = lim_j; }
        for (int j = 0; j < s_n; ++j) out[j] *= s;
        r.scale_attitude = s;
        r.scale_translation = s;
    }

    // Belt and braces. Nothing above can exceed 1 by construction, so if this
    // ever bites, the construction is wrong and the clamp is hiding it -- which
    // is why the native test asserts the pre-clamp bound directly.
    for (int j = 0; j < s_n; ++j)
        out[j] = (out[j] < -1.0f) ? -1.0f : (out[j] > 1.0f ? 1.0f : out[j]);

    r.ok = true;
    return r;
}

}  // namespace allocator
