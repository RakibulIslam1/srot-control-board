// =============================================================================
//  control/safety_monitor — implementation
// =============================================================================

#include "control/safety_monitor.h"
#include "comms/params.h"
#include <math.h>
#include <stdlib.h>

namespace safety_monitor {

static const float R2D = 57.2957795f;

bool ok(float roll, float pitch, float gx, float gy, float gz,
        float depth, float depth0, const int16_t* rpm, int n_rpm, const char** why,
        bool allow_inverted, bool depth_ref_valid) {
    auto fail = [&](const char* r) { if (why) *why = r; return false; };

    // Every check below is written so a NaN FAILS it. That is not automatic: a comparison
    // against NaN is false, so the natural phrasing `if (x > limit) fail` silently PASSES a
    // NaN — which is backwards for a guard whose entire job is to catch a broken state. The
    // rate and depth checks used to have exactly that hole.
    if (!isfinite(roll) || !isfinite(pitch)) return fail("attitude NaN");
    if (!isfinite(gx) || !isfinite(gy) || !isfinite(gz)) return fail("gyro NaN");

    // Tumbling guard — skipped for a commanded spin (STYLE/STUNT), which is *supposed* to
    // go past vertical. Every other guard below still applies.
    if (!allow_inverted) {
        const float amax = g_params.st_angle_max;
        if (fabsf(roll) * R2D > amax || fabsf(pitch) * R2D > amax) return fail("angle limit");
    }

    const float rmax = g_params.st_rate_max;
    if (fabsf(gx) * R2D > rmax || fabsf(gy) * R2D > rmax || fabsf(gz) * R2D > rmax)
        return fail("rate limit");

    // Depth runaway — only when depth0 is a reference the mode actually commands. See the
    // header: STUNT owns no depth setpoint, so comparing against its frozen entry depth
    // would punish a deliberate pilot climb.
    //
    // A non-finite depth used to SKIP this check entirely (the isfinite() was a
    // precondition, so a NaN read as "safe"). A depth sensor producing NaN is exactly when
    // a depth-driven manoeuvre must stop, so it now fails. depth0 is checked too: it comes
    // from depth::target(), which a bad setpoint could poison.
    if (depth_ref_valid) {
        if (!isfinite(depth) || !isfinite(depth0)) return fail("depth NaN");
        if (fabsf(depth - depth0) > g_params.st_depth_delta)
            return fail("depth runaway");
    }

    if (rpm) {
        const int rmaxrpm = (int)g_params.st_rpm_max;
        for (int i = 0; i < n_rpm; ++i)
            if (abs((int)rpm[i]) > rmaxrpm) return fail("RPM limit");
    }
    return true;
}

}  // namespace safety_monitor
