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
        bool allow_inverted) {
    auto fail = [&](const char* r) { if (why) *why = r; return false; };

    if (!isfinite(roll) || !isfinite(pitch)) return fail("attitude NaN");

    // Tumbling guard — skipped for a commanded spin (STYLE/STUNT), which is *supposed* to
    // go past vertical. Every other guard below still applies.
    if (!allow_inverted) {
        const float amax = g_params.st_angle_max;
        if (fabsf(roll) * R2D > amax || fabsf(pitch) * R2D > amax) return fail("angle limit");
    }

    const float rmax = g_params.st_rate_max;
    if (fabsf(gx) * R2D > rmax || fabsf(gy) * R2D > rmax || fabsf(gz) * R2D > rmax)
        return fail("rate limit");

    if (isfinite(depth) && fabsf(depth - depth0) > g_params.st_depth_delta)
        return fail("depth runaway");

    if (rpm) {
        const int rmaxrpm = (int)g_params.st_rpm_max;
        for (int i = 0; i < n_rpm; ++i)
            if (abs((int)rpm[i]) > rmaxrpm) return fail("RPM limit");
    }
    return true;
}

}  // namespace safety_monitor
