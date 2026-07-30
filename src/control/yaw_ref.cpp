// =============================================================================
//  control/yaw_ref — implementation. See the header for the rationale.
// =============================================================================

#include "control/yaw_ref.h"
#include "comms/params.h"
#include "state_types.h"
#include "config.h"
#include <math.h>

namespace yaw_ref {

// Sampling window. 100 samples at the sensor task rate is well under a second of
// data, but we also hold a minimum wall-clock span so a burst of identical samples
// can't satisfy the stability test trivially.
static const uint16_t NEED_SAMPLES = 100;
static const uint32_t MIN_SPAN_MS  = 1000;

// Earth's field is 25-65 uT everywhere on the surface. Outside that window the
// reading is dominated by something local (hull, magnet, live wire) and the
// heading derived from it is meaningless.
static const float FIELD_MIN_UT = 25.0f;
static const float FIELD_MAX_UT = 65.0f;

// Max spread (rad) between the min and max sampled heading for the average to be
// trusted. ~8 deg: loose enough for a boat rocking on its mooring, tight enough to
// reject a vehicle being carried or a nearby motor cycling.
static const float MAX_SPREAD_RAD = 0.14f;

static State    s_state = State::IDLE;
static bool     s_changed = false;
static float    s_offset = 0.0f;

static uint16_t s_n = 0;
static uint32_t s_start_ms = 0;
static float    s_sum_sin = 0, s_sum_cos = 0;      // circular mean of the mag heading
static float    s_sum_dsin = 0, s_sum_dcos = 0;    // circular mean of (mag - game) delta
static float    s_min_h = 0, s_max_h = 0;
static float    s_field_sum = 0;

static void setState(State s) {
    if (s_state != s) { s_state = s; s_changed = true; }
}

static float wrapPi(float a) {
    if (!isfinite(a)) return 0.0f;
    while (a >  (float)M_PI) a -= 2.0f * (float)M_PI;
    while (a < -(float)M_PI) a += 2.0f * (float)M_PI;
    return a;
}

static void clearAccum() {
    s_n = 0; s_start_ms = 0;
    s_sum_sin = s_sum_cos = 0;
    s_sum_dsin = s_sum_dcos = 0;
    s_min_h = s_max_h = 0;
    s_field_sum = 0;
}

void reset() {
    clearAccum();
    s_offset = 0.0f;
    setState(State::IDLE);
}

float offset() { return s_offset; }
State state()  { return s_state; }

bool takeStateChange() { bool c = s_changed; s_changed = false; return c; }

const char* stateText() {
    switch (s_state) {
        case State::SAMPLING:      return "Mag yaw ref: sampling";
        case State::LOCKED:        return "Mag yaw ref: LOCKED - heading is absolute";
        case State::REFUSED_CAL:   return "Mag yaw ref refused: mag not calibrated";
        case State::REFUSED_FIELD: return "Mag yaw ref refused: field strength implausible";
        case State::REFUSED_NOISE: return "Mag yaw ref refused: unstable - hold still, retry";
        default:                   return "Mag yaw ref: off";
    }
}

void update(float mx, float my, float mz, uint8_t mag_accuracy,
            float roll, float pitch, float game_yaw, bool imu_ok) {
    // Disabled, or already settled (locked or refused) — do nothing. A refusal is
    // sticky on purpose: retrying forever would flap the reported heading. MAG_ALIGN
    // is the explicit way to try again.
    if (g_params.mag_yaw_ref < 0.5f) {
        if (s_state != State::IDLE) reset();
        return;
    }
    if (s_state == State::LOCKED || s_state == State::REFUSED_CAL ||
        s_state == State::REFUSED_FIELD || s_state == State::REFUSED_NOISE) return;

    if (!imu_ok) return;                 // attitude not trustworthy -> tilt comp would be wrong

    // Never align while armed: the thrusters are the interference source we are trying
    // to be immune to, and shifting the heading reference mid-dive would step the
    // heading-hold target.
    {
        StateLock lk(g_state.mtx_control, pdMS_TO_TICKS(2));
        if (lk.ok() && g_state.control.armed) return;
    }

    // The BNO085 reports its own mag calibration confidence 0..3. Below 2 the hard-iron
    // solution has not converged and the heading can be tens of degrees out.
    if (mag_accuracy < 2) {
        // Give it time to converge before giving up — accuracy climbs as the vehicle is
        // moved around. Only refuse once the window has fully elapsed.
        if (s_start_ms == 0) { s_start_ms = millis(); setState(State::SAMPLING); }
        if (millis() - s_start_ms > 20000) setState(State::REFUSED_CAL);
        return;
    }

    // Apply the stored hard/soft-iron calibration. SensorState publishes the mag RAW
    // (task_sensor_read applies only the AHRS mounting trim), so the consumer applies
    // cal — same convention as the other mag consumers.
    Vec3f off{0, 0, 0}, scl{1, 1, 1};
    {
        StateLock lk(g_state.mtx_cal, pdMS_TO_TICKS(2));
        if (!lk.ok()) return;            // try again next cycle; never guess at cal
        off = g_state.cal.mag_offset;
        scl = g_state.cal.mag_scale;
    }
    const float cx = (mx - off.x) * ((scl.x != 0.0f) ? scl.x : 1.0f);
    const float cy = (my - off.y) * ((scl.y != 0.0f) ? scl.y : 1.0f);
    const float cz = (mz - off.z) * ((scl.z != 0.0f) ? scl.z : 1.0f);

    const float field = sqrtf(cx * cx + cy * cy + cz * cz);
    if (!isfinite(field) || field < FIELD_MIN_UT || field > FIELD_MAX_UT) {
        if (s_start_ms == 0) { s_start_ms = millis(); setState(State::SAMPLING); }
        if (millis() - s_start_ms > 20000) setState(State::REFUSED_FIELD);
        return;
    }

    // TILT COMPENSATION. A bare atan2(my, mx) is only valid dead level; a sub sitting
    // a few degrees down at the bow would otherwise read a heading error of the same
    // order as the thing we are trying to correct. Project the field onto the local
    // horizontal plane using the (mag-free, therefore trustworthy) roll/pitch.
    const float sr = sinf(roll),  cr = cosf(roll);
    const float sp = sinf(pitch), cp = cosf(pitch);
    const float xh = cx * cp + cy * sr * sp - cz * cr * sp;
    const float yh = cy * cr + cz * sr;
    if (fabsf(xh) < 1e-6f && fabsf(yh) < 1e-6f) return;     // degenerate — straight down
    const float heading = atan2f(-yh, xh);
    if (!isfinite(heading)) return;

    // Accumulate. Headings are circular, so average the unit vectors, not the angles —
    // a naive mean of values either side of +/-pi lands 180 deg wrong.
    if (s_n == 0) { s_start_ms = millis(); s_min_h = s_max_h = heading; setState(State::SAMPLING); }
    s_sum_sin += sinf(heading);
    s_sum_cos += cosf(heading);
    const float delta = wrapPi(heading - game_yaw);
    s_sum_dsin += sinf(delta);
    s_sum_dcos += cosf(delta);
    s_field_sum += field;
    if (heading < s_min_h) s_min_h = heading;
    if (heading > s_max_h) s_max_h = heading;
    s_n++;

    if (s_n < NEED_SAMPLES || (millis() - s_start_ms) < MIN_SPAN_MS) return;

    // Stability gate. Use the resultant length of the circular mean rather than
    // max-min: max-min is wrong across the +/-pi wrap, and R falls off with spread
    // regardless of where the samples sit. R == 1 is perfectly consistent.
    const float R = sqrtf(s_sum_sin * s_sum_sin + s_sum_cos * s_sum_cos) / (float)s_n;
    // R = cos(spread) approximately for a small symmetric spread.
    if (R < cosf(MAX_SPREAD_RAD)) {
        setState(State::REFUSED_NOISE);
        clearAccum();
        return;
    }

    // offset = (mag heading - game yaw) + declination, as a circular mean.
    const float decl = g_params.mag_decl * (float)M_PI / 180.0f;
    s_offset = wrapPi(atan2f(s_sum_dsin, s_sum_dcos) + decl);
    clearAccum();
    setState(State::LOCKED);
}

}  // namespace yaw_ref
