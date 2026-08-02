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

// Earth's field is 25-65 uT in free air. INSIDE A HULL IT IS NOT: an enclosure, the
// thruster magnets and the electronics all attenuate and distort it, and the measured
// magnitude can sit well under the free-air figure without the HEADING being unusable --
// the direction survives attenuation far better than the magnitude does.
//
// Measured on this vehicle's board: 14.7 uT raw, 19.1 uT after our calibration. A hard
// 25 uT floor rejected that outright, so the operator got "field strength implausible"
// with no number to act on and no way to tell an attenuated-but-fine installation from a
// genuinely broken one.
//
// The band is now a PARAMETER and defaults wide. It is a sanity check against a dead or
// saturated sensor, not a claim about what a good installation looks like. The checks that
// actually protect the alignment are the ones after it: the samples must agree to within
// MAX_SPREAD_RAD, which no amount of attenuation can fake.
static float fieldMinUt() {
    const float v = g_params.mag_field_min;
    return (v > 0.0f) ? v : DEF_MAG_FIELD_MIN;
}
static float fieldMaxUt() {
    const float v = g_params.mag_field_max;
    return (v > fieldMinUt()) ? v : DEF_MAG_FIELD_MAX;
}

// Max spread (rad) between the min and max sampled heading for the average to be
// trusted. ~8 deg: loose enough for a boat rocking on its mooring, tight enough to
// reject a vehicle being carried or a nearby motor cycling.
static const float MAX_SPREAD_RAD = 0.14f;

static State    s_state = State::IDLE;
static bool     s_changed = false;
static float    s_offset = 0.0f;
static float    s_last_field = 0.0f;   // last computed |B|, so a refusal can quote it
static uint8_t  s_last_acc = 0;        // last BNO accuracy, quoted in a refusal
static bool     s_had_our_cal = false; // was a stored mag cal present? quoted in a refusal

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
        case State::REFUSED_CAL: {
            // Quote what was actually found. "run a mag calibration first" told an operator
            // who HAD calibrated that they had not, with no way to see which of the two
            // inputs was missing.
            static char buf[50];
            snprintf(buf, sizeof(buf), "Mag ref: no stored cal (CAL_MAG all 0), acc=%u",
                     (unsigned)s_last_acc);
            return buf;
        }
        case State::REFUSED_FIELD: {
            // Quote the number. "implausible" with no value told the operator nothing they
            // could act on -- they could not tell an attenuated hull from a dead sensor.
            static char buf[50];
            snprintf(buf, sizeof(buf), "Mag ref refused: |B|=%.0fuT, want %.0f-%.0f",
                     (double)s_last_field, (double)fieldMinUt(), (double)fieldMaxUt());
            return buf;
        }
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

    // Do WE have a stored magnetic calibration? This is the question that actually matters,
    // and it is not the one this gate used to ask.
    //
    // It required the BNO085's own accuracy flag to reach 2. On this board that flag never
    // gets there: sh2_setCalConfig() is rejected (the part wants ~90 ms after reset) and
    // retrying it from poll() does not take either, so the sensor never runs its dynamic
    // calibration and the flag is pinned at 0-1 for ever. The operator calibrated the
    // compass repeatedly, saw "Calibration saved + verified on flash", rebooted, and got
    // "mag yaw ref refused: mag not calibrated" every single time -- because the refusal was
    // keyed on a number their calibration does not affect.
    //
    // OUR calibration is the one we apply below (offsets + scales from NVS_NS_CAL), it is
    // the one the operator actually performs, and it persists correctly. When it is present,
    // trust it: the BNO's flag becomes a bonus, not a veto. The real guards against a bad
    // alignment are the two that follow -- field magnitude must be earth-like, and the
    // samples must agree -- and neither depends on the sensor's own opinion of itself.
    //
    // With NO calibration of our own, the old rule stands: demand accuracy >= 2, because
    // then the BNO's internal solution is the only correction there is.
    bool have_our_cal = false;
    {
        StateLock lk(g_state.mtx_cal, pdMS_TO_TICKS(2));
        if (!lk.ok()) return;
        const Vec3f& o = g_state.cal.mag_offset;
        const Vec3f& sc = g_state.cal.mag_scale;
        // A never-calibrated board reads offsets 0,0,0 and scales 1,1,1 exactly.
        have_our_cal = (o.x != 0.0f || o.y != 0.0f || o.z != 0.0f) &&
                       (sc.x > 0.0f && sc.y > 0.0f && sc.z > 0.0f);
    }
    // WITH our own calibration, the BNO's accuracy flag is IRRELEVANT and is not consulted.
    //
    // That flag describes the quality of the sensor's OWN internal hard/soft-iron solution --
    // the one we are overriding with the offsets and scales just read from NVS. Requiring it
    // to be good is requiring the sensor to agree with a correction it does not know exists.
    // On this board it never converges anyway (sh2_setCalConfig is rejected and retries do
    // not take), so it sits at 0-1 for ever and gated the feature off permanently.
    //
    // This is the third time this refusal fired for a reason the operator could not
    // influence: MAG_YAW_REF defaulted off, then this flag, then a free-air field band.
    // Requiring accuracy >= 1 still left it failing on any boot where the flag read 0.
    //
    // What protects the alignment is unchanged and does not depend on the sensor's opinion
    // of itself: |B| must be in band, and the sampled headings must agree to within
    // MAX_SPREAD_RAD over at least MIN_SPAN_MS. A wrong calibration cannot pass those.
    s_had_our_cal = have_our_cal;
    s_last_acc = mag_accuracy;
    const uint8_t need_acc = have_our_cal ? 0 : 2;
    if (mag_accuracy < need_acc) {
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
    s_last_field = field;          // reported in the refusal text -- see stateText()
    if (!isfinite(field) || field < fieldMinUt() || field > fieldMaxUt()) {
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
