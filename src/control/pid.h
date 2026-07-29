#pragma once

// =============================================================================
//  control/pid — reusable PID with I-clamp, output-clamp, derivative-on-measure
// =============================================================================

#include <Arduino.h>

class PID {
public:
    void setGains(float kp, float ki, float kd) { _kp = kp; _ki = ki; _kd = kd; }
    void setLimits(float imax, float outmax)    { _imax = imax; _outmax = outmax; }

    // One step. Derivative is taken on the measurement (no setpoint-kick).
    float update(float setpoint, float measurement, float dt) {
        float err = setpoint - measurement;

        _integ += err * dt;
        _integ = constrain(_integ, -_imax, _imax);

        float deriv = 0.0f;
        if (!_first && dt > 0.0f) {
            deriv = -(measurement - _prev_meas) / dt;
        }
        _prev_meas = measurement;
        _first = false;

        float out = _kp * err + _ki * _integ + _kd * deriv;
        return constrain(out, -_outmax, _outmax);
    }

    void reset() { _integ = 0; _first = true; _prev_meas = 0; }

    // Current integrator state (for CoB auto-trim / diagnostics).
    float integral() const { return _integ; }

private:
    float _kp = 0, _ki = 0, _kd = 0;
    float _imax = 1.0f, _outmax = 1.0f;
    float _integ = 0, _prev_meas = 0;
    bool  _first = true;
};
