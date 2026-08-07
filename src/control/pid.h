#pragma once

// =============================================================================
//  control/pid — reusable PID with conditional integration (anti-windup),
//  filtered derivative-on-measurement, and output clamping.
//
//  Three properties worth knowing about, because each one was a real defect:
//
//  1. ANTI-WINDUP. The integrator is clamped to +/-imax, but that alone does not
//     stop windup: while the OUTPUT sits pinned at +/-outmax, integrating further
//     cannot possibly help — the extra charge only has to be paid back later as
//     overshoot. The mixer scales every thruster down uniformly on saturation, so
//     saturation is common here, not exotic. We therefore integrate only when the
//     output is inside its limits, or when the error would drive it back toward
//     the linear region (ArduPilot AC_PID does the same thing).
//
//  2. FILTERED DERIVATIVE. A raw (meas - prev)/dt at 500 Hz multiplies gyro noise
//     by 500. Unfiltered D is why D could not be raised during tuning: the term is
//     mostly noise, so it buzzes the motors long before it damps anything. A
//     one-pole low-pass at PID_D_FILT_HZ makes D usable. Cutoff is compile-time on
//     purpose — see config.h.
//
//  3. NaN CONTAINMENT. constrain(NaN, lo, hi) returns NaN, so a single non-finite
//     sample used to poison the integrator PERMANENTLY (every later constrain()
//     passed the NaN straight through) and there was no way back except reset().
//     Attitude and gyro are guarded upstream; depth is not, so a bad Bar30 read
//     reached the depth PID directly. Non-finite inputs are now rejected and a
//     poisoned integrator self-heals.
// =============================================================================

#include <Arduino.h>
#include <math.h>
#include "config.h"      // PID_D_FILT_HZ

class PID {
public:
    void setGains(float kp, float ki, float kd) { _kp = kp; _ki = ki; _kd = kd; }
    void setLimits(float imax, float outmax)    { _imax = imax; _outmax = outmax; }
    // Per-instance filter cutoffs, Hz. 0 disables that filter.
    //   fltd = DERIVATIVE filter, was the single compile-time PID_D_FILT_HZ for every axis.
    //          ArduSub runs 30 Hz on roll/pitch but only 5 Hz on yaw -- one global value
    //          cannot express that, and yaw is the axis with the least useful signal.
    //   fltt = TARGET filter (ArduSub FLTT). We had none: a stepped setpoint went straight
    //          into the proportional term, so every stick movement and every controller
    //          handover arrived as a step the thrusters had to answer instantly.
    void setFilters(float fltd_hz, float fltt_hz) { _fltd = fltd_hz; _fltt = fltt_hz; }

    // One step. Derivative is taken on the measurement (no setpoint-kick).
    float update(float setpoint, float measurement, float dt) {
        // A non-finite input contributes nothing and must not touch stored state.
        if (!isfinite(setpoint) || !isfinite(measurement) || !(dt > 0.0f)) {
            return 0.0f;
        }
        // Self-heal if state was somehow poisoned (belt-and-braces with the check above).
        if (!isfinite(_integ)) _integ = 0.0f;
        if (!isfinite(_deriv)) _deriv = 0.0f;

        // --- target (setpoint) low-pass, ArduSub FLTT -------------------------
        if (_fltt > 0.0f) {
            const float rc_t = 1.0f / (2.0f * (float)M_PI * _fltt);
            const float a_t  = constrain(dt / (dt + rc_t), 0.0f, 1.0f);
            _sp_f = _first ? setpoint : (_sp_f + a_t * (setpoint - _sp_f));
        } else {
            _sp_f = setpoint;
        }
        setpoint = _sp_f;

        const float err = setpoint - measurement;

        // --- filtered derivative-on-measurement -------------------------------
        float d_raw = 0.0f;
        if (!_first) d_raw = -(measurement - _prev_meas) / dt;
        _prev_meas = measurement;

        // Per-instance cutoff when set, else the compile-time default (back-compat).
        const float fc = (_fltd >= 0.0f) ? _fltd
                       : ((PID_D_FILT_HZ > 0.0f) ? (float)PID_D_FILT_HZ : 0.0f);
        if (fc > 0.0f) {
            // Single-pole RC low-pass: a = dt / (dt + 1/(2*pi*fc)).
            const float rc = 1.0f / (2.0f * (float)M_PI * fc);
            const float a  = constrain(dt / (dt + rc), 0.0f, 1.0f);
            // Seed with the first sample rather than ramping up from 0, so D is not
            // artificially suppressed for the first few cycles after a reset.
            _deriv = _first ? d_raw : (_deriv + a * (d_raw - _deriv));
        } else {
            _deriv = d_raw;                      // filter disabled
        }
        _first = false;

        // --- conditional integration (anti-windup) ----------------------------
        // Predict the output WITHOUT adding to the integrator. If it is already
        // saturated and this error pushes further into the same rail, hold the
        // integrator; otherwise integrate normally.
        const float out_unsat = _kp * err + _ki * _integ + _kd * _deriv;
        const bool  push_high = (out_unsat >=  _outmax) && (err > 0.0f);
        const bool  push_low  = (out_unsat <= -_outmax) && (err < 0.0f);
        if (!push_high && !push_low) {
            _integ = constrain(_integ + err * dt, -_imax, _imax);
        }

        const float out = _kp * err + _ki * _integ + _kd * _deriv;
        if (!isfinite(out)) { _integ = 0.0f; _deriv = 0.0f; return 0.0f; }
        return constrain(out, -_outmax, _outmax);
    }

    void reset() { _integ = 0; _first = true; _prev_meas = 0; _deriv = 0; _sp_f = 0; }

    // Current integrator state (for CoB auto-trim / diagnostics).
    float integral() const { return _integ; }

    // Remove `amount` from the integrator, saturating at zero and never changing sign.
    // Used by the CoB auto-trim to actually HAND OVER steady effort to its feedforward
    // trim: it used to only add to the trim while leaving the integrator untouched, so
    // the same charge was transferred over and over and the trim ramped to its clamp.
    void bleedIntegral(float amount) {
        if (!isfinite(amount) || !isfinite(_integ)) return;
        if (_integ > 0.0f)      _integ = max(0.0f, _integ - fabsf(amount));
        else if (_integ < 0.0f) _integ = min(0.0f, _integ + fabsf(amount));
    }

private:
    float _kp = 0, _ki = 0, _kd = 0;
    float _imax = 1.0f, _outmax = 1.0f;
    float _integ = 0, _prev_meas = 0, _deriv = 0;
    float _sp_f  = 0;              // filtered setpoint (FLTT)
    float _fltd  = -1.0f;          // <0 = use PID_D_FILT_HZ
    float _fltt  = 0.0f;           // 0 = target filter off
    bool  _first = true;
};
