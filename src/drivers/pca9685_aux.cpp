// =============================================================================
//  drivers/pca9685_aux — implementation (Adafruit PWM Servo Driver)
// =============================================================================

#include "drivers/pca9685_aux.h"
#include <Adafruit_PWMServoDriver.h>

namespace pca9685_aux {

static Adafruit_PWMServoDriver* s_pwm = nullptr;
static bool s_ok = false;

bool begin(TwoWire* wire) {
    static Adafruit_PWMServoDriver pwm(I2C_ADDR_PCA9685, *wire);
    s_pwm = &pwm;
    s_pwm->begin();
    s_pwm->setOscillatorFrequency(27000000);
    s_pwm->setPWMFreq(PCA9685_PWM_FREQ);
    s_ok = true;
    return s_ok;
}

void writeState(const uint8_t  role[PCA9685_NUM_CH],
                const uint16_t servo_us[PCA9685_NUM_CH],
                const bool     switch_on[PCA9685_NUM_CH]) {
    if (!s_ok || !s_pwm) return;
    for (int ch = 0; ch < PCA9685_NUM_CH; ++ch) {
        switch (role[ch]) {
            case PCA_SERVO:
                if (servo_us[ch] > 0) s_pwm->writeMicroseconds(ch, servo_us[ch]);
                else                  s_pwm->setPWM(ch, 0, 4096);   // disabled → output off
                break;
            case PCA_SWITCH: {
                bool on = SWITCH_ACTIVE_HIGH ? switch_on[ch] : !switch_on[ch];
                // Full-ON = setPWM(ch, 4096, 0); Full-OFF = setPWM(ch, 0, 4096).
                if (on) s_pwm->setPWM(ch, 4096, 0);
                else    s_pwm->setPWM(ch, 0, 4096);
                break;
            }
            case PCA_DISABLED:
            default:
                s_pwm->setPWM(ch, 0, 4096);   // off
                break;
        }
    }
}

bool healthy() { return s_ok; }

}  // namespace pca9685_aux
