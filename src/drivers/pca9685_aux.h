#pragma once

// =============================================================================
//  drivers/pca9685_aux — PCA9685 16-ch servo + payload switch expander (Bus1)
//
//  Per-channel role is RUNTIME (SERVOn_ROLE param): 0 = off, 1 = PWM servo (µs),
//  2 = MOSFET/relay full-ON/OFF. The caller passes the role[] array each write.
// =============================================================================

#include <Arduino.h>
#include <Wire.h>
#include "config.h"

namespace pca9685_aux {

bool begin(TwoWire* wire);

// Push the desired outputs. role[]: 0 = off, 1 = servo (uses servo_us[]),
// 2 = MOSFET/switch (uses switch_on[]).
void writeState(const uint8_t  role[PCA9685_NUM_CH],
                const uint16_t servo_us[PCA9685_NUM_CH],
                const bool     switch_on[PCA9685_NUM_CH]);

bool healthy();

}  // namespace pca9685_aux
