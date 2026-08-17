// Copyright (c) 2024 -	Bart Dring
// Use of this source code is governed by a GPLv3 license that can be found in the LICENSE file.

#include "TMC2240Stepper.h"

#include <cmath>

void TMC2240Stepper::begin() {
    TMC2160Stepper::begin();
    DRV_CONF(_drv_conf);
}

// I_FS = K_IFS / R_REF, where K_IFS depends on CURRENT_RANGE.  The datasheet
// values are peak amps times kOhms; scaling by 1000 gives mA and dividing by
// sqrt(2) converts the peak to the RMS figure FluidNC configures.
uint16_t TMC2240Stepper::full_scale_rms_ma(uint8_t range) const {
    static constexpr float k_ifs[4] = { 11750.0f, 24000.0f, 36000.0f, 36000.0f };

    return static_cast<uint16_t>(k_ifs[range & 0x3] / _r_ref / 1.4142136f);
}

// I_RMS = I_FS / sqrt(2) * GLOBALSCALER/256 * (IRUN+1)/32
//
// The lowest CURRENT_RANGE that reaches the requested current gives the finest
// resolution and the tightest overcurrent threshold.  Within that range IRUN is
// kept as high as possible - the datasheet recommends 16..31 because IRUN also
// scales the microstep sine table - while holding GLOBALSCALER at 128 or above,
// which is where the chopper hysteresis behaves best.
void TMC2240Stepper::rms_current(uint16_t mA, float mult) {
    holdMultiplier = mult;

    uint8_t range = 3;
    for (uint8_t r = 0; r < 4; r++) {
        if (mA <= full_scale_rms_ma(r)) {
            range = r;
            break;
        }
    }
    current_range(range);

    const float full_scale = full_scale_rms_ma(range);

    uint8_t cs     = 31;
    int32_t scaler = 0;
    while (true) {
        scaler = lroundf(256.0f * 32.0f * mA / (full_scale * (cs + 1)));

        if (scaler > 255) {
            scaler = 0;  // 0 means full scale, i.e. 256/256
            break;
        }
        if (scaler >= 128 || cs == 0) {
            break;
        }
        cs--;  // a smaller run current setting needs a larger scaler
    }

    GLOBAL_SCALER(scaler);
    irun(cs);
    ihold(static_cast<uint8_t>(cs * holdMultiplier));
}

uint16_t TMC2240Stepper::rms_current() {
    uint16_t scaler = GLOBAL_SCALER();
    if (scaler == 0) {
        scaler = 256;
    }

    return static_cast<uint32_t>(full_scale_rms_ma(current_range())) * scaler * (irun() + 1) / (256 * 32);
}

void TMC2240Stepper::DRV_CONF(uint32_t value) {
    _drv_conf = value & 0x33;
    write(DRV_CONF_ADDRESS, _drv_conf);
}

void TMC2240Stepper::current_range(uint8_t value) {
    DRV_CONF((_drv_conf & ~0x3) | (value & 0x3));
}

void TMC2240Stepper::slope_control(uint8_t value) {
    DRV_CONF((_drv_conf & ~0x30) | ((value & 0x3) << 4));
}

void TMC2240Stepper::tpfd(uint8_t value) {
    CHOPCONF_register.tpfd = value;
    write(CHOPCONF_register.address, CHOPCONF_register.sr);
}

uint8_t TMC2240Stepper::tpfd() {
    CHOPCONF_t r { 0 };
    r.sr = CHOPCONF();
    return r.tpfd;
}
