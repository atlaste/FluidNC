// Copyright (c) 2024 -	Bart Dring
// Use of this source code is governed by a GPLv3 license that can be found in the LICENSE file.

#pragma once

// TMCStepper 0.7.x knows nothing about the TMC2240, so the chip level register
// access lives here instead of in the library.  TMC2160Stepper is the base
// because it carries the SPI datagram transport that the platform code in
// esp32/tmc_spi.cpp and esp32/tmc_spi_espidf.cpp hooks into by replacing the
// weak TMC2130Stepper::read()/write().
//
// Every field this class inherits was checked against the TMC2240 datasheet
// (Rev.2, 12/2023).  GCONF, CHOPCONF, COOLCONF, IHOLD_IRUN, DRV_STATUS,
// PWMCONF, TSTEP, TCOOLTHRS and THIGH place the fields FluidNC uses at the same
// bit positions as the TMC2160/TMC5160, so those accessors are reused as-is.
// What differs, and is therefore reimplemented below:
//   - The 2240 senses coil current internally.  There is no sense resistor and
//     no vsense bit; full scale current comes from the IREF resistor plus
//     CURRENT_RANGE in DRV_CONF, trimmed by GLOBAL_SCALER and IRUN/IHOLD.
//   - DRV_CONF shares address 0x0A with the TMC2160 register of the same name
//     but holds completely different fields.
//   - tpfd is a CHOPCONF field the library only exposes on TMC5160Stepper.

#include <cstdint>       // Must be before TMCStepper.h
#include <TMCStepper.h>  // https://github.com/teemuatlut/TMCStepper

class TMC2240Stepper : public TMC2160Stepper {
public:
    // r_ref is the IREF resistor in kOhms.  The base class sense resistor is
    // unused, so it gets a harmless placeholder.
    TMC2240Stepper(uint16_t pinCS, float r_ref, int8_t link_index = -1) :
        TMC2160Stepper(pinCS, 0.075f, link_index), _r_ref(r_ref) {}

    // IOIN VERSION of the first (and so far only) silicon revision
    static constexpr uint8_t expected_version = 0x40;

    void begin();

    // Highest RMS current in mA that a CURRENT_RANGE setting can deliver
    uint16_t full_scale_rms_ma(uint8_t range) const;

    // Picks a CURRENT_RANGE, GLOBAL_SCALER and IRUN/IHOLD combination that
    // comes as close as possible to the requested run current in mA RMS.
    void     rms_current(uint16_t mA, float mult);
    uint16_t rms_current();

    // RW: DRV_CONF
    void     DRV_CONF(uint32_t value);
    uint32_t DRV_CONF() { return _drv_conf; }
    void     current_range(uint8_t value);
    uint8_t  current_range() { return _drv_conf & 0x3; }
    void     slope_control(uint8_t value);
    uint8_t  slope_control() { return (_drv_conf >> 4) & 0x3; }

    // RW: CHOPCONF
    void    tpfd(uint8_t value);
    uint8_t tpfd();

private:
    // These inherited members address registers or fields that the TMC2240 does
    // not have, or that it defines differently.  Writing them would corrupt
    // unrelated bits, so they are hidden rather than left as traps.
    using TMC2160Stepper::bbmclks;
    using TMC2160Stepper::bbmtime;
    using TMC2160Stepper::cs2rms;
    using TMC2160Stepper::drvstrength;
    using TMC2160Stepper::ENCM_CTRL;
    using TMC2160Stepper::enc_commutation;
    using TMC2160Stepper::filt_isense;
    using TMC2160Stepper::I_scale_analog;
    using TMC2160Stepper::internal_Rsense;
    using TMC2160Stepper::OFFSET_READ;
    using TMC2160Stepper::otselect;
    using TMC2160Stepper::push;
    using TMC2160Stepper::rndtf;
    using TMC2160Stepper::s2g_level;
    using TMC2160Stepper::s2vs_level;
    using TMC2160Stepper::SHORT_CONF;
    using TMC2160Stepper::shortdelay;
    using TMC2160Stepper::shortfilter;
    using TMC2160Stepper::sync;
    using TMC2160Stepper::VDCMIN;
    using TMC2160Stepper::vsense;

    static constexpr uint8_t DRV_CONF_ADDRESS = 0x0A;

    static constexpr uint8_t MIN_IRUN          = 16;  // below this the microstep table gets coarse
    static constexpr uint8_t MIN_GLOBAL_SCALER = 32;  // 1..31 are not valid settings

    float    _r_ref;         // IREF resistor in kOhms
    uint32_t _drv_conf = 0;  // shadow of the write-mostly DRV_CONF register
};
