// Copyright (c) 2024 -	Bart Dring
// Use of this source code is governed by a GPLv3 license that can be found in the LICENSE file.

/*
    This is used for the Trinamic TMC2240 SPI controlled stepper motor driver.
*/

#include "TMC2240Driver.h"
#include "Machine/MachineConfig.h"
#include <atomic>

namespace MotorDrivers {

    void TMC2240Driver::init() {
        TrinamicSpiDriver::init();

        uint8_t cs_id;
        cs_id = setupSPI();

        tmc2240 = new TMC2240Stepper(cs_id, _r_ref, _spi_index);

        // use slower speed if I2S
        if (_cs_pin.capabilities().has(Pin::Capabilities::I2S)) {
            tmc2240->setSPISpeed(_spi_freq);
        }
        registration();
    }

    void TMC2240Driver::config_message() {
        log_info("    " << name() << " Step:" << _step_pin.name() << " Dir:" << _dir_pin.name() << " CS:" << _cs_pin.name()
                        << " Disable:" << _disable_pin.name() << " Index:" << _spi_index << " Rref:" << _r_ref << "k");
    }

    void TMC2240Driver::config_motor() {
        tmc2240->begin();
        TrinamicBase::config_motor();
    }

    bool TMC2240Driver::test() {
        return checkVersion(TMC2240Stepper::expected_version, tmc2240->version());
    }

    void TMC2240Driver::set_registers(bool isHoming) {
        if (_has_errors) {
            return;
        }

        _mode = static_cast<TrinamicMode>(trinamicModes[isHoming ? _homing_mode : _run_mode].value);

        tmc2240->slope_control(_slope_control);

        // Run and hold current configuration items are in (float) Amps, but the
        // TMC2240 current setter takes (uint16_t) mA RMS and expresses hold
        // current as a (float) fraction of run current.
        uint16_t run_i = (uint16_t)(_run_current * 1000.0);

        // An unreachable current would otherwise be clamped without a trace,
        // and the ceiling depends on the IREF resistor, which is easy to get wrong.
        uint16_t max_i = tmc2240->full_scale_rms_ma(3);
        if (run_i > max_i) {
            log_warn(axisName() << " run_amps " << _run_current << " is above the " << max_i / 1000.0
                                << "A limit for r_ref_kohms " << _r_ref);
        }

        tmc2240->rms_current(run_i, TrinamicSpiDriver::holdPercent());

        // The TMCStepper library uses the value 0 to mean 1x microstepping
        uint32_t usteps = _microsteps == 1 ? 0 : _microsteps;
        tmc2240->microsteps(usteps);

        tmc2240->tpfd(_tpfd);

        tmc2240->diag0_error(_diag0_error);
        tmc2240->diag0_otpw(_diag0_otpw);
        tmc2240->diag0_int_pushpull(_diag0_int_pushpull);

        switch (_mode) {
            case TrinamicMode ::StealthChop:
                log_debug(axisName() << " StealthChop");
                tmc2240->en_pwm_mode(true);
                tmc2240->pwm_autoscale(true);
                tmc2240->diag1_stall(false);
                break;
            case TrinamicMode ::CoolStep:
                log_debug(axisName() << " Coolstep");
                tmc2240->en_pwm_mode(false);
                tmc2240->pwm_autoscale(false);
                tmc2240->TCOOLTHRS(NORMAL_TCOOLTHRS);  // when to turn on coolstep
                tmc2240->THIGH(NORMAL_THIGH);
                break;
            case TrinamicMode ::StallGuard:
                log_debug(axisName() << " Stallguard");
                {
                    tmc2240->en_pwm_mode(false);
                    tmc2240->pwm_autoscale(false);
                    tmc2240->TCOOLTHRS(calc_tstep(150));
                    tmc2240->THIGH(calc_tstep(60));
                    tmc2240->sfilt(1);
                    tmc2240->diag1_stall(true);  // stallguard i/o is on diag1
                    tmc2240->sgt(constrain(_stallguard, -64, 63));
                    break;
                }
        }

        log_verbose("Run current: " << tmc2240->rms_current() << "mA in range " << tmc2240->current_range());
        log_verbose("CHOPCONF: " << to_hex(tmc2240->CHOPCONF()));
        log_verbose("COOLCONF: " << to_hex(tmc2240->COOLCONF()));
        log_verbose("THIGH: " << to_hex(tmc2240->THIGH()));
        log_verbose("TCOOLTHRS: " << to_hex(tmc2240->TCOOLTHRS()));
        log_verbose("GCONF: " << to_hex(tmc2240->GCONF()));
        log_verbose("PWMCONF: " << to_hex(tmc2240->PWMCONF()));
        log_verbose("IHOLD_IRUN: " << to_hex(tmc2240->IHOLD_IRUN()));
        log_verbose("DRV_CONF: " << to_hex(tmc2240->DRV_CONF()));
    }

    // Report diagnostic and tuning info
    void TMC2240Driver::debug_message() {
        if (_has_errors) {
            return;
        }

        uint32_t tstep = tmc2240->TSTEP();

        if (tstep == 0xFFFFF || tstep < 1) {  // if axis is not moving return
            return;
        }
        float feedrate = Stepper::get_realtime_rate();  //* settings.microsteps[axis_index] / 60.0 ; // convert mm/min to Hz

        log_info(axisName() << " Stallguard " << tmc2240->stallguard() << "   SG_Val:" << tmc2240->sg_result() << " Rate:" << feedrate
                            << " mm/min SG_Setting:" << constrain(_stallguard, -64, 63));
    }

    void TMC2240Driver::set_disable(bool disable) {
        if (TrinamicSpiDriver::startDisable(disable)) {
            if (_use_enable) {
                tmc2240->toff(TrinamicSpiDriver::toffValue());
            }
        }
    }

    // Configuration registration
    namespace {
        MotorFactory::InstanceBuilder<TMC2240Driver> registration("tmc_2240");
    }
}
