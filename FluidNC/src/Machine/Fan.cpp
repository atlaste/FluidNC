// Copyright (c) 2026 - Stefan de Bruijn
// Use of this source code is governed by a GPLv3 license that can be found in the LICENSE file.

#include "Fan.h"

#include "Config.h"  // log_*
#include "Job.h"     // Job::active()
#include "State.h"   // state_is()

namespace Machine {
    void Fan::init() {
        if (_pwmPin.undefined()) {
            return;
        }
        if (!_pwmPin.capabilities().has(Pin::Capabilities::PWM)) {
            log_error("Fan pwm pin " << _pwmPin.name() << " cannot do PWM");
            return;
        }

        _pwmPin.setAttr(Pin::Attr::PWM, _pwmHz);
        _active = true;
        setPercent(_idlePercent);

        log_info("Fan on Pin:" << _pwmPin.name() << " Freq:" << _pwmHz << "Hz Idle:" << _idlePercent << "% Job:" << _jobPercent << "%");
    }

    void Fan::poll() {
        if (!_active) {
            return;
        }
        // A job streamed over a channel shows up only as Cycle, while a job read
        // from a file also makes Job::active() true, which unlike Cycle stays true
        // across the gaps between moves.  Jogging and homing count as idle.
        setPercent((Job::active() || state_is(State::Cycle)) ? _jobPercent : _idlePercent);
    }

    void Fan::setPercent(float percent) {
        if (!_active || percent == _currentPercent) {
            return;
        }
        _currentPercent = percent;

        // The 0.5 rounds to the nearest duty unit
        _pwmPin.setDuty(uint32_t(((percent * _pwmPin.maxDuty()) / 100.0f) + 0.5f));
    }

    void Fan::group(Configuration::HandlerBase& handler) {
        handler.item("pwm_pin", _pwmPin);
        handler.item("pwm_hz", _pwmHz, 1, 20000000);
        handler.item("idle_percent", _idlePercent, 0.0f, 100.0f);
        handler.item("job_percent", _jobPercent, 0.0f, 100.0f);
    }

    namespace {
        ConfigurableModuleFactory::InstanceBuilder<Fan> registration("fan");
    }
}
