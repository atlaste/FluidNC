// Copyright (c) 2020 -	Bart Dring
// Use of this source code is governed by a GPLv3 license that can be found in the LICENSE file.

#pragma once

#include <cstdint>

#include "SpindleDatatypes.h"
#include "Machine/Macros.h"
#include "Configuration/Configurable.h"
#include "Configuration/GenericFactory.h"
#include "GCode.h"  // MaxToolNumber
#include "Module.h"
#include "ToolChangers/atc.h"

// ===============  No floats! ===========================
// ================ NO FLOATS! ==========================

namespace Spindles {
    // ISR-safe callback for setting spindle speed from stepper ISR.
    // Using a static callback avoids vtable lookups which can fail in IRAM
    // because the vtable might be in flash memory on ESP32.
    using SetSpeedCallback = void (*)(uint32_t dev_speed, void* userData);

    // Struct to hold the callback and its associated userData (typically 'this' pointer)
    struct SpeedCallbackInfo {
        SetSpeedCallback callback;
        void*            userData;
    };
    class Spindle;
    using SpindleList = std::vector<Spindle*>;

    // This is the base class. Do not use this as your spindle
    class Spindle : public Configuration::Configurable {
    private:
        const char* _name;
        std::string _atc_info = "";

        // _zero_speed_with_disable forces speed to 0 when disabled
        bool _zero_speed_with_disable = false;

    protected:
        ATCs::ATC* _atc       = nullptr;
        uint32_t   _last_tool = 0;

        void startRamp(uint32_t millis);
        void endRamp();

    public:
        // _disable_with_zero_speed forces a disable when speed is 0
        bool             _disable_with_zero_speed = false;
        // Time (in microseconds from esp_timer_get_time) after which speed is considered valid.
        // 0 means speed is currently valid. Uses esp_timer for cross-core consistency.
        volatile int64_t _speedIsValidAfter       = 0;
        bool             speedIsValid();

        Spindle(const char* name) : _name(name) {}

        Spindle(const Spindle&)            = delete;
        Spindle(Spindle&&)                 = delete;
        Spindle& operator=(const Spindle&) = delete;
        Spindle& operator=(Spindle&&)      = delete;

        bool     _defaultedSpeeds;
        uint32_t offSpeed() { return _speeds[0].offset; }
        uint32_t maxSpeed();
        uint32_t mapSpeed(SpindleState state, SpindleSpeed speed);
        void     setupSpeeds(uint32_t max_dev_speed);
        void     shelfSpeeds(SpindleSpeed min, SpindleSpeed max);
        void     linearSpeeds(SpindleSpeed maxSpeed, float maxPercent);

        static void switchSpindle(uint32_t new_tool, SpindleList spindles, Spindle*& spindle, bool& stop_spindle, bool& new_spindle);

        void         spindleDelay(SpindleState state, SpindleSpeed speed);
        virtual void init() = 0;  // not in constructor because this also gets called when $$ settings change
        virtual void init_atc();
        std::string  atc_info() { return _atc_info; };

        void save_atc_data(std::vector<uint8_t>& buffer) {
            if (_atc) {
                _atc->save_atc_data(buffer);
            }
        }
        void restore_atc_data(const std::vector<uint8_t>& buffer, size_t& index) {
            if (_atc) {
                _atc->restore_atc_data(buffer, index);
            }
        }

        // Used by Protocol.cpp to restore the state during a restart
        virtual void   setState(SpindleState state, uint32_t speed) = 0;
        SpindleState   get_state() { return _current_state; };
        void           stop() { setState(SpindleState::Disable, 0); }
        virtual void   config_message() = 0;
        virtual bool   isRateAdjusted();
        virtual bool   use_delay_settings() const { return true; }
        virtual tool_t get_current_tool_num() { return _current_tool; }
        virtual bool   tool_change(uint32_t tool_number, bool pre_select, bool set_tool);

        virtual void setSpeedfromISR(uint32_t dev_speed) = 0;

        // ISR-safe speed callback mechanism.
        // Returns a callback function and userData that can be called from ISR context
        // without vtable lookups. Derived classes that support real-time speed changes
        // should override this to return their ISR-safe implementation.
        // The default implementation returns a no-op callback.
        virtual SpeedCallbackInfo getISRSpeedCallback() { return { defaultSpeedCallback, nullptr }; }

        // Default no-op callback for spindles that don't support ISR speed changes
        static void IRAM_ATTR defaultSpeedCallback(uint32_t dev_speed, void* userData) {
            // No-op: spindles that don't support ISR speed changes will use this
        }

        void spinDown() { setState(SpindleState::Disable, 0); }

        bool                  is_reversable;
        volatile SpindleState _current_state = SpindleState::Unknown;
        volatile SpindleSpeed _current_speed = 0;

        // scaler units are ms/rpm * 2^16.
        // The computation is deltaRPM * scaler >> 16
        uint32_t _spinup_ms   = 0;
        uint32_t _spindown_ms = 0;

        int32_t _tool = -1;

        std::vector<Configuration::speedEntry> _speeds;

        bool _off_on_alarm = false;

        Macro       _m6_macro;
        std::string _atc_name = "";

        // Name is required for the configuration factory to work.
        const char* name() { return _name; }

        // Configuration handlers:
        void validate() override;
        void afterParse() override;

        void group(Configuration::HandlerBase& handler) override {
            if (use_delay_settings()) {
                handler.item("spinup_ms", _spinup_ms, 0, 60000);
                handler.item("spindown_ms", _spindown_ms, 0, 60000);
            }
            handler.item("tool_num", _tool, 0, MaxToolNumber);
            handler.item("speed_map", _speeds);
            handler.item("off_on_alarm", _off_on_alarm);
            handler.item("atc", _atc_name);
            handler.item("m6_macro", _m6_macro);
            handler.item("s0_with_disable", _zero_speed_with_disable);
            handler.item("disable_with_s0", _disable_with_zero_speed);
        }

        // Virtual base classes require a virtual destructor.
        virtual ~Spindle() {}

    protected:
        tool_t _current_tool = 0;
    };

    using SpindleFactory = Configuration::GenericFactory<Spindle>;
}
extern Spindles::Spindle* spindle;
