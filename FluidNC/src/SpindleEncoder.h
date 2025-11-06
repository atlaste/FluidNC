#pragma once

#include "Module.h"
#include "Pin.h"
#include "Spindles/Spindle.h"

#include <driver/pulse_cnt.h>

class SpindleEncoder : public ConfigurableModule {
    Pin   pin_a;
    Pin   pin_b;
    int32_t ratio; // in 1/10000

    volatile int64_t   totalCount = 0;
    pcnt_unit_handle_t pcnt_total = NULL;
    pcnt_unit_handle_t pcnt_alm   = NULL;

    static bool pcnt_on_overflow(pcnt_unit_handle_t unit, const pcnt_watch_event_data_t* edata, void* user_ctx);
    static bool pcnt_on_reach(pcnt_unit_handle_t unit, const pcnt_watch_event_data_t* edata, void* user_ctx);

    using SetpointReachedCallback = void (*)(void*);

    int32_t tolerance = 10; // 10% RPM tolerance. If we go out of this range, it's an alarm.
    int64_t lastCount = 0;
    int32_t countPerRevolution = 800; // encoder CPR

public:
    SpindleEncoder(const char* name) : ConfigurableModule(name) {}

    void init() override;
    void deinit() override;

    int64_t getCount();

    void resetCount() { lastCount  = getCount(); }
    
    // ISR-safe validation - returns true if out of tolerance, false if OK
    bool validateSpeed(int32_t usecs);

    uint32_t lastEncoderSpeed_ = 0;

    void group(Configuration::HandlerBase& handler) override {
        handler.item("pin_a", pin_a);
        handler.item("pin_b", pin_b);
        handler.item("tolerance", tolerance);
        handler.item("cpr", countPerRevolution);
        float r = float(ratio / 10000.0);
        handler.item("gear_ratio", r);
        ratio = int(r * 10000.0);
    }

    virtual ~SpindleEncoder() {}
};

extern SpindleEncoder* spindle_encoder;
