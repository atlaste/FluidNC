#pragma once

#include "Module.h"
#include "Pin.h"

#include <driver/pulse_cnt.h>

class SpindleEncoder : public ConfigurableModule {
    Pin pin_a;
    Pin pin_b;
    float ratio = 1.0f;

    int64_t            totalCount = 0;
    pcnt_unit_handle_t pcnt_total = NULL;


public:
    SpindleEncoder() : ConfigurableModule("spindle_encoder") {}

    void init() override;
    void deinit() override;

    void group(Configuration::HandlerBase& handler) override {
        handler.item("pin_a", pin_a);
        handler.item("pin_b", pin_b);
        handler.item("ratio", ratio);
    }

    virtual ~SpindleEncoder() {}
};
