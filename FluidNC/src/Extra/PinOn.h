#pragma once

#include "Configuration/HandlerBase.h"
#include "Module.h"
#include "Pin.h"

namespace Extra
{
    class PinOn : public ConfigurableModule
    {
        Pin pin1_;
        int32_t pinValue1_;

        Pin pin2_;
        int32_t pinValue2_;

        Pin pin3_;
        int32_t pinValue3_;

        Pin pin4_;
        int32_t pinValue4_;

        Pin pin5_;
        int32_t pinValue5_;

    public:
        PinOn(const char* name);

        void init() override;

        void group(Configuration::HandlerBase& handler) override
        {
            handler.item("pin1", pin1_);
            handler.item("value1", pinValue1_);

            handler.item("pin2", pin2_);
            handler.item("value2", pinValue2_);

            handler.item("pin3", pin3_);
            handler.item("value3", pinValue3_);

            handler.item("pin4", pin4_);
            handler.item("value4", pinValue4_);

            handler.item("pin5", pin5_);
            handler.item("value5", pinValue5_);
        }

        ~PinOn() {}
    };
}
