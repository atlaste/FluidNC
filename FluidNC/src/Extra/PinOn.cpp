#include "PinOn.h"

namespace Extra {
    PinOn::PinOn(const char* name) : ConfigurableModule(name) {}

    void PinOn::init() {
        if (pin1_.defined()) {
            log_info("Setting pin " << pin1_.name() << " to " << int(pinValue1_));

            pin1_.setAttr(Pin::Attr::Output);
            pin1_.write(pinValue1_ != 0);
        }
        if (pin2_.defined()) {
            log_info("Setting pin " << pin2_.name() << " to " << int(pinValue2_));

            pin2_.setAttr(Pin::Attr::Output);
            pin2_.write(pinValue2_ != 0);
        }
        if (pin3_.defined()) {
            log_info("Setting pin " << pin3_.name() << " to " << int(pinValue3_));

            pin3_.setAttr(Pin::Attr::Output);
            pin3_.write(pinValue3_ != 0);
        }
        if (pin4_.defined()) {
            log_info("Setting pin " << pin4_.name() << " to " << int(pinValue4_));

            pin4_.setAttr(Pin::Attr::Output);
            pin4_.write(pinValue4_ != 0);
        }
        if (pin5_.defined()) {
            log_info("Setting pin " << pin5_.name() << " to " << int(pinValue5_));

            pin5_.setAttr(Pin::Attr::Output);
            pin5_.write(pinValue5_ != 0);
        }
    }

    // Configuration registration
    namespace {
        ConfigurableModuleFactory::InstanceBuilder<PinOn> registration("pin_on");
    }

}
