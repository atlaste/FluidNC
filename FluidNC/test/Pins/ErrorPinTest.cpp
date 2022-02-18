#include "../TestFramework.h"

#include <src/Pin.h>
#include <esp32-hal-gpio.h>  // CHANGE

namespace Pins {
    Test(Error, Pins) {
        // Error pins should throw whenever they are used.

        Pin errorPin = Pin::Error();

        {
            std::ostringstream oss;
            auto               oldbuf = std::cout.rdbuf();
            std::cout.set_rdbuf(oss.rdbuf());
            errorPin.write(true);
            std::cout.set_rdbuf(oldbuf);
            Assert(oss.str().size() != 0, "Expected error written to output");
        }

        {
            std::ostringstream oss;
            auto               oldbuf = std::cout.rdbuf();
            std::cout.set_rdbuf(oss.rdbuf());
            errorPin.read();
            std::cout.set_rdbuf(oldbuf);
            Assert(oss.str().size() != 0, "Expected error written to output");
        }

        errorPin.setAttr(Pin::Attr::None);

        {
            std::ostringstream oss;
            auto               oldbuf = std::cout.rdbuf();
            std::cout.set_rdbuf(oss.rdbuf());
            errorPin.write(true);
            std::cout.set_rdbuf(oldbuf);
            Assert(oss.str().size() != 0, "Expected error written to output");
        }

        {
            std::ostringstream oss;
            auto               oldbuf = std::cout.rdbuf();
            std::cout.set_rdbuf(oss.rdbuf());
            errorPin.read();
            std::cout.set_rdbuf(oldbuf);
            Assert(oss.str().size() != 0, "Expected error written to output");
        }

        AssertThrow(errorPin.attachInterrupt([](void* arg) {}, CHANGE));
        AssertThrow(errorPin.detachInterrupt());

        Assert(errorPin.capabilities() == Pin::Capabilities::Error, "Incorrect caps");
    }
}
