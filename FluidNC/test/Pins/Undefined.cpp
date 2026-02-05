#include "TestFramework.h"

#include <Pin.h>

namespace Pins {
    Test(Undefined, Pins) {
        // Unassigned pins are not doing much...

        Pin unassigned;
        Assert(Pin() == unassigned, "Undefined has wrong pin id");

        {
            unassigned.write(true);
            auto result = unassigned.read();
            Assert(0 == result, "Result value incorrect");
        }

        {
            unassigned.write(false);
            auto result = unassigned.read();
            Assert(0 == result, "Result value incorrect");
        }

        Assert(unassigned.capabilities().has(Pin::Capabilities::Void));
        auto name = unassigned.name();
        Assert(name == "NO_PIN", "Expected name 'NO_PIN'");
    }

    Test(Undefined, MultipleInstances) {
        {
            Pin unassigned;
            Pin unassigned2;

            Assert(unassigned == unassigned2, "Should evaluate to true");
        }

        {
            Pin unassigned = Pin();
            Pin unassigned2;

            Assert(unassigned == unassigned2, "Should evaluate to true");
        }

        {
            Pin unassigned = Pin::create("void.2");
            Pin unassigned2;

            Assert(unassigned != unassigned2, "Void pin should not match undefined");
        }

        {
            Pin unassigned  = Pin::create("void.2");
            Pin unassigned2 = Pin::create("void.2");

            Assert(unassigned != unassigned2, "Second void pin should not match first");
        }
    }
}
