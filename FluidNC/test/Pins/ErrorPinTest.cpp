#include "TestFramework.h"

#include <Pin.h>

namespace Pins {
    // Tests that error pins throw exceptions when read/write operations are attempted.
    // Error pins are used for invalid configurations and should alert the user
    // immediately when accessed.
    Test(Error, Pins) {
        // Error pins should throw whenever they are used.

        Pin errorPin = Pin::Error();

        // Should throw on write
        AssertThrow(errorPin.write(true));
        
        // Should throw on read  
        AssertThrow(errorPin.read());

        // Setting attributes shouldn't throw (needed to configure before use)
        errorPin.setAttr(Pin::Attr::None);

        // Should still throw after setAttr
        AssertThrow(errorPin.write(true));
        AssertThrow(errorPin.read());

        // Should report Error capability
        Assert(errorPin.capabilities() == Pin::Capabilities::Error, "Incorrect caps");
    }
}
