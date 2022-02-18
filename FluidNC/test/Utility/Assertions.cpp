#include "../TestFramework.h"

#include <src/EnumItem.h>
#include <src/StringStream.h>
#include <src/MyIOStream.h>

namespace Configuration {
    Test(UtilAssert, Basics) {
        {
            bool ok = true;
            try {
                Assert(false);
                ok = false;
            } catch (AssertionFailed& e) { Assert(e.msg.length() != 0); }
            Assert(ok);
        }

        {
            bool ok = true;
            try {
                Assert(false, "oops");
                ok = false;
            } catch (AssertionFailed& e) { Assert(e.msg.length() != 0); }
            Assert(ok);
        }
    }

}
