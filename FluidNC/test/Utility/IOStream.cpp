#include "../TestFramework.h"

#include <src/EnumItem.h>
#include <src/StringStream.h>
#include <src/MyIOStream.h>

namespace Configuration {
    Test(UtilIOStream, Strings) {
        // const char*
        {
            StringStream ss;
            ss << "aap";
            Assert(ss.str() == "aap");
        }
        {
            StringStream ss;
            ss << "aap"
               << "banaan";
            Assert(ss.str() == "aapbanaan");
        }
        {
            StringStream ss;
            ss << "aap";
            ss << "banaan";
            Assert(ss.str() == "aapbanaan");
        }

        // String
        {
            StringStream ss;
            ss << String("aap");
            Assert(ss.str() == "aap");
        }
        {
            StringStream ss;
            ss << String("aap") << String("banaan");
            Assert(ss.str() == "aapbanaan");
        }
        {
            StringStream ss;
            ss << String("aap");
            ss << String("banaan");
            Assert(ss.str() == "aapbanaan");
        }
    }

    Test(UtilIOStream, Characters) {
        {
            StringStream ss;
            ss << 'a' << 'a' << 'p';
            Assert(ss.str() == "aap");
        }
    }

    Test(UtilIOStream, Integers) {  // int
        {
            StringStream ss;
            ss << 123;
            Assert(ss.str() == "123");
        }
        {
            StringStream ss;
            ss << 123 << "." << 456;
            Assert(ss.str() == "123.456");
        }
        {
            StringStream ss;
            ss << 0;
            Assert(ss.str() == "0");
        }
        {
            StringStream ss;
            ss << -123;
            Assert(ss.str() == "-123");
        }
        {
            StringStream ss;
            ss << -2000000000;
            Assert(ss.str() == "-2000000000");
        }
        {
            StringStream ss;
            ss << 2000000000;
            Assert(ss.str() == "2000000000");
        }
    }
    Test(UtilIOStream, UnsignedIntegers) {
        // unsigned int
        {
            StringStream ss;
            ss << 123u;
            Assert(ss.str() == "123");
        }
        {
            StringStream ss;
            ss << 123u << "." << 456u;
            Assert(ss.str() == "123.456");
        }
        {
            StringStream ss;
            ss << 0u;
            Assert(ss.str() == "0");
        }
        {
            StringStream ss;
            ss << 3000000000u;
            Assert(ss.str() == "3000000000");
        }

        // uint64_t
        {
            StringStream ss;
            ss << 123ull;
            Assert(ss.str() == "123");
        }
        {
            StringStream ss;
            ss << 123ull << "." << 456ull;
            Assert(ss.str() == "123.456");
        }
        {
            StringStream ss;
            ss << 0ull;
            Assert(ss.str() == "0");
        }
        {
            StringStream ss;
            ss << 1234567890123456ull;
            Assert(ss.str() == "1234567890123456");
        }
    }

    Test(UtilIOStream, FloatingPoint) {
        // double
        {
            StringStream ss;
            ss << -123.456;
            Assert(ss.str() == "-123.456");
        }
        {
            StringStream ss;
            ss << 123.0;
            Assert(ss.str() == "123.000");
        }
        {
            StringStream ss;
            ss << 123.4;
            Assert(ss.str() == "123.400");
        }
        {
            StringStream ss;
            ss << 123.4111;
            Assert(ss.str() == "123.411");
        }
        {
            StringStream ss;
            ss << 123.4119;
            Assert(ss.str() == "123.412");
        }

        // float
        {
            StringStream ss;
            ss << -123.456f;
            Assert(ss.str() == "-123.456");
        }
        {
            StringStream ss;
            ss << 123.0f;
            Assert(ss.str() == "123.000");
        }
        {
            StringStream ss;
            ss << 123.4f;
            Assert(ss.str() == "123.400");
        }
        {
            StringStream ss;
            ss << 123.4111f;
            Assert(ss.str() == "123.411");
        }
        {
            StringStream ss;
            ss << 123.4119f;
            Assert(ss.str() == "123.412");
        }
    }

    Test(UtilIOStream, FloatingPointPrecision) {
        // double
        {
            StringStream ss;
            ss << setprecision(4) << -123.456 << -123.456;
            Assert(ss.str() == "-123.4560-123.456");
        }
        {
            StringStream ss;
            ss << setprecision(4) << 123.0 << 123.0;
            Assert(ss.str() == "123.0000123.000");
        }
        {
            StringStream ss;
            ss << setprecision(4) << 123.4 << 123.4;
            Assert(ss.str() == "123.4000123.400");
        }
        {
            StringStream ss;
            ss << setprecision(4) << 123.4111 << 123.4111;
            Assert(ss.str() == "123.4111123.411");
        }
        {
            StringStream ss;
            ss << setprecision(4) << 123.4119 << 123.4119;
            Assert(ss.str() == "123.4119123.412");
        }

        // float
        {
            StringStream ss;
            ss << setprecision(4) << -123.456f << -123.456f;
            Assert(ss.str() == "-123.4560-123.456");
        }
        {
            StringStream ss;
            ss << setprecision(4) << 123.0f << 123.0f;
            Assert(ss.str() == "123.0000123.000");
        }
        {
            StringStream ss;
            ss << setprecision(4) << 123.4f << 123.4f;
            Assert(ss.str() == "123.4000123.400");
        }
        {
            StringStream ss;
            ss << setprecision(4) << 123.4111f << 123.4111f;
            Assert(ss.str() == "123.4111123.411");
        }
        {
            StringStream ss;
            ss << setprecision(4) << 123.4119f << 123.4119f;
            Assert(ss.str() == "123.4119123.412");
        }
    }

    Test(UtilIOStream, Pins) {
        {
            StringStream ss;
            Pin          p = Pin::create("gpio.12:pu");
            ss << p;
            Assert(ss.str() == "gpio.12:pu");
        }
    }
}
