#include "TestFramework.h"

#include <Pins/PinOptionsParser.h>
#include <cstdio>
#include <cstring>

namespace Pins {
    Test(PinOptionParsing, NoArgs) {
        Pins::PinOptionsParser parser("");

        {
            auto opt    = parser.begin();
            auto endopt = parser.end();
            Assert(opt == endopt, "Expected empty enumerator");
        }

        // Typical use is a for loop. Let's test the two ways to use it:
        for (auto it : parser) {
            Assert(false, "Didn't expect to get here");
        }

        for (auto it = parser.begin(); it != parser.end(); ++it) {
            Assert(false, "Didn't expect to get here");
        }
    }

    Test(PinOptionParsing, SingleArg) {
        Pins::PinOptionsParser parser("first");

        {
            auto opt    = parser.begin();
            auto endopt = parser.end();
            Assert(opt != endopt, "Expected an argument");
            Assert(opt->is("first"), "Expected 'first'");

            ++opt;
            Assert(opt == endopt, "Expected one argument");
        }

        // Typical use is a for loop. Let's test the two ways to use it:
        int ctr = 0;
        for (auto it : parser) {
            if (ctr == 0) {
                Assert(it.is("first"), "Expected 'first'");
            } else {
                Assert(false, "Didn't expect to get here");
            }
            ++ctr;
        }
    }

    Test(PinOptionParsing, SingleArgWithWS) {
        Pins::PinOptionsParser parser("  first");

        {
            auto opt    = parser.begin();
            auto endopt = parser.end();
            Assert(opt != endopt, "Expected an argument");
            Assert(opt->is("first"), "Expected 'first'");

            ++opt;
            Assert(opt == endopt, "Expected one argument");
        }

        // Typical use is a for loop. Let's test the two ways to use it:
        int ctr = 0;
        for (auto it : parser) {
            if (ctr == 0) {
                Assert(it.is("first"), "Expected 'first'");
            } else {
                Assert(false, "Didn't expect to get here");
            }
            ++ctr;
        }
    }

    Test(PinOptionParsing, SingleArgWithWS2) {
        Pins::PinOptionsParser parser("  first  ");

        {
            auto opt    = parser.begin();
            auto endopt = parser.end();
            Assert(opt != endopt, "Expected an argument");
            Assert(opt->is("first"), "Expected 'first'");

            ++opt;
            Assert(opt == endopt, "Expected one argument");
        }

        // Typical use is a for loop. Let's test the two ways to use it:
        int ctr = 0;
        for (auto it : parser) {
            if (ctr == 0) {
                Assert(it.is("first"), "Expected 'first'");
            } else {
                Assert(false, "Didn't expect to get here");
            }
            ++ctr;
        }
    }

    Test(PinOptionParsing, TwoArg1) {
        Pins::PinOptionsParser parser("first;second");

        {
            auto opt    = parser.begin();
            auto endopt = parser.end();
            Assert(opt != endopt, "Expected an argument");
            Assert(opt->is("first"), "Expected 'first'");

            ++opt;
            Assert(opt != endopt, "Expected second argument");
            Assert(opt->is("second"), "Expected 'second'");

            ++opt;
            Assert(opt == endopt, "Expected two arguments");
        }

        // Typical use is a for loop. Let's test the two ways to use it:
        int ctr = 0;
        for (auto it : parser) {
            if (ctr == 0) {
                Assert(it.is("first"), "Expected 'first'");
            } else if (ctr == 1) {
                Assert(it.is("second"), "Expected 'second'");
            } else {
                Assert(false, "Didn't expect to get here");
            }
            ++ctr;
        }
    }

    Test(PinOptionParsing, TwoArg2) {
        Pins::PinOptionsParser parser("first:second");

        {
            auto opt    = parser.begin();
            auto endopt = parser.end();
            Assert(opt != endopt, "Expected an argument");
            Assert(opt->is("first"), "Expected 'first'");

            ++opt;
            Assert(opt != endopt, "Expected second argument");
            Assert(opt->is("second"), "Expected 'second'");

            ++opt;
            Assert(opt == endopt, "Expected two arguments");
        }

        // Typical use is a for loop. Let's test the two ways to use it:
        int ctr = 0;
        for (auto it : parser) {
            if (ctr == 0) {
                Assert(it.is("first"), "Expected 'first'");
            } else if (ctr == 1) {
                Assert(it.is("second"), "Expected 'second'");
            } else {
                Assert(false, "Didn't expect to get here");
            }
            ++ctr;
        }
    }

    Test(PinOptionParsing, TwoArgWithValues) {
        Pins::PinOptionsParser parser("first=12;second=13");

        {
            auto opt    = parser.begin();
            auto endopt = parser.end();
            Assert(opt != endopt, "Expected an argument");
            Assert(opt->is("first"), "Expected 'first'");
            auto val1 = (*opt).value();
            Assert(val1 == "12", "Expected value '12'");
            Assert(12 == opt->iValue(), "Expected iValue 12");

            ++opt;
            Assert(opt != endopt, "Expected second argument");
            Assert(opt->is("second"), "Expected 'second'");
            auto val2 = (*opt).value();
            Assert(val2 == "13", "Expected value '13'");
            Assert(13 == opt->iValue(), "Expected iValue 13");

            ++opt;
            Assert(opt == endopt, "Expected two arguments");
        }

        // Typical use is a for loop. Let's test the two ways to use it:
        int ctr = 0;
        for (auto it : parser) {
            if (ctr == 0) {
                Assert(it.is("first"), "Expected 'first'");
            } else if (ctr == 1) {
                Assert(it.is("second"), "Expected 'second'");
            } else {
                Assert(false, "Didn't expect to get here");
            }
            ++ctr;
        }
    }
}
