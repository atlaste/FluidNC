#include "../TestFramework.h"

#include <src/Pins/PinAttributes.h>

#include <cstdio>
#include <cstring>

namespace Pins {
    Test(PinAttributesBasics, Conflicts) {
        {
            PinAttributes input = PinAttributes::Input;

            Assert(input.conflictsWith(PinAttributes::Output), "Input and output don't mix");
            Assert(input.conflictsWith(PinAttributes::ActiveLow));
            Assert(!input.conflictsWith(PinAttributes::Exclusive));
            Assert(input.conflictsWith(PinAttributes::InitialOn));
            Assert(!input.conflictsWith(PinAttributes::Input));
            Assert(!input.conflictsWith(PinAttributes::ISR));
            Assert(!input.conflictsWith(PinAttributes::PullUp));
            Assert(!input.conflictsWith(PinAttributes::PullDown));
        }

        {
            PinAttributes output = PinAttributes::Output;

            Assert(output.conflictsWith(PinAttributes::Input), "Input and output don't mix");
            Assert(!output.conflictsWith(PinAttributes::ActiveLow));
            Assert(!output.conflictsWith(PinAttributes::Exclusive));
            Assert(!output.conflictsWith(PinAttributes::InitialOn));
            Assert(!output.conflictsWith(PinAttributes::Output));
            Assert(output.conflictsWith(PinAttributes::ISR));
            Assert(output.conflictsWith(PinAttributes::PullUp));
            Assert(output.conflictsWith(PinAttributes::PullDown));
        }

        {
            PinAttributes pu = PinAttributes::Input | PinAttributes::PullUp;

            Assert(!pu.conflictsWith(PinAttributes::PullUp));
            Assert(pu.conflictsWith(PinAttributes::PullDown));
        }

        {
            PinAttributes pd = PinAttributes::Input | PinAttributes::PullDown;

            Assert(pd.conflictsWith(PinAttributes::PullUp));
            Assert(!pd.conflictsWith(PinAttributes::PullDown));
        }

        {
            Assert(PinAttributes::Output.conflictsWith(PinAttributes::ISR));
            Assert(!PinAttributes::Input.conflictsWith(PinAttributes::ISR));
        }

        {
            PinAttributes excl = PinAttributes::Input | PinAttributes::Exclusive;

            Assert(excl.conflictsWith(PinAttributes::ActiveLow));
            Assert(excl.conflictsWith(PinAttributes::Exclusive));
            Assert(excl.conflictsWith(PinAttributes::InitialOn));
            Assert(excl.conflictsWith(PinAttributes::Input));
            Assert(excl.conflictsWith(PinAttributes::ISR));
            Assert(excl.conflictsWith(PinAttributes::PullUp));
            Assert(excl.conflictsWith(PinAttributes::PullDown));
        }
    }

    Test(PinAttributesBasics, MaskingBitOp) {
        PinAttributes a = PinAttributes::None;

        Assert(!bool(a));

        a = a | PinAttributes::Input;
        a = a | PinAttributes::ISR;
        a = a | PinAttributes::PullUp;

        Assert(a.has(PinAttributes::Input));
        Assert(a.has(PinAttributes::ISR));
        Assert(a.has(PinAttributes::PullUp));
        Assert(bool(a));

        auto b = a;

        Assert(a == b);

        a = a & (PinAttributes::Input | PinAttributes::ISR);

        Assert(a.has(PinAttributes::Input));
        Assert(a.has(PinAttributes::ISR));
        Assert(!a.has(PinAttributes::PullUp));
        Assert(a != b);
        Assert(bool(a));
    }

    Test(PinCapabilitiesBasics, Operators) {
        {
            PinCapabilities a = PinCapabilities::Input;
            PinCapabilities b = PinCapabilities::Output;
            PinCapabilities c = PinCapabilities::Input;

            Assert(a.has(PinCapabilities::Input));
            Assert(a == c);
            Assert(b != a);
            Assert(b != c);

            PinCapabilities d = a | PinCapabilities::ISR;
            Assert(d != a);
            Assert(d != b);
            Assert(d != c);

            Assert(!(d == a));
            Assert(!(d == b));
            Assert(!(d == c));

            PinCapabilities e = d & a;
            Assert(e == c);
        }
    }
}
