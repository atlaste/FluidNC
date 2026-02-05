#include "TestFramework.h"

#include <Scheduler/Timepoint.h>
#include <Scheduler/Timespan.h>

namespace Scheduler {

    Test(Timepoint, DefaultConstruction) {
        Timepoint t;
        Assert(t() == 0, "default timepoint is 0");
    }

    Test(Timepoint, FromInt64) {
        Timepoint t(12345);
        Assert(t() == 12345, "timepoint from int64");
    }

    Test(Timepoint, AddTimespan) {
        Timepoint t(1000);
        Timespan  d(500);
        Timepoint u = t + d;
        Assert(u() == 1500, "t + d");
        t += d;
        Assert(t() == 1500, "t += d");
    }

    Test(Timepoint, SubtractTimespan) {
        Timepoint t(1000);
        Timespan  d(300);
        Timepoint u = t - d;
        Assert(u() == 700, "t - d");
        t -= d;
        Assert(t() == 700, "t -= d");
    }

    Test(Timepoint, Comparisons) {
        Timepoint a(100), b(200), c(100);
        Assert(a < b, "a < b");
        Assert(a <= b, "a <= b");
        Assert(a <= c, "a <= c");
        Assert(b > a, "b > a");
        Assert(b >= a, "b >= a");
        Assert(a >= c, "a >= c");
        Assert(a == c, "a == c");
        Assert(a != b, "a != b");
    }

    Test(Timespan, DefaultConstruction) {
        Timespan ts;
        Assert(ts() == 0, "default timespan is 0");
    }

    Test(Timespan, FromInt64) {
        Timespan ts(1000000);
        Assert(ts() == 1000000, "timespan from int64");
    }

    Test(Timespan, Literals) {
        Assert(1_msec() == 1000, "1_msec");
        Assert(1_sec() == 1000000, "1_sec");
        Assert(100_usec() == 100, "100_usec");
    }

    Test(Timespan, StaticFactories) {
        Assert(Timespan::FromMicroseconds(1)() == 1, "FromMicroseconds");
        Assert(Timespan::FromMilliseconds(1)() == 1000, "FromMilliseconds");
        Assert(Timespan::FromSeconds(1)() == 1000000, "FromSeconds");
    }
}
