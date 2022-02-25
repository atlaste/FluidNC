#include "../TestFramework.h"

#include <src/Motion/Planner.h>

namespace Motion {

    Test(MotionTests, Planner) {
        Planner planner;
        Axes    axes;
        axes._numberAxis = 2;

        // Make two the same axis:
        auto& axis = axes._axis[0] = new Axis();
        axis->acceleration         = 10;
        axis->maxRate              = 10'000;
        axis->maxTravel            = 100'000;
        axis->stepsPerMm           = 100;
        axes._axis[1]              = axes._axis[0];

        // Position is (0). Let's make a little square:
        {
            Vector<float> target({ 100.0f, 0.0f });
            planner.Add(target, 1e38f, axes);
        }
        {
            Vector<float> target({ 100.0f, 100.0f });
            planner.Add(target, 1e38f, axes);
        }
        {
            Vector<float> target({ 0.0f, 100.0f });
            planner.Add(target, 1e38f, axes);
        }
        {
            Vector<float> target({ 0.0f, 0.0f });
            planner.Add(target, 1e38f, axes);
        }
    }
}
