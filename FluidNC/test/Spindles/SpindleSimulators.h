// Use of this source code is governed by a GPLv3 license that can be found in the LICENSE file.
// Concrete spindle simulators for unit tests.

#pragma once

#include "Spindles/SpindleTestBase.h"
#include "Spindles/NullSpindle.h"

namespace SpindleTest {

    /** Null spindle: no I/O, setState/stop/mapSpeed/off_on_alarm behavior. */
    class NullSpindleSimulator : public ISpindleSimulator {
    public:
        NullSpindleSimulator() : _spindle("NoSpindle") {}
        void               Setup() override {}
        Spindles::Spindle* GetSpindle() override { return &_spindle; }
        bool               HasPhysicalOutput() const override { return false; }

    private:
        Spindles::Null _spindle;
    };

}  // namespace SpindleTest
