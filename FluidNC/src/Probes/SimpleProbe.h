#pragma once

#include "ProbeDriver.h"

namespace Probes {
    class SimpleProbe : public ProbeDriver {
    protected:
        Pin _probePin;
        bool _checkModeStart;

    public:
        const char* name() const override;

        void validate() const override;
        void group(Configuration::HandlerBase& handler) override;

        void init(TripProbe* callback);
        void start_cycle();
        void stop_cycle();
        bool is_tripped();

        ~SimpleProbe() override {}
    };
}
