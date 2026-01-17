#pragma once

#include "Module.h"

namespace Extra {
    // Simple module that signals LED startup is complete
    // Initialize this with a very high priority so it runs after everything else
    class LedStripStartupComplete : public ConfigurableModule {
    public:
        LedStripStartupComplete(const char* name) : ConfigurableModule(name) {}

        void init() override;
        int  init_priority() override { return 1'000'000; }  // Very high priority - runs last
        void deinit() override {}

        // No configuration needed
        void group(Configuration::HandlerBase& handler) override {}

        ~LedStripStartupComplete() = default;
    };
}
