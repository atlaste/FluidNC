#include "LedStripStartupComplete.h"

namespace Extra {
    extern void (*g_startupCompleteCallback)();

    void LedStripStartupComplete::init() {
        // Signal that startup is complete
        if (g_startupCompleteCallback) {
            g_startupCompleteCallback();
        }
        log_info("LED startup sequence complete");
    }

    // Configuration registration
    namespace {
        ConfigurableModuleFactory::InstanceBuilder<Extra::LedStripStartupComplete> registration("led_startup_complete", true);
    }
}
