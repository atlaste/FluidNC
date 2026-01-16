#pragma once

#include "Configuration/HandlerBase.h"
#include "Module.h"
#include "Pin.h"

#include "driver/rmt_tx.h"
#include "esp_err.h"

#include <stdint.h>
#include <string>
#include <vector>

namespace Extra {
    class LedStripFeedback;  // Forward declaration
    // Supported LED strip types
    enum class LedStripType {
        WS2812,       // 5V, most common
        WS2812B,      // 5V, improved WS2812
        WS2811_FAST,  // 5V/12V/24V, WS2812-compatible timing_
        WS2811_SLOW,  // 5V/12V/24V, original WS2811 timing_
        SK6812,       // 5V, RGB variant
        SK6812_RGBW,  // 5V, with white channel
        WS2813,       // 5V, data backup (same timing_ as WS2812)
        WS2815,       // 12V, data backup (same timing_ as WS2812)
    };

    // Color channel order
    enum class LedStripColorOrder {
        UseDefault,
        RGB,   // Red, Green, Blue
        GRB,   // Green, Red, Blue (most common)
        BGR,   // Blue, Green, Red
        GRBW,  // Green, Red, Blue, White (SK6812 RGBW)
    };

    // timing_ configuration (in 0.1us units at 10MHz resolution)
    struct LedStripTiming {
        uint32_t t0h   = 0;  // Time for 0 bit high
        uint32_t t0l   = 0;  // Time for 0 bit low
        uint32_t t1h   = 0;  // Time for 1 bit high
        uint32_t t1l   = 0;  // Time for 1 bit low
        uint32_t reset = 0;  // Reset time (low)
    };

    struct LedPosition {
        float x, y, z;
    };

    class LedStripRMT : public ConfigurableModule {
        uint32_t rmtResolutionHz_ = 10000000;  // 10MHz

        void*   ledStripHandle_ = nullptr;  // PIMPL'ed.
        int32_t numberLeds_     = 0;

        LedStripFeedback* feedback_ = nullptr;

        // Position mapping
        std::vector<LedPosition> ledPositions_;  // Calculated XYZ position of each LED
        void                     calculateLedPositions();

    public:
        LedStripRMT(const char* name) : ConfigurableModule(name) {}

        friend class LedStripFeedback;

        Pin                  ledPin_;
        LedStripType         type_       = LedStripType::WS2812;
        LedStripColorOrder   colorOrder_ = LedStripColorOrder::UseDefault;
        std::vector<int32_t> leds_;  // Physical LED indices to use (e.g., [10,11,12...])
        std::vector<int32_t> travel_;
        std::string          direction_;
        uint8_t              bytesPerLed_ = 0;  // 3 for RGB, 4 for RGBW
        LedStripTiming       timing_;

        void setPixel(uint16_t index, uint8_t r, uint8_t g, uint8_t b);
        void refresh();
        void clear();

        // Position mapping API
        const std::vector<LedPosition>& getLedPositions() const { return ledPositions_; }
        bool                            hasPositionMapping() const { return !ledPositions_.empty(); }

        void group(Configuration::HandlerBase& handler) override;
        void afterParse() override;

        void init() override;
        int  init_priority() override { return 0; }
        void deinit() override;

        ~LedStripRMT() = default;
    };
}
