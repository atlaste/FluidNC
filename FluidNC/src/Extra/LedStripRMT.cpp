#include "LedStripRMT.h"
#include "LedStripFeedback.h"

#include <esp_err.h>
#include <memory>

namespace Extra {
    namespace {
        EnumItem LedStripTypeDescr[] = {
            { uint32_t(LedStripType::WS2812), "WS2812" },            // 5V, most common
            { uint32_t(LedStripType::WS2812B), "WS2812B" },          // 5V, improved WS2812
            { uint32_t(LedStripType::WS2811_FAST), "WS2811_FAST" },  // 5V/12V/24V, WS2812-compatible timing
            { uint32_t(LedStripType::WS2811_SLOW), "WS2811_SLOW" },  // 5V/12V/24V, original WS2811 timing
            { uint32_t(LedStripType::SK6812), "SK6812" },            // 5V, RGB variant
            { uint32_t(LedStripType::SK6812_RGBW), "SK6812_RGBW" },  // 5V, with white channel
            { uint32_t(LedStripType::WS2813), "WS2813" },            // 5V, data backup (same timing as WS2812)
            { uint32_t(LedStripType::WS2815), "WS2815" },            // 12V, data backup (same timing as WS2812)
            EnumItem(uint32_t(LedStripType::WS2812B))                // Default
        };

        EnumItem LedStripColorOrderDescr[] = {
            { uint32_t(LedStripColorOrder::UseDefault), "UseDefault" },
            { uint32_t(LedStripColorOrder::RGB), "RGB" },                 // Red, Green, Blue
            { uint32_t(LedStripColorOrder::GRB), "GRB" },                 // Green, Red, Blue (most common)
            { uint32_t(LedStripColorOrder::BGR), "BGR" },                 // Blue, Green, Red
            { uint32_t(LedStripColorOrder::GRBW), "GRBW" },               // Green, Red, Blue, White (SK6812 RGBW)
            EnumItem(uint32_t(uint32_t(LedStripColorOrder::UseDefault)))  // Default
        };

        // LED strip structure
        struct led_strip_t {
            rmt_channel_handle_t rmtChannel = nullptr;
            rmt_encoder_handle_t rmtEncoder = nullptr;
            uint8_t*             buffer     = nullptr;
            size_t               bufferSize = 0;
        };

        // RMT encoder for LED strips
        struct led_strip_encoder_t {
            rmt_encoder_t     base          = {};
            rmt_encoder_t*    bytes_encoder = nullptr;
            rmt_encoder_t*    copy_encoder  = nullptr;
            rmt_symbol_word_t led_bit0      = {};
            rmt_symbol_word_t led_bit1      = {};
            rmt_symbol_word_t reset         = {};
        };

        // RMT encoder callbacks
        static size_t led_strip_rmt_encode(
            rmt_encoder_t* encoder, rmt_channel_handle_t channel, const void* primary_data, size_t data_size, rmt_encode_state_t* ret_state) {
            led_strip_encoder_t* led_encoder     = __containerof(encoder, led_strip_encoder_t, base);
            rmt_encode_state_t   session_state   = RMT_ENCODING_RESET;
            rmt_encode_state_t   state           = RMT_ENCODING_RESET;
            size_t               encoded_symbols = 0;

            rmt_encoder_handle_t bytes_encoder = led_encoder->bytes_encoder;
            rmt_encoder_handle_t copy_encoder  = led_encoder->copy_encoder;

            switch (session_state) {
                case RMT_ENCODING_RESET:
                    encoded_symbols += bytes_encoder->encode(bytes_encoder, channel, primary_data, data_size, &state);
                    if (state & RMT_ENCODING_COMPLETE) {
                        session_state = RMT_ENCODING_COMPLETE;
                    }
                    if (state & RMT_ENCODING_MEM_FULL) {
                        state = rmt_encode_state_t(state | RMT_ENCODING_MEM_FULL);
                        goto out;
                    }
                // fallthrough
                case RMT_ENCODING_COMPLETE:
                    encoded_symbols += copy_encoder->encode(copy_encoder, channel, &led_encoder->reset, sizeof(led_encoder->reset), &state);
                    if (state & RMT_ENCODING_COMPLETE) {
                        session_state = RMT_ENCODING_RESET;
                    }
                    if (state & RMT_ENCODING_MEM_FULL) {
                        state = rmt_encode_state_t(state | RMT_ENCODING_MEM_FULL);
                        goto out;
                    }
                    // fallthrough
                default:
                    break;
            }
        out:
            *ret_state = state;
            return encoded_symbols;
        }

        static esp_err_t led_strip_rmt_encoder_del(rmt_encoder_t* encoder) {
            led_strip_encoder_t* led_encoder = __containerof(encoder, led_strip_encoder_t, base);
            rmt_del_encoder(led_encoder->bytes_encoder);
            rmt_del_encoder(led_encoder->copy_encoder);
            free(led_encoder);
            return ESP_OK;
        }

        static esp_err_t led_strip_rmt_encoder_reset(rmt_encoder_t* encoder) {
            led_strip_encoder_t* led_encoder = __containerof(encoder, led_strip_encoder_t, base);
            rmt_encoder_reset(led_encoder->bytes_encoder);
            rmt_encoder_reset(led_encoder->copy_encoder);
            return ESP_OK;
        }

    }

    void LedStripRMT::init() {
        Assert(!ledPin_.undefined(), "LED strip pin must be defined.");
        Assert(ledPin_.capabilities().has(Pins::PinCapabilities::Native), "Must be a GPIO (RMT capable) pin");
        auto gpio = ledPin_.getNative(Pin::Capabilities::Output | Pin::Capabilities::Native);

        Assert(!leds_.empty(), "LED strip: must specify leds");

        // Number of logical LEDs (how many we're controlling)
        numberLeds_ = leds_.size();

        // Find maximum physical LED index for buffer allocation
        int32_t maxLedIndex = 0;
        for (auto ledIdx : leds_) {
            if (ledIdx > maxLedIndex) {
                maxLedIndex = ledIdx;
            }
        }

        auto strip = std::make_unique<led_strip_t>();

        // Allocate buffer for all LEDs up to max index
        strip->bufferSize = (maxLedIndex + 1) * bytesPerLed_;
        strip->buffer     = new uint8_t[strip->bufferSize];
        memset(strip->buffer, 0, strip->bufferSize);  // Initialize to off

        // Configure RMT TX channel
        rmt_tx_channel_config_t txConfig = {};
        txConfig.gpio_num                = gpio_num_t(gpio);
        txConfig.clk_src                 = RMT_CLK_SRC_DEFAULT;
        txConfig.resolution_hz           = rmtResolutionHz_;
        txConfig.mem_block_symbols       = 64;
        txConfig.trans_queue_depth       = 4;
        Assert(rmt_new_tx_channel(&txConfig, &strip->rmtChannel) == ESP_OK, "create RMT TX channel failed");

        // Create encoder
        auto encoder = std::make_unique<led_strip_encoder_t>();

        encoder->base.encode = led_strip_rmt_encode;
        encoder->base.del    = led_strip_rmt_encoder_del;
        encoder->base.reset  = led_strip_rmt_encoder_reset;

        // Configure bit encoding based on timing
        encoder->led_bit0.level0    = 1;
        encoder->led_bit0.duration0 = timing_.t0h;
        encoder->led_bit0.level1    = 0;
        encoder->led_bit0.duration1 = timing_.t0l;

        encoder->led_bit1.level0    = 1;
        encoder->led_bit1.duration0 = timing_.t1h;
        encoder->led_bit1.level1    = 0;
        encoder->led_bit1.duration1 = timing_.t1l;

        // Reset code
        encoder->reset.level0    = 0;
        encoder->reset.duration0 = timing_.reset;
        encoder->reset.level1    = 0;
        encoder->reset.duration1 = 0;

        strip->rmtEncoder = &encoder->base;

        // Create bytes encoder
        rmt_bytes_encoder_config_t bytes_encoder_config = {};
        bytes_encoder_config.bit0                       = encoder->led_bit0;
        bytes_encoder_config.bit1                       = encoder->led_bit1;
        bytes_encoder_config.flags.msb_first            = 1;

        Assert(rmt_new_bytes_encoder(&bytes_encoder_config, &encoder->bytes_encoder) == ESP_OK, "LED strip: create bytes encoder failed");

        // Create copy encoder
        rmt_copy_encoder_config_t copy_encoder_config = {};
        Assert(rmt_new_copy_encoder(&copy_encoder_config, &encoder->copy_encoder) == ESP_OK, "LED strip: create copy encoder failed");

        // Enable RMT channel
        Assert(rmt_enable(strip->rmtChannel) == ESP_OK, "LED strip: enable RMT channel failed");

        log_info("Created LED strip");

        ledStripHandle_ = strip.release();

        // Calculate LED positions if direction/travel configured
        calculateLedPositions();

        // Initialize feedback system
        if (!feedback_) {
            feedback_ = new LedStripFeedback();
        }
        feedback_->init(this);
    }

    void LedStripRMT::deinit() {
        if (feedback_) {
            feedback_->deinit();
            delete feedback_;
            feedback_ = nullptr;
        }

        if (ledStripHandle_) {
            auto strip = static_cast<led_strip_t*>(ledStripHandle_);
            rmt_disable(strip->rmtChannel);
            rmt_del_encoder(strip->rmtEncoder);
            rmt_del_channel(strip->rmtChannel);
            delete[] strip->buffer;
            delete strip;
        }
    }

    void LedStripRMT::setPixel(uint16_t index, uint8_t r, uint8_t g, uint8_t b) {
        if (!ledStripHandle_ || index >= numberLeds_) {
            return;
        }

        // Map logical index to physical LED index
        int32_t physicalIndex = leds_[index];

        auto     strip = static_cast<led_strip_t*>(ledStripHandle_);
        uint8_t* pixel = strip->buffer + (physicalIndex * bytesPerLed_);

        // Set color based on color order
        switch (colorOrder_) {
            case LedStripColorOrder::RGB:
                pixel[0] = r;
                pixel[1] = g;
                pixel[2] = b;
                break;
            case LedStripColorOrder::UseDefault:
            case LedStripColorOrder::GRB:
                pixel[0] = g;
                pixel[1] = r;
                pixel[2] = b;
                break;
            case LedStripColorOrder::BGR:
                pixel[0] = b;
                pixel[1] = g;
                pixel[2] = r;
                break;
            case LedStripColorOrder::GRBW:
                pixel[0] = g;
                pixel[1] = r;
                pixel[2] = b;
                pixel[3] = 0;  // White channel = 0 for RGB mode
                break;
        }
    }

    void LedStripRMT::refresh() {
        if (ledStripHandle_) {
            auto                  strip    = static_cast<led_strip_t*>(ledStripHandle_);
            rmt_transmit_config_t tx_cfg   = {};
            tx_cfg.loop_count              = 0;
            tx_cfg.flags.queue_nonblocking = 1;

            if (rmt_transmit(strip->rmtChannel, strip->rmtEncoder, strip->buffer, strip->bufferSize, &tx_cfg) == ESP_OK) {
                rmt_tx_wait_all_done(strip->rmtChannel, 0);  // non-blocking after all.
            }
        }
    }

    void LedStripRMT::clear() {
        if (ledStripHandle_) {
            auto strip = static_cast<led_strip_t*>(ledStripHandle_);
            memset(strip->buffer, 0, strip->bufferSize);
        }
    }

    void LedStripRMT::group(Configuration::HandlerBase& handler) {
        handler.item("pin", ledPin_);
        {
            uint32_t v = uint32_t(type_);
            handler.item("type", v, LedStripTypeDescr);
            type_ = LedStripType(v);
        }
        {
            uint32_t v = uint32_t(colorOrder_);
            handler.item("color_order", v, LedStripColorOrderDescr);
            colorOrder_ = LedStripColorOrder(v);
        }
        handler.item("leds", leds_);
        handler.item("travel", travel_);
        handler.item("direction", direction_);
        handler.item("bytes_per_led", bytesPerLed_);

        handler.item("timing_1l", timing_.t1l);
        handler.item("timing_1h", timing_.t1h);
        handler.item("timing_0l", timing_.t0l);
        handler.item("timing_0h", timing_.t0h);
        handler.item("timing_rst", timing_.reset);

        handler.item("resolution_hz", rmtResolutionHz_);

        // Feedback configuration
        handler.section("feedback", feedback_);
    }

    void LedStripRMT::afterParse() {
        // Check timing_:
        if (timing_.t1l == 0 && timing_.t1h == 0 && timing_.t0h == 0 && timing_.t0l == 0 && timing_.reset == 0) {
            switch (type_) {
                case LedStripType::WS2812:
                case LedStripType::WS2812B:
                case LedStripType::WS2813:
                case LedStripType::WS2815:
                case LedStripType::WS2811_FAST:
                    timing_.t0h   = 4;    // 0.4�s
                    timing_.t0l   = 9;    // 0.9�s
                    timing_.t1h   = 8;    // 0.8�s
                    timing_.t1l   = 6;    // 0.6�s
                    timing_.reset = 500;  // 50�s
                    if (colorOrder_ == LedStripColorOrder::UseDefault) {
                        colorOrder_ = LedStripColorOrder::GRB;
                    }
                    if (bytesPerLed_ == 0) {
                        bytesPerLed_ = 3;
                    }
                    break;

                case LedStripType::SK6812:
                case LedStripType::SK6812_RGBW:
                    timing_.t0h   = 3;    // 0.3�s
                    timing_.t0l   = 9;    // 0.9�s
                    timing_.t1h   = 6;    // 0.6�s
                    timing_.t1l   = 6;    // 0.6�s
                    timing_.reset = 800;  // 80�s
                    break;

                case LedStripType::WS2811_SLOW:
                    timing_.t0h   = 5;    // 0.5�s
                    timing_.t0l   = 20;   // 2.0�s
                    timing_.t1h   = 12;   // 1.2�s
                    timing_.t1l   = 13;   // 1.3�s
                    timing_.reset = 500;  // 50�s

                    break;
            }
        } else {
            if (timing_.t1l == 0 || timing_.t1h == 0 || timing_.t0h == 0 || timing_.t0l == 0 || timing_.reset == 0) {
                Assert(false, "Either provide all the timings (t1l, t1h, t0l, t0h and reset) or none.");
            }
        }

        if (bytesPerLed_ == 0) {
            switch (type_) {
                case LedStripType::WS2812:
                case LedStripType::WS2812B:
                case LedStripType::WS2813:
                case LedStripType::WS2815:
                case LedStripType::WS2811_FAST:
                case LedStripType::WS2811_SLOW:
                case LedStripType::SK6812:
                    bytesPerLed_ = 3;
                    break;
                case LedStripType::SK6812_RGBW:
                    bytesPerLed_ = 4;
                    break;
            }
        }

        if (colorOrder_ == LedStripColorOrder::UseDefault) {
            switch (type_) {
                case LedStripType::WS2812:
                case LedStripType::WS2812B:
                case LedStripType::WS2813:
                case LedStripType::WS2815:
                case LedStripType::WS2811_FAST:
                    colorOrder_ = LedStripColorOrder::GRB;
                    break;

                case LedStripType::SK6812:
                    colorOrder_ = LedStripColorOrder::GRB;
                    break;

                case LedStripType::SK6812_RGBW:
                    colorOrder_ = LedStripColorOrder::GRBW;
                    break;

                case LedStripType::WS2811_SLOW:
                    colorOrder_ = LedStripColorOrder::GRB;  // Can be RGB, check your strip!
                    break;
            }
        }
    }

    void LedStripRMT::calculateLedPositions() {
        // Early exit if no direction/travel configured
        if (direction_.empty() || travel_.empty() || leds_.empty()) {
            return;
        }

        int numLeds = leds_.size();

        // Parse direction string - skip spaces
        std::vector<char> directions;
        for (char c : direction_) {
            if (c != ' ' && c != '\t') {
                char upper = std::toupper(c);
                if (upper == 'X' || upper == 'Y' || upper == 'Z') {
                    directions.push_back(upper);
                }
            }
        }

        // Validate: directions and travel should have same length
        if (directions.size() != travel_.size()) {
            log_error("LED strip: direction length (" << directions.size() << ") != travel length (" << travel_.size() << ")");
            return;
        }

        // Build segments
        struct Segment {
            char  axis;
            float distance;
            int   startLed;
            int   numLeds;
        };
        std::vector<Segment> segments;

        int ledIndex = 0;
        for (size_t i = 0; i < directions.size(); i++) {
            Segment seg;
            seg.axis     = directions[i];
            seg.distance = travel_[i];
            seg.startLed = ledIndex;

            // Count consecutive LEDs with same direction
            int numInSegment = 0;
            while (ledIndex < numLeds && directions.size() > i) {
                // Check if we've moved to next segment
                if (i + 1 < directions.size() && ledIndex > 0) {
                    // Simple heuristic: if we have more segments than LEDs left, move to next
                    int ledsRemaining     = numLeds - ledIndex;
                    int segmentsRemaining = directions.size() - i;
                    if (numInSegment > 0 && ledsRemaining <= segmentsRemaining) {
                        break;
                    }
                }
                numInSegment++;
                ledIndex++;

                // If this is the last segment, take all remaining LEDs
                if (i == directions.size() - 1) {
                    numInSegment = numLeds - seg.startLed;
                    ledIndex     = numLeds;
                    break;
                }

                // Otherwise, distribute evenly
                float avgLedsPerSegment = (float)(numLeds - seg.startLed) / (directions.size() - i);
                if (numInSegment >= avgLedsPerSegment) {
                    break;
                }
            }

            seg.numLeds = numInSegment;
            if (seg.numLeds > 0) {
                segments.push_back(seg);
            }
        }

        // Calculate positions
        ledPositions_.clear();
        ledPositions_.reserve(numLeds);

        float currentX = 0, currentY = 0, currentZ = 0;

        for (const auto& seg : segments) {
            float startX = currentX, startY = currentY, startZ = currentZ;

            // Calculate end position for this segment
            float endX = currentX, endY = currentY, endZ = currentZ;
            switch (seg.axis) {
                case 'X':
                    endX += seg.distance;
                    break;
                case 'Y':
                    endY += seg.distance;
                    break;
                case 'Z':
                    endZ += seg.distance;
                    break;
                default:
                    continue;  // ignore.
            }

            // Distribute LEDs along segment
            for (int i = 0; i < seg.numLeds; i++) {
                float t = (seg.numLeds > 1) ? (float)i / (seg.numLeds - 1) : 0.5f;

                LedPosition pos;
                pos.x = startX + t * (endX - startX);
                pos.y = startY + t * (endY - startY);
                pos.z = startZ + t * (endZ - startZ);

                ledPositions_.push_back(pos);
            }

            currentX = endX;
            currentY = endY;
            currentZ = endZ;
        }

        log_info("LED strip: calculated positions for " << ledPositions_.size() << " LEDs over " << segments.size() << " segments");
    }

    // Configuration registration
    namespace {
        ConfigurableModuleFactory::InstanceBuilder<Extra::LedStripRMT> registration("led_strip");
    }
}
