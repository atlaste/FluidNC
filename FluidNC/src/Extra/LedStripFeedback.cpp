#include "LedStripFeedback.h"
#include "../System.h"
#include "../Config.h"
#include "../Scheduler/Platform.h"
#include <cmath>
#include <algorithm>

namespace Extra {

    // Global callbacks
    void (*g_probeCallback)()                   = nullptr;
    void (*g_progressCallback)(float percent)   = nullptr;
    void (*g_startupCompleteCallback)()         = nullptr;
    static LedStripFeedback* g_feedbackInstance = nullptr;

    static void globalProbeCallback() {
        if (g_feedbackInstance) {
            g_feedbackInstance->onProbeTouch();
        }
    }

    static void globalProgressCallback(float percent) {
        if (g_feedbackInstance) {
            g_feedbackInstance->setProgress(percent);
        }
    }

    static void globalStartupCompleteCallback() {
        if (g_feedbackInstance) {
            g_feedbackInstance->endStartup();
        }
    }

    // Task runs at ~60 FPS for smooth animations
    static constexpr uint32_t UPDATE_INTERVAL_MS   = 16;
    static constexpr uint32_t PROBE_FLASH_DURATION = 30;  // frames (~500ms)

    LedStripFeedback::~LedStripFeedback() {
        deinit();
    }

    void LedStripFeedback::init(LedStripRMT* strip) {
        strip_ = strip;

        // Generate sine lookup table (only place we use sin/cos)
        for (int i = 0; i < SINE_TABLE_SIZE; i++) {
            float angle   = (i * 2.0f * M_PI) / SINE_TABLE_SIZE;
            float value   = (std::sin(angle) + 1.0f) * 0.5f;  // 0.0 to 1.0
            sineTable_[i] = static_cast<uint8_t>(value * 255);
        }

        // Register for system state changes
        sys.register_change_handler(onSystemChange, this);

        // Register global callbacks
        g_feedbackInstance        = this;
        g_probeCallback           = globalProbeCallback;
        g_progressCallback        = globalProgressCallback;
        g_startupCompleteCallback = globalStartupCompleteCallback;

        // Start LED update coroutine
        schedulerEvent_ = Scheduler::schedule(updateCoroutine());

        log_info("LED feedback initialized");
    }

    void LedStripFeedback::deinit() {
        // Unregister global callbacks
        if (g_feedbackInstance == this) {
            g_probeCallback           = nullptr;
            g_progressCallback        = nullptr;
            g_startupCompleteCallback = nullptr;
            g_feedbackInstance        = nullptr;
        }

        // Cancel scheduler event
        if (schedulerEvent_) {
            Scheduler::slowScheduler->unschedule(schedulerEvent_);
            schedulerEvent_ = nullptr;
        }
    }

    uint8_t LedStripFeedback::fastSine(uint32_t phase) {
        return sineTable_[phase & 0xFF];
    }

    LedStripFeedback::Color LedStripFeedback::hsvToRgb(uint8_t hue, uint8_t sat, uint8_t val) {
        // Fast HSV to RGB conversion
        // hue: 0-255, sat: 0-255, val: 0-255
        if (sat == 0) {
            return { val, val, val };
        }

        uint8_t region    = hue / 43;  // 0-5
        uint8_t remainder = (hue - (region * 43)) * 6;

        uint8_t p = (val * (255 - sat)) >> 8;
        uint8_t q = (val * (255 - ((sat * remainder) >> 8))) >> 8;
        uint8_t t = (val * (255 - ((sat * (255 - remainder)) >> 8))) >> 8;

        switch (region) {
            case 0:
                return { val, t, p };
            case 1:
                return { q, val, p };
            case 2:
                return { p, val, t };
            case 3:
                return { p, q, val };
            case 4:
                return { t, p, val };
            default:
                return { val, p, q };
        }
    }

    void LedStripFeedback::setProgress(float percent) {
        progressPercent_ = std::max(0.0f, std::min(1.0f, percent));
        hasProgress_     = true;
    }

    void LedStripFeedback::clearProgress() {
        hasProgress_     = false;
        progressPercent_ = 0.0f;
    }

    void LedStripFeedback::onProbeTouch() {
        probeTriggered_    = true;
        probeFlashCounter_ = PROBE_FLASH_DURATION;
    }

    void LedStripFeedback::endStartup() {
        inStartup_ = false;
    }

    void LedStripFeedback::updateMachinePosition() {
        float* mpos = get_mpos();
        machineX_   = mpos[0];
        machineY_   = mpos[1];
        machineZ_   = mpos[2];
    }

    uint8_t LedStripFeedback::getPositionBoost(int ledIndex) {
        if (!strip_ || !strip_->hasPositionMapping() || ledIndex < 0) {
            return 0;
        }

        const auto& positions = strip_->getLedPositions();
        if (ledIndex >= positions.size()) {
            return 0;
        }

        const auto& ledPos = positions[ledIndex];

        // Calculate distance from current position to this LED
        float dx     = machineX_ - ledPos.x;
        float dy     = machineY_ - ledPos.y;
        float dz     = machineZ_ - ledPos.z;
        float distSq = dx * dx + dy * dy + dz * dz;
        float dist   = std::sqrt(distSq);

        // Apply smooth falloff - full boost at 0mm, zero boost at 100mm
        // Using inverse square falloff for smooth gradients
        constexpr float maxDist = 100.0f;  // mm
        if (dist >= maxDist) {
            return 0;
        }

        // Smooth falloff: 1 - (dist/maxDist)^2
        float factor = 1.0f - (dist / maxDist);
        factor       = factor * factor;  // Square for smoother falloff

        return static_cast<uint8_t>(positionBoost_ * factor);
    }

    void LedStripFeedback::onSystemChange(SystemDirty changes, const system_t& state, void* userData) {
        auto* feedback = static_cast<LedStripFeedback*>(userData);
        if (int(changes) & int(SystemDirty::State)) {
            State newState          = state.state();
            State oldState          = feedback->currentState_;
            feedback->currentState_ = newState;

            // Clear progress when exiting cycle state
            if (oldState == State::Cycle && newState != State::Cycle && newState != State::Hold && newState != State::Held) {
                feedback->clearProgress();
            }
        }
    }

    LedStripFeedback::Color LedStripFeedback::getIdleColor(uint8_t brightness) {
        // Warm amber/yellow: more red, good amount of green, minimal blue
        return { brightness, static_cast<uint8_t>(brightness * 0.7f), static_cast<uint8_t>(brightness * 0.2f) };
    }

    LedStripFeedback::Color LedStripFeedback::getRunningColor(uint8_t brightness) {
        // Greenish overlay
        return { static_cast<uint8_t>(brightness * 0.3f), brightness, static_cast<uint8_t>(brightness * 0.3f) };
    }

    LedStripFeedback::Color LedStripFeedback::getAlarmColor(uint8_t brightness) {
        // Warm red (not pure red, closer to base color)
        return { brightness, static_cast<uint8_t>(brightness * 0.4f), static_cast<uint8_t>(brightness * 0.2f) };
    }

    LedStripFeedback::Color LedStripFeedback::getHoldColor(uint8_t brightness) {
        // Orange
        return { brightness, static_cast<uint8_t>(brightness * 0.5f), 0 };
    }

    LedStripFeedback::Color LedStripFeedback::getHomingColor(uint8_t brightness) {
        // Cyan/blue
        return { static_cast<uint8_t>(brightness * 0.2f), static_cast<uint8_t>(brightness * 0.8f), brightness };
    }

    void LedStripFeedback::updateStartup() {
        if (!strip_)
            return;

        int numLeds = strip_->numberLeds_;
        if (numLeds == 0)
            return;

        // Slow rainbow wave
        uint8_t baseHue    = (animationCounter_ / 2) & 0xFF;  // Slow rotation
        uint8_t brightness = (idleBrightness_ * 255) / 100;

        for (int i = 0; i < numLeds; i++) {
            // Each LED offset in hue creates the wave
            uint8_t hue   = baseHue + ((i * 256) / numLeds);
            Color   color = hsvToRgb(hue, 255, brightness);
            strip_->setPixel(i, color.r, color.g, color.b);
        }

        strip_->refresh();
    }

    void LedStripFeedback::updateIdle() {
        if (!strip_)
            return;

        // Update machine position for position-based boost
        updateMachinePosition();

        // 30 second breathing cycle
        uint32_t msPerCycle = idleBreathePeriod_;
        uint32_t phase      = (animationCounter_ / (UPDATE_INTERVAL_MS)) % (msPerCycle / UPDATE_INTERVAL_MS);
        uint8_t  sineIdx    = (phase * SINE_TABLE_SIZE * UPDATE_INTERVAL_MS) / msPerCycle;

        // Get breathing brightness (70-100% of idleBrightness_)
        uint8_t sineVal        = fastSine(sineIdx);
        uint8_t brightness     = (idleBrightness_ * 70) / 100 + ((sineVal * idleBrightness_ * 30) / (255 * 100));
        uint8_t baseBrightness = (brightness * 255) / 100;

        Color baseColor = getIdleColor(baseBrightness);

        // Apply position-based boost to each LED
        int numLeds = strip_->numberLeds_;
        for (int i = 0; i < numLeds; i++) {
            uint8_t boost = getPositionBoost(i);

            // Add boost while respecting maximum brightness
            uint8_t r = std::min(255, baseColor.r + boost);
            uint8_t g = std::min(255, baseColor.g + boost);
            uint8_t b = std::min(255, baseColor.b + boost);

            strip_->setPixel(i, r, g, b);
        }

        strip_->refresh();
    }

    void LedStripFeedback::updateRunning() {
        if (!strip_)
            return;

        int numLeds = strip_->numberLeds_;
        if (numLeds == 0)
            return;

        // Dim base color to 40%
        uint8_t baseBrightness = (idleBrightness_ * 40 * 255) / (100 * 100);
        Color   baseColor      = getIdleColor(baseBrightness);

        // Green overlay boost
        uint8_t boostBrightness = (runningBoost_ * 255) / 100;
        Color   boostColor      = getRunningColor(boostBrightness);

        if (hasProgress_) {
            // Show as progress bar fill
            int fillLeds = static_cast<int>(progressPercent_ * numLeds);

            for (int i = 0; i < numLeds; i++) {
                if (i < fillLeds) {
                    // Add green boost
                    uint8_t r = std::min(255, baseColor.r + boostColor.r);
                    uint8_t g = std::min(255, baseColor.g + boostColor.g);
                    uint8_t b = std::min(255, baseColor.b + boostColor.b);
                    strip_->setPixel(i, r, g, b);
                } else {
                    strip_->setPixel(i, baseColor.r, baseColor.g, baseColor.b);
                }
            }
        } else {
            // Moving wave animation
            uint8_t phase = (animationCounter_ / 4) & 0xFF;  // Slower wave

            for (int i = 0; i < numLeds; i++) {
                uint8_t ledPhase = (phase + (i * 256) / numLeds) & 0xFF;
                uint8_t wave     = fastSine(ledPhase);

                uint8_t r = baseColor.r + ((boostColor.r * wave) / 255);
                uint8_t g = baseColor.g + ((boostColor.g * wave) / 255);
                uint8_t b = baseColor.b + ((boostColor.b * wave) / 255);

                strip_->setPixel(i, r, g, b);
            }
        }

        strip_->refresh();
    }

    void LedStripFeedback::updateHoming() {
        if (!strip_)
            return;

        int numLeds = strip_->numberLeds_;
        if (numLeds == 0)
            return;

        // Wave moving along strip
        uint8_t phase          = (animationCounter_ / 2) & 0xFF;
        uint8_t baseBrightness = (idleBrightness_ * 255) / 100;
        Color   color          = getHomingColor(baseBrightness);

        for (int i = 0; i < numLeds; i++) {
            uint8_t ledPhase = (phase + (i * 256) / numLeds) & 0xFF;
            uint8_t wave     = fastSine(ledPhase);

            uint8_t r = (color.r * wave) / 255;
            uint8_t g = (color.g * wave) / 255;
            uint8_t b = (color.b * wave) / 255;

            strip_->setPixel(i, r, g, b);
        }

        strip_->refresh();
    }

    void LedStripFeedback::updateHold() {
        if (!strip_)
            return;

        int numLeds = strip_->numberLeds_;
        if (numLeds == 0)
            return;

        // Base dimmed color
        uint8_t baseBrightness = (idleBrightness_ * 40 * 255) / (100 * 100);
        Color   baseColor      = getIdleColor(baseBrightness);

        // Orange pulse on last progress LED
        uint8_t pulsePhase      = (animationCounter_ / 8) & 0xFF;  // ~2 second pulse
        uint8_t pulse           = fastSine(pulsePhase);
        uint8_t pulseBrightness = (pulse * idleBrightness_ * 255) / (255 * 100);
        Color   pulseColor      = getHoldColor(pulseBrightness);

        int pulseLed = hasProgress_ ? static_cast<int>(progressPercent_ * numLeds) : numLeds - 1;
        if (pulseLed >= numLeds)
            pulseLed = numLeds - 1;

        for (int i = 0; i < numLeds; i++) {
            if (i == pulseLed) {
                strip_->setPixel(i, pulseColor.r, pulseColor.g, pulseColor.b);
            } else {
                strip_->setPixel(i, baseColor.r, baseColor.g, baseColor.b);
            }
        }

        strip_->refresh();
    }

    void LedStripFeedback::updateSafetyDoor() {
        if (!strip_)
            return;

        int numLeds = strip_->numberLeds_;

        // Red pulse (similar to hold but red)
        uint8_t pulsePhase = (animationCounter_ / 8) & 0xFF;
        uint8_t pulse      = fastSine(pulsePhase);
        uint8_t brightness = (pulse * idleBrightness_ * 255) / (255 * 100);
        Color   color      = getAlarmColor(brightness);

        for (int i = 0; i < numLeds; i++) {
            strip_->setPixel(i, color.r, color.g, color.b);
        }

        strip_->refresh();
    }

    void LedStripFeedback::updateAlarm() {
        if (!strip_)
            return;

        int numLeds = strip_->numberLeds_;

        // Solid warm red
        uint8_t brightness = (idleBrightness_ * 255) / 100;
        Color   color      = getAlarmColor(brightness);

        for (int i = 0; i < numLeds; i++) {
            strip_->setPixel(i, color.r, color.g, color.b);
        }

        strip_->refresh();
    }

    void LedStripFeedback::renderProbeFlash() {
        if (!strip_ || probeFlashCounter_ == 0)
            return;

        int numLeds = strip_->numberLeds_;

        // White flash, fading out
        uint8_t brightness = (255 * probeFlashCounter_) / PROBE_FLASH_DURATION;

        for (int i = 0; i < numLeds; i++) {
            strip_->setPixel(i, brightness, brightness, brightness);
        }

        strip_->refresh();
        probeFlashCounter_--;
    }

    Scheduler::Schedulable<void> LedStripFeedback::updateCoroutine() {
        while (true) {
            if (!enabled_) {
                co_yield 100_msec;
                continue;
            }
            
            // Startup rainbow overrides everything except probe flash
            if (inStartup_) {
                if (probeFlashCounter_ > 0) {
                    renderProbeFlash();
                } else {
                    updateStartup();
                }
            }
            // Probe flash overrides everything else
            else if (probeFlashCounter_ > 0) {
                renderProbeFlash();
            } else {
                // Update based on current state
                switch (currentState_) {
                    case State::Idle:
                    case State::CheckMode:
                        updateIdle();
                        break;
                        
                    case State::Cycle:
                    case State::Jog:
                        updateRunning();
                        break;
                        
                    case State::Homing:
                        updateHoming();
                        break;
                        
                    case State::Hold:
                    case State::Held:
                        updateHold();
                        break;
                        
                    case State::SafetyDoor:
                        updateSafetyDoor();
                        break;
                        
                    case State::Alarm:
                    case State::ConfigAlarm:
                    case State::Critical:
                        updateAlarm();
                        break;
                        
                    case State::Sleep:
                        // Turn off LEDs
                        if (strip_) {
                            strip_->clear();
                            strip_->refresh();
                        }
                        break;
                }
            }
            
            animationCounter_++;
            co_yield 16_msec;  // 60 FPS
        }
    }

    void LedStripFeedback::group(Configuration::HandlerBase& handler) {
        handler.item("enabled", enabled_);
        handler.item("idle_brightness", idleBrightness_);
        handler.item("idle_breathe_period", idleBreathePeriod_);
        handler.item("running_boost", runningBoost_);
        handler.item("position_boost", positionBoost_);
    }
}
