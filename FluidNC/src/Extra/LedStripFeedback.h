#pragma once

#include "LedStripRMT.h"
#include "../State.h"
#include "../System.h"
#include "Configuration/HandlerBase.h"
#include "../Scheduler/ISchedulable.h"

namespace Extra {
    // Global callbacks (set by LedStripFeedback when initialized)
    extern void (*g_probeCallback)();
    extern void (*g_progressCallback)(float percent);  // 0.0-1.0
    extern void (*g_startupCompleteCallback)();

    class LedStripFeedback : public Configuration::Configurable {
    private:
        LedStripRMT* strip_ = nullptr;

        // Configurable parameters
        uint8_t  idleBrightness_    = 70;     // 0-100, base brightness when idle
        uint32_t idleBreathePeriod_ = 30000;  // milliseconds for one breath cycle
        uint8_t  runningBoost_      = 10;     // brightness boost when running
        uint8_t  positionBoost_     = 25;     // brightness boost around position
        bool     enabled_           = true;

        // Runtime state
        volatile State currentState_    = State::Idle;
        volatile bool  probeTriggered_  = false;
        volatile float progressPercent_ = 0.0f;  // 0.0-1.0
        volatile bool  hasProgress_     = false;
        volatile bool  inStartup_       = true;

        uint32_t animationCounter_  = 0;
        uint32_t probeFlashCounter_ = 0;

        // Machine position (updated periodically)
        float machineX_ = 0.0f;
        float machineY_ = 0.0f;
        float machineZ_ = 0.0f;

        // Fast sine lookup table (256 entries, 0-255 range)
        static constexpr int SINE_TABLE_SIZE = 256;
        uint8_t              sineTable_[SINE_TABLE_SIZE];

        // Scheduler event
        Scheduler::Event* schedulerEvent_ = nullptr;

        // Color helpers (returns brightness 0-255)
        struct Color {
            uint8_t r, g, b;
        };
        Color getIdleColor(uint8_t brightness);
        Color getRunningColor(uint8_t brightness);
        Color getAlarmColor(uint8_t brightness);
        Color getHoldColor(uint8_t brightness);
        Color getHomingColor(uint8_t brightness);

        // Position-based boost
        uint8_t getPositionBoost(int ledIndex);  // Returns 0-positionBoost_
        void    updateMachinePosition();

        // Animation methods
        void updateStartup();
        void updateIdle();
        void updateRunning();
        void updateHoming();
        void updateHold();
        void updateSafetyDoor();
        void updateAlarm();
        void renderProbeFlash();

        // Fast lookup
        uint8_t fastSine(uint32_t phase);                         // phase 0-255
        Color   hsvToRgb(uint8_t hue, uint8_t sat, uint8_t val);  // Fast HSV->RGB

        // Coroutine for LED updates
        Scheduler::Schedulable<void> updateCoroutine();

    public:
        LedStripFeedback() = default;
        ~LedStripFeedback();

        void init(LedStripRMT* strip);
        void deinit();

        void setProgress(float percent);
        void clearProgress();
        void onProbeTouch();
        void endStartup();  // Called when system is fully initialized

        // System state callback
        static void onSystemChange(SystemDirty changes, const system_t& state, void* userData);

        void group(Configuration::HandlerBase& handler);
    };
}
