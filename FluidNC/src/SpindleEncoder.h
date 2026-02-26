#pragma once

#include "Module.h"
#include "Pin.h"
#include "Spindles/Spindle.h"
#include "Scheduler/ISchedulable.h"

#include <driver/pulse_cnt.h>

class SpindleEncoder : public ConfigurableModule {
    Pin     pin_a;
    Pin     pin_b;
    int32_t ratio;  // in 1/10000
    int32_t allowedErrors = 5;
    int32_t errorCount    = 0;

    volatile int64_t   totalCount = 0;
    pcnt_unit_handle_t pcnt_total = NULL;
    pcnt_unit_handle_t pcnt_alm   = NULL;

    static bool pcnt_on_overflow(pcnt_unit_handle_t unit, const pcnt_watch_event_data_t* edata, void* user_ctx);
    static bool pcnt_on_reach(pcnt_unit_handle_t unit, const pcnt_watch_event_data_t* edata, void* user_ctx);

    using SetpointReachedCallback = void (*)(void*);

    int32_t tolerance          = 10;   // 10% RPM tolerance. If we go out of this range, it's an alarm.
    int64_t lastCount          = 0;    // For ISR validation (during motion)
    int64_t lastCountIdle      = 0;    // For coroutine validation (when idle)
    int32_t countPerRevolution = 800;  // encoder CPR

    // For stepping we need the FP value (count per step / 1024) and the remainder to know if we should add an extra count
    int32_t current_step_fp_remainder = 0;
    int32_t current_step_fp           = 0;

    // Encoder-driven stepping callback (for G95/G33)
    // Callback should execute one step cycle and return encoder counts until next step
    // Returns 0 when motion is complete
    typedef bool (*encoder_step_callback_t)();

    volatile encoder_step_callback_t encoder_callback         = nullptr;
    volatile int32_t                 encoder_counts_remaining = 0;  // Remaining counts to next step (for >16-bit threshold handling)

    // Counting direction: +1 for positive, -1 for negative (determined by monitorSpeed)
    volatile int8_t countDirection_ = 1;

    // Threading sync state
    volatile int64_t target_absolute_count = 0;  // Target count for setCountAlarm()

    Scheduler::Event* monitorTask_ = nullptr;  // Track the scheduled monitoring task

    void armAlarm(int64_t targetCount);

public:
    SpindleEncoder(const char* name) : ConfigurableModule(name) {}

    void init() override;
    void deinit() override;

    int64_t getCount();

    void resetCount() { lastCount = getCount(); }

    // ISR-safe validation - returns true if in tolerance, false if out of tolerance
    // fromISR: true when called from Stepper ISR, false when called from idle coroutine
    bool validateSpeed(int32_t usecs, bool fromISR = true);

    // Coroutine for periodic spindle speed monitoring (runs when idle)
    Scheduler::Schedulable<void> monitorSpeed();

    // Encoder-driven stepping callback registration (for G95/G33)
    // Callback is called from ALM ISR and should return encoder counts until next step (0 = complete)
    void registerStepCallback(encoder_step_callback_t callback);
    void unregisterStepCallback();
    void startStepCallback(int64_t target);
    void stopStepCallback();

    // Gets the value to set for the next step alarm given a number of counts
    int64_t setStepAlarmValue(int32_t counts_per_step_fp);

    // Threading sync methods (for G33/G76)
    // setIndexAlarm: wait for index pulse (once per revolution)
    // setCountAlarm: wait until encoder reaches a specific absolute count
    int64_t setIndexAlarm();
    int64_t setCountAlarm(int32_t target_count);

    // Calculate encoder counts per motor step for spindle-synchronized motion
    // pitch_mm: thread pitch (mm per spindle revolution)
    // steps_per_mm: motor steps per mm for the sync axis
    // Returns: fixed-point encoder counts per step (scaled by 1024)
    inline int32_t countsPerStep(float pitch_mm, float steps_per_mm) {
        // encoder_counts_per_rev = CPR × gear_ratio / 10000
        // steps_per_rev = pitch_mm × steps_per_mm
        // counts_per_step = encoder_counts_per_rev / steps_per_rev
        //                 = (CPR × gear_ratio / 10000) / (pitch_mm × steps_per_mm)
        //                 = (CPR × gear_ratio) / (10000 × pitch_mm × steps_per_mm)

        // Use fixed-point: multiply by 1024 for precision
        // counts_per_step_fp = (CPR × ratio × 1024) / (10000 × pitch × steps_per_mm)
        //                    = (CPR × ratio) / (10 × pitch × steps_per_mm)

        int64_t encoder_cpr = countPerRevolution;
        int64_t gear_ratio  = ratio;  // Already scaled by 10000

        float steps_per_rev = pitch_mm * steps_per_mm;
        
        // Guard against divide-by-zero (shouldn't happen if caller checks feed_per_revolution > 0)
        if (steps_per_rev == 0) {
            return 0;
        }
        
        auto result = int32_t((encoder_cpr * gear_ratio * 1024) / (10000 * int64_t(steps_per_rev)));

        // Debug logging (commented out for production - too verbose during motion)
        // log_info("Encoder sync: CPR=" << encoder_cpr << ", ratio=" << gear_ratio 
        //          << ", pitch=" << pitch_mm << "mm, steps/rev=" << steps_per_rev 
        //          << ", counts/step=" << (result / 1024));

        return result;
    }

    uint32_t lastEncoderSpeed_ = 0;

    void group(Configuration::HandlerBase& handler) override {
        handler.item("pin_a", pin_a);
        handler.item("pin_b", pin_b);
        handler.item("tolerance", tolerance);
        handler.item("allowed_errors", allowedErrors);
        handler.item("cpr", countPerRevolution);
        float r = float(ratio / 10000.0);
        handler.item("gear_ratio", r);
        ratio = int(r * 10000.0);
    }

    virtual ~SpindleEncoder() {}
};

extern SpindleEncoder* spindle_encoder;
