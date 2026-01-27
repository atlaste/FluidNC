#pragma once

#include "Module.h"
#include "Pin.h"
#include "Spindles/Spindle.h"
#include "Scheduler/ISchedulable.h"

#include <driver/pulse_cnt.h>

class SpindleEncoder : public ConfigurableModule {
    Pin     pin_a;
    Pin     pin_b;
    int32_t ratio;      // in 1/10000
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
    int32_t alarmValue         = 0;

    // Encoder-driven stepping callback (for G95/G33)
    // Callback should execute one step cycle and return encoder counts until next step
    // Returns 0 when motion is complete
    typedef bool (*encoder_step_callback_t)();

    volatile encoder_step_callback_t encoder_callback         = nullptr;
    volatile int64_t                 encoder_last_alm_count   = 0;  // Last total count when ALM fired
    volatile int32_t                 encoder_counts_remaining = 0;  // Remaining counts to next step (for >16-bit threshold handling)

    // Threading sync state
    volatile int64_t target_absolute_count = 0;   // Target count for setCountAlarm()
    volatile bool    waiting_for_index     = false;  // True when waiting for index pulse

    Scheduler::Event* monitorTask_ = nullptr;  // Track the scheduled monitoring task

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
    void startStepCallback();
    void stopStepCallback();
    void setStepAlarmValue(int32_t value);

    // Threading sync methods (for G33/G76)
    // setIndexAlarm: wait for index pulse (once per revolution)
    // setCountAlarm: wait until encoder reaches a specific absolute count
    void setIndexAlarm();
    void setCountAlarm(int32_t target_count);

    inline int32_t countsPerStep(int64_t total_steps) {
        // Calculate encoder counts per step using integer arithmetic
        // feed_per_revolution is in mm/rev (how much the tool moves per spindle revolution)
        // We need: encoder_counts_per_step = (encoder_CPR * gear_ratio) / total_steps

        // For one revolution: encoder counts = CPR * gear_ratio / 10000
        // For this block: steps = step_event_count
        // Result: encoder_counts_per_step = (CPR * ratio) / (10000 * step_event_count)

        // Use fixed-point arithmetic with 1000x scaling to preserve precision
        
        int64_t encoder_cpr = countPerRevolution;
        int64_t gear_ratio  = ratio;  // Already scaled by 10000

        // counts_per_step_fp = (CPR * ratio * 1000) / (10000 * total_steps)
        //                    = (CPR * ratio) / (10 * total_steps)
        auto result = int32_t((encoder_cpr * gear_ratio) / (10 * total_steps));

        log_info("Encoder sync: CPR=" << encoder_cpr << ", ratio=" << gear_ratio << ", counts/step=" << (result / 1000) << "."
                                      << (result % 1000) << ", steps=" << total_steps);
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
