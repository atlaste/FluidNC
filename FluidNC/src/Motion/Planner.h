// Copyright (c) 2022 -	Stefan de Bruijn
// Use of this source code is governed by a GPLv3 license that can be found in the LICENSE file.

#pragma once

#include <cstdint>
#include <cmath>

const int MAX_AXIS = 9;

class Axis {
public:
    float stepsPerMm   = 320.0f;   // steps/mm
    float maxRate      = 1000.0f;  // mm/min
    float acceleration = 25.0f;    // mm/min^2
    float maxTravel    = 1000.0f;  // mm
};

class Axes {
public:
    int   _numberAxis = 0;
    Axis* _axis[MAX_AXIS];

    float arcTolerance      = 0.002f;
    float junctionDeviation = 0.01f;
};

template <typename T>
struct Vector {
    T value[MAX_AXIS];

public:
    Vector() { memset(value, 0, sizeof(value)); }
    Vector(Vector&& o) { memcpy(value, o.value, sizeof(value)); }
    Vector(const Vector& o) { memcpy(value, o.value, sizeof(value)); }
    Vector& operator=(Vector&& o) {
        memcpy(value, o.value, sizeof(value));
        return *this;
    }
    Vector& operator=(const Vector& o) {
        memcpy(value, o.value, sizeof(value));
        return *this;
    }

    template <size_t N>
    Vector(const T (&values)[N]) {
        const size_t n = N < MAX_AXIS ? N : MAX_AXIS;
        memcpy(value, values, sizeof(T) * n);
        memset(value + n, 0, sizeof(T) * (MAX_AXIS - n));
    }

    T&       operator[](int index) { return value[index]; }
    const T& operator[](int index) const { return value[index]; }

    float Length() const {
        float total = 0;
        for (int i = 0; i < MAX_AXIS; ++i) {
            total += value[i] * value[i];
        }
        return float(std::sqrt(total));
    }

    T Max(int numberAxis) const {
        T maxVal = value[0];
        for (int i = 1; i < numberAxis; ++i) {
            if (value[i] > maxVal) {
                maxVal = value[i];
            }
        }
        return maxVal;
    }

    T Min(int numberAxis) const {
        T minVal = value[0];
        for (int i = 1; i < numberAxis; ++i) {
            if (value[i] < maxVal) {
                minVal = value[i];
            }
        }
        return minVal;
    }

    float Normalize(int numberAxis) {
        auto length    = Length();
        auto invLength = (length > 0.0f) ? (1.0f / length) : 0.0f;

        for (int i = 0; i < numberAxis; ++i) {
            value[i] = value[i] * invLength;
        }
        return length;
    }

    Vector<T> operator-(const Vector<T>& rhs) const {
        Vector<T> result;
        for (int i = 0; i < MAX_AXIS; ++i) {
            result[i] = (*this)[i] - rhs[i];
        }
        return result;
    }

    Vector<T> operator+(const Vector<T>& rhs) const {
        Vector<T> result;
        for (int i = 0; i < MAX_AXIS; ++i) {
            result[i] = (*this)[i] + rhs[i];
        }
        return result;
    }

    Vector<T> operator*(float scalar) const {
        Vector<T> result;
        for (int i = 0; i < MAX_AXIS; ++i) {
            result[i] = T((*this)[i] * scalar)
        }
        return result;
    }
};

struct PlannerBlock {
    // Status of the block:
    //
    // 0 = not written
    // 1 = planned, not executed
    // 2 = [partly] executed
    //
    // TODO FIXME: I'm not sure if we even need status, because we use counters. Let's find out...

    volatile uint8_t status = 0;

    bool NominalLength() const { return (status & 1) != 0; }
    void SetNominalLength(bool value) {
        if (value) {
            status |= 1;
        } else {
            status &= ~1;
        }
    }

    bool Recalculate() const { return (status & 2) != 0; }
    void SetRecalculate(bool value) {
        if (value) {
            status |= 2;
        } else {
            status &= ~2;
        }
    }

    float millimeters  = 0;  // The total travel of this block in mm
    float acceleration = 0;  // acceleration mm/sec^2
    float nominalSpeed = 0;  // the fastest allowed rate

    // Fields used by the motion planner to manage acceleration
    float nominalSpeedSqr     = 0;  // The nominal speed for this block in (mm/sec)^2
    float entrySpeedSqr       = 0;  // Entry speed at previous-current junction in (mm/sec)^2
    float maxJunctionSpeedSqr = 0;  // Maximum allowable junction entry speed in (mm/sec)^2

    // What we really need to know:
    Vector<int32_t> targetPosition;           // Step count along each axis
    uint16_t        direction           = 0;  // Direction bitmask for each axis
    uint32_t        totalStepCount      = 0;  // The number of step events required to complete this block
    uint32_t        accelerateUntilStep = 0;  // The index of the step event on which to stop acceleration
    uint32_t        decelerateAfterStep = 0;  // The index of the step event on which to start decelerating
                                              //
    uint32_t cruiseRate              = 0;     // The actual cruise rate
    uint32_t accelerationTime        = 0;     // Acceleration time and deceleration time in STEP timer counts
    uint32_t decelerationTime        = 0;     //
    uint32_t accelerationTimeInverse = 0;     // Inverse of acceleration and deceleration periods
    uint32_t decelerationTimeInverse = 0;     //
                                              //
    uint32_t nominalRate            = 0;      // The nominal step rate for this block in step_events/sec
    uint32_t initialRate            = 0;      // The jerk-adjusted step rate at start of block
    uint32_t finalRate              = 0;      // The minimal rate at exit
    uint32_t accelerationStepsPerS2 = 0;      // acceleration steps/sec^2
};

class PlannerBuffer {
public:
    static const int PlannerSize = 128;

    PlannerBlock blocks[PlannerSize];

    // If this were to be a non-cyclic buffer, the values would be:
    // currentIndex <= scheduledIndex <= writeIndex.
    //
    // A separate task attempts to convert the indices here into motion. The way it does
    // that is by grabbing blocks, and converting them to timed events. There are three
    // pointers that matter.
    // 1. CurrentIndex is the index into the block buffer of the first block that has been
    //    scheduled, but which time has not elapsed yet. The current index is written by the
    //    task, and read by the Planner.
    // 2. ScheduleIndex is the index into the block buffer of the first block that needs to
    //    be scheduled. The write index is written by the task, and read by the Planner.
    // 3. WriteIndex is the index into the buffer of the first block to write with new planner
    //    entries. The write index is written by the Planner, and read by the task.
    //
    // When we're recalculating, we're iterating [ScheduleIndex, WriteIndex>, marking these
    // blocks while we're running.

    volatile int currentIndex  = 0;
    volatile int scheduleIndex = 0;
    volatile int writeIndex    = 0;

    bool IsBlockBusy(const PlannerBlock* const block) const {
        // auto idx = block - (&(blocks[0]));
        // return idx == scheduleIndex;
        // TODO FIXME: I think...!!
        // I think atomic int's with compare_exchange is easier to handle!
        return false;
    }

    PlannerBlock& GrabWriteBlock() {
        // Can we write?
        int c = (currentIndex + PlannerBuffer::PlannerSize - 1) % PlannerBuffer::PlannerSize;
        while (writeIndex == c) {
            // yield();
        }

        return blocks[writeIndex];
    }

    void IncrementWriteIndex() {
        auto idx   = writeIndex;
        idx        = (idx + 1) % PlannerSize;
        writeIndex = idx;
    }

    bool Empty() const { return writeIndex == currentIndex; }

    int LastWriteIndex() const { return (writeIndex + PlannerSize - 1) % PlannerSize; }
};

class Planner {
    PlannerBuffer   buffer;
    Vector<float>   previousUnitVector;
    Vector<int32_t> lastPositionVector;
    float           previousNominalSpeed;
    float           previousNominalSpeedSqr;
    int             blockBufferOptimal = 0;

    const float           MINIMUM_PLANNER_SPEED = 0.05f;  // (mm/s)
    static const uint32_t MINIMAL_STEP_RATE     = 80u;
    static const uint32_t STEPPER_TIMER_RATE    = 1'000u;  // 20'000'000u;

    float LimitAccelerationByAxes(Vector<float> unitVector, const Axes& axes, float limit = 1e38f) {
        int numberAxis = axes._numberAxis;

        float maxAcceleration = limit;
        for (int idx = 0; idx < numberAxis; idx++) {
            if (unitVector[idx] != 0) {  // Avoid divide by zero.
                auto axisSetting = axes._axis[idx];
                auto newValue    = float(fabs(axisSetting->acceleration / unitVector[idx]));
                if (newValue < maxAcceleration) {
                    newValue = maxAcceleration;
                }
            }
        }

        // The acceleration setting is stored and displayed in units of mm/sec^2,
        // but used in units of mm/min^2.  It suffices to perform the conversion once on
        // exit, since the limit computation above is independent of units - it simply
        // finds the smallest value.
        return maxAcceleration * 60.0f * 60.0f;
    }

    float LimitRateByAxes(Vector<float> unitVector, const Axes& axes) {
        float maxRate    = 1e38f;
        int   numberAxis = axes._numberAxis;

        for (int idx = 0; idx < numberAxis; idx++) {
            if (unitVector[idx] != 0) {  // Avoid divide by zero.
                auto axisSetting = axes._axis[idx];
                auto newValue    = float(fabs(axisSetting->maxRate / unitVector[idx]));

                if (maxRate > newValue) {
                    maxRate = newValue;
                }
            }
        }
        return maxRate;
    }

    inline float    minf(float lhs, float rhs) { return lhs < rhs ? lhs : rhs; }
    inline float    maxf(float lhs, float rhs) { return lhs > rhs ? lhs : rhs; }
    inline uint32_t minu(uint32_t lhs, uint32_t rhs) { return lhs < rhs ? lhs : rhs; }
    inline uint32_t maxu(uint32_t lhs, uint32_t rhs) { return lhs > rhs ? lhs : rhs; }

    // The kernel called by recalculate() when scanning the plan from last to first entry.
    void ReversePassKernel(PlannerBlock* const current, const PlannerBlock* const next) {
        if (current) {
            // If entry speed is already at the maximum entry speed, and there was no change of speed
            // in the next block, there is no need to recheck. Block is cruising and there is no need to
            // compute anything for this block,
            // If not, block entry speed needs to be recalculated to ensure maximum possible planned speed.
            const float maxEntrySpeedSqr = current->maxJunctionSpeedSqr;

            // Compute maximum entry speed decelerating over the current block from its exit speed.
            // If not at the maximum entry speed, or the previous block entry speed changed
            if (current->entrySpeedSqr != maxEntrySpeedSqr || (next && next->Recalculate())) {
                // If nominal length true, max junction speed is guaranteed to be reached.
                // If a block can de/ac-celerate from nominal speed to zero within the length of the block, then
                // the current block and next block junction speeds are guaranteed to always be at their maximum
                // junction speeds in deceleration and acceleration, respectively. This is due to how the current
                // block nominal speed limits both the current and next maximum junction speeds. Hence, in both
                // the reverse and forward planners, the corresponding block junction speed will always be at the
                // the maximum junction speed and may always be ignored for any speed reduction checks.

                float newEntrySpeedSqr =
                    current->NominalLength() ?
                        maxEntrySpeedSqr :
                        minf(maxEntrySpeedSqr,
                             MaxAllowableSpeedSqr(-current->acceleration,
                                                  next ? next->entrySpeedSqr : (MINIMUM_PLANNER_SPEED * MINIMUM_PLANNER_SPEED),
                                                  current->millimeters));

                if (current->entrySpeedSqr != newEntrySpeedSqr) {
                    // Need to recalculate the block speed - Mark it now, so the stepper
                    // ISR does not consume the block before being recalculated
                    current->SetRecalculate(true);

                    // But there is an inherent race condition here, as the block may have
                    // become BUSY just before being marked RECALCULATE, so check for that!
                    if (buffer.IsBlockBusy(current)) {
                        // Block became busy. Clear the RECALCULATE flag (no point in
                        // recalculating BUSY blocks). And don't set its speed, as it can't
                        // be updated at this time.
                        current->SetRecalculate(false);
                    } else {
                        // Block is not BUSY so this is ahead of the Stepper ISR:
                        // Just Set the new entry speed.
                        current->entrySpeedSqr = newEntrySpeedSqr;
                    }
                }
            }
        }
    }

    static float MaxAllowableSpeedSqr(float accel, float targetVelocitySqr, float distance) {
        return targetVelocitySqr - 2 * accel * distance;
    }

    void ReversePass() {
        // Initialize block index to the last block in the planner buffer.
        int blockIndex = buffer.LastWriteIndex();

        // Read the index of the last buffer planned block.
        // The ISR may change it so get a stable local copy.
        int plannedBlockIndex = blockBufferOptimal;

        // If there was a race condition and buffer.scheduleIndex was incremented
        //  or was pointing at the head (queue empty) break loop now and avoid
        //  planning already consumed blocks
        if (plannedBlockIndex == buffer.writeIndex) {
            return;
        }

        // Reverse Pass: Coarsely maximize all possible deceleration curves back-planning from the last
        // block in buffer. Cease planning when the last optimal planned or tail pointer is reached.
        // NOTE: Forward pass will later refine and correct the reverse pass to create an optimal plan.
        const PlannerBlock* next = nullptr;
        while (blockIndex != plannedBlockIndex) {
            // Perform the reverse pass
            PlannerBlock* current = &(buffer.blocks[blockIndex]);

            ReversePassKernel(current, next);
            next = current;

            // Advance to the next
            blockIndex = (blockIndex + PlannerBuffer::PlannerSize - 1) % PlannerBuffer::PlannerSize;

            // The ISR could advance the buffer.scheduleIndex while we were doing the reverse pass.
            // We must try to avoid using an already consumed block as the last one - So follow
            // changes to the pointer and make sure to limit the loop to the currently busy block
            while (plannedBlockIndex != blockBufferOptimal) {
                // If we reached the busy block or an already processed block, break the loop now
                if (blockIndex == plannedBlockIndex)
                    return;

                // Advance the pointer, following the busy block
                plannedBlockIndex = (plannedBlockIndex + 1) % PlannerBuffer::PlannerSize;
            }
        }
    }

    // The kernel called by recalculate() when scanning the plan from first to last entry.
    void ForwardPassKernel(const PlannerBlock* const previous, PlannerBlock* const current, const uint8_t block_index) {
        if (previous) {
            // If the previous block is an acceleration block, too short to complete the full speed
            // change, adjust the entry speed accordingly. Entry speeds have already been reset,
            // maximized, and reverse-planned. If nominal length is set, max junction speed is
            // guaranteed to be reached. No need to recheck.
            if (!previous->NominalLength() && previous->entrySpeedSqr < current->entrySpeedSqr) {
                // Compute the maximum allowable speed
                const float newEntrySpeedSqr = MaxAllowableSpeedSqr(-previous->acceleration, previous->entrySpeedSqr, previous->millimeters);

                // If true, current block is full-acceleration and we can move the planned pointer forward.
                if (newEntrySpeedSqr < current->entrySpeedSqr) {
                    // Mark we need to recompute the trapezoidal shape, and do it now,
                    // so the stepper ISR does not consume the block before being recalculated
                    current->SetRecalculate(true);

                    // But there is an inherent race condition here, as the block maybe
                    // became BUSY, just before it was marked as RECALCULATE, so check
                    // if that is the case!
                    if (buffer.IsBlockBusy(current)) {
                        // Block became busy. Clear the RECALCULATE flag (no point in
                        //  recalculating BUSY blocks and don't set its speed, as it can't
                        //  be updated at this time.
                        current->SetRecalculate(false);
                    } else {
                        // Block is not BUSY, we won the race against the Stepper ISR:

                        // Always <= max_entry_speed_sqr. Backward pass sets this.
                        current->entrySpeedSqr = newEntrySpeedSqr;  // Always <= max_entry_speed_sqr. Backward pass sets this.

                        // Set optimal plan pointer.
                        blockBufferOptimal = block_index;
                    }
                }
            }

            // Any block set at its maximum entry speed also creates an optimal plan up to this
            // point in the buffer. When the plan is bracketed by either the beginning of the
            // buffer and a maximum entry speed or two maximum entry speeds, every block in between
            // cannot logically be further improved. Hence, we don't have to recompute them anymore.
            if (current->entrySpeedSqr == current->maxJunctionSpeedSqr) {
                blockBufferOptimal = block_index;
            }
        }
    }

    void ForwardPass() {
        // Forward Pass: Forward plan the acceleration curve from the planned pointer onward.
        // Also scans for optimal plan breakpoints and appropriately updates the planned pointer.
        //
        // Begin at buffer planned pointer. Note that buffer.scheduleIndex can be modified
        // by the stepper ISR, so read it ONCE. It it guaranteed that buffer.scheduleIndex
        // will never lead head, so the loop is safe to execute. Also note that the forward
        // pass will never modify the values at the tail.
        auto block_index = blockBufferOptimal;

        PlannerBlock*       block;
        const PlannerBlock* previous = nullptr;
        while (block_index != buffer.writeIndex) {
            // Perform the forward pass
            block = &(buffer.blocks[block_index]);

            // Skip SYNC and page blocks
            //if (!(block->status & BLOCK_MASK_SYNC) && !IS_PAGE(block)) {
            // If there's no previous block or the previous block is not
            // BUSY (thus, modifiable) run the forward_pass_kernel. Otherwise,
            // the previous block became BUSY, so assume the current block's
            // entry speed can't be altered (since that would also require
            // updating the exit speed of the previous block).
            if (!previous || !buffer.IsBlockBusy(previous)) {
                ForwardPassKernel(previous, block, block_index);
            }
            previous = block;
            //}

            // Advance to the previous
            block_index = (block_index + 1) % PlannerBuffer::PlannerSize;
        }
    }

    /**
     * Recalculate the trapezoid speed profiles for all blocks in the plan
     * according to the entry_factor for each junction. Must be called by
     * recalculate() after updating the blocks.
     */
    void RecalculateZoids() {
        // The tail may be changed by the ISR so get a local copy.
        int blockIndex     = buffer.scheduleIndex;
        int headBlockIndex = buffer.writeIndex;

        // Go from the tail (currently executed block) to the first block, without including it)
        PlannerBlock* block             = nullptr;
        PlannerBlock* next              = nullptr;
        float         currentEntrySpeed = 0.0f;
        float         nextEntrySpeed    = 0.0f;
        while (blockIndex != headBlockIndex) {
            next = &(buffer.blocks[blockIndex]);

            nextEntrySpeed = std::sqrt(next->entrySpeedSqr);

            if (block) {
                // Recalculate if current block entry or exit junction speed has changed.
                if (block->Recalculate() || next->Recalculate()) {
                    // Mark the current block as RECALCULATE, to protect it from the Stepper ISR running it.
                    // Note that due to the above condition, there's a chance the current block isn't marked as
                    // RECALCULATE yet, but the next one is. That's the reason for the following line.
                    block->SetRecalculate(true);

                    // But there is an inherent race condition here, as the block maybe
                    // became BUSY, just before it was marked as RECALCULATE, so check
                    // if that is the case!
                    if (!buffer.IsBlockBusy(block)) {
                        // Block is not BUSY, we won the race against the Stepper ISR:

                        // NOTE: Entry and exit factors always > 0 by all previous logic operations.
                        float current_nominal_speed = std::sqrt(block->nominalSpeedSqr);
                        float nomr                  = 1.0f / current_nominal_speed;
                        CalculateZoidForBlock(block, currentEntrySpeed * nomr, nextEntrySpeed * nomr);
                    }

                    // Reset current only to ensure next trapezoid is computed - The
                    // stepper is free to use the block from now on.
                    block->SetRecalculate(false);
                }
            }

            block             = next;
            currentEntrySpeed = nextEntrySpeed;

            blockIndex = (blockIndex + 1) % PlannerBuffer::PlannerSize;
        }

        // Last/newest block in buffer. Exit speed is set with MINIMUM_PLANNER_SPEED. Always recalculated.
        if (next) {
            // Mark the next(last) block as RECALCULATE, to prevent the Stepper ISR running it.
            // As the last block is always recalculated here, there is a chance the block isn't
            // marked as RECALCULATE yet. That's the reason for the following line.
            next->SetRecalculate(true);

            // But there is an inherent race condition here, as the block maybe
            // became BUSY, just before it was marked as RECALCULATE, so check
            // if that is the case!
            if (!buffer.IsBlockBusy(block)) {
                // Block is not BUSY, we won the race against the Stepper ISR:

                const float next_nominal_speed = std::sqrt(next->nominalSpeedSqr), nomr = 1.0f / next_nominal_speed;
                CalculateZoidForBlock(next, nextEntrySpeed * nomr, float(MINIMUM_PLANNER_SPEED) * nomr);
            }

            // Reset next only to ensure its trapezoid is computed - The stepper is free to use
            // the block from now on.
            next->SetRecalculate(false);
        }
    }

    /**
     * Calculate the distance (not time) it takes to accelerate
     * from initial_rate to target_rate using the given acceleration:
     */
    static float estimate_acceleration_distance(uint32_t initialRate, uint32_t targetRate, int32_t acceleration) {
        if (acceleration == 0) {
            return 0;  // accel was 0, set acceleration distance to 0
        } else {
            return (float(targetRate * targetRate) - float(initialRate * initialRate)) / float(acceleration * 2);
        }
    }

    /**
     * Return the point at which you must start braking (at the rate of -'accel') if
     * you start at 'initial_rate', accelerate (until reaching the point), and want to end at
     * 'final_rate' after traveling 'distance'.
     *
     * This is used to compute the intersection point between acceleration and deceleration
     * in cases where the "trapezoid" has no plateau (i.e., never reaches maximum speed)
     */
    static float intersection_distance(uint32_t initialRate, uint32_t finalRate, uint32_t acceleration, uint32_t distance) {
        if (acceleration == 0) {
            return 0;  // accel was 0, set intersection distance to 0
        } else {
            return (acceleration * 2 * distance - float(initialRate * initialRate) + float(finalRate * finalRate)) / (acceleration * 4);
        }
    }

    /**
     * Calculate trapezoid parameters, multiplying the entry- and exit-speeds
     * by the provided factors.
     */
    void Planner::CalculateZoidForBlock(PlannerBlock* const block, float entryFactor, float exitFactor) {
        uint32_t initialRate = uint32_t(std::ceil(block->nominalRate * entryFactor));
        uint32_t finalRate   = uint32_t(std::ceil(block->nominalRate * exitFactor));  // (steps per second)

        // Limit minimal step rate (Otherwise the timer will overflow.)
        if (initialRate < MINIMAL_STEP_RATE) {
            initialRate = MINIMAL_STEP_RATE;
        }
        if (finalRate < MINIMAL_STEP_RATE) {
            finalRate = MINIMAL_STEP_RATE;
        }

        uint32_t cruiseRate = initialRate;
        uint32_t accel      = block->accelerationStepsPerS2;

        // Steps required for acceleration, deceleration to/from nominal rate
        uint32_t accelerateSteps = uint32_t(std::ceil(estimate_acceleration_distance(initialRate, block->nominalRate, int32_t(accel))));
        uint32_t decelerateSteps = uint32_t(std::floor(estimate_acceleration_distance(block->nominalRate, finalRate, -int32_t(accel))));

        // Steps between acceleration and deceleration, if any
        int32_t plateauSteps = block->totalStepCount - accelerateSteps - decelerateSteps;

        // Does accelerate_steps + decelerate_steps exceed step_event_count?
        // Then we can't possibly reach the nominal rate, there will be no cruising.
        // Use intersection_distance() to calculate accel / braking time in order to
        // reach the final_rate exactly at the end of this block.
        if (plateauSteps < 0) {
            float accelerate_steps_float = std::ceil(intersection_distance(initialRate, finalRate, accel, block->totalStepCount));
            accelerateSteps              = minu(uint32_t(maxf(accelerate_steps_float, 0)), block->totalStepCount);
            plateauSteps                 = 0;

            // We won't reach the cruising rate. Let's calculate the speed we will reach
            cruiseRate = FinalSpeed(initialRate, accel, accelerateSteps);
        } else  // We have some plateau time, so the cruise rate will be the nominal rate
        {
            cruiseRate = block->nominalRate;
        }

        // Jerk controlled speed requires to express speed versus time, NOT steps
        uint32_t accelerationTime = uint32_t((float(cruiseRate - initialRate) / accel) * STEPPER_TIMER_RATE);
        uint32_t decelerationTime = uint32_t((float(cruiseRate - finalRate) / accel) * STEPPER_TIMER_RATE);

        // And to offload calculations from the ISR, we also calculate the inverse of those times here
        uint32_t accelerationTimeInverse = GetPeriodInverse(accelerationTime);
        uint32_t decelerationTimeInverse = GetPeriodInverse(decelerationTime);

        // Store new block parameters
        block->accelerateUntilStep     = accelerateSteps;
        block->decelerateAfterStep     = accelerateSteps + plateauSteps;
        block->initialRate             = initialRate;
        block->accelerationTime        = accelerationTime;
        block->decelerationTime        = decelerationTime;
        block->accelerationTimeInverse = accelerationTimeInverse;
        block->decelerationTimeInverse = decelerationTimeInverse;
        block->cruiseRate              = cruiseRate;
        block->finalRate               = finalRate;
    }

    static uint32_t FinalSpeed(uint32_t initial_velocity, uint32_t accel, uint32_t distance) {
        return uint32_t(std::sqrt(float(initial_velocity) * float(initial_velocity) + float(2 * accel) * float(distance)));
    }
    // This routine, for all other archs, returns 0x100000000 / d ~= 0xFFFFFFFF / d
    static uint32_t GetPeriodInverse(const uint32_t d) { return d ? 0xFFFFFFFF / d : 0xFFFFFFFF; }

    void Recalculate() {
        // Initialize block index to the last block in the planner buffer.
        int blockIndex = buffer.LastWriteIndex();

        // If there is just one block, no planning can be done.
        if (blockIndex != blockBufferOptimal) {
            ReversePass();
            ForwardPass();
        }
        RecalculateZoids();
    }

public:
    void Add(const Vector<float>& targetPosition, float feedRate, Axes& config) {
        // TODO: Kinematics!

        auto numberAxis = config._numberAxis;

        auto& last  = lastPositionVector;
        auto& block = buffer.GrabWriteBlock();

        Vector<int32_t>& targetPositionSteps = block.targetPosition;
        Vector<float>    unitVector;
        Vector<uint32_t> stepsPerAxis;

        uint16_t directionVector = 0;  // A bitmask holding directions for all axis.
        int32_t  maxNumberSteps  = 0;  // This is the number of steps for the dominant axis.

        float totalLengthSqr = 0;

        for (int i = 0; i < numberAxis; ++i) {
            auto axis = config._axis[i];

            targetPositionSteps[i] = int32_t(targetPosition[i] * axis->stepsPerMm);

            // Max rate depends on the max rate per axis:
            auto deltaSteps = last[i] - targetPositionSteps[i];

            auto d = deltaSteps / axis->stepsPerMm;
            totalLengthSqr += d * d;

            // Update the direction:
            // TODO: If deltaSteps == 0, we can just use the old value and don't have to swap directions.
            directionVector |= uint16_t(int(deltaSteps < 0) << i);

            if (deltaSteps < 0) {
                deltaSteps = -deltaSteps;
            }

            if (deltaSteps > maxNumberSteps) {
                maxNumberSteps = deltaSteps;
            }
            stepsPerAxis[i] = deltaSteps;

            unitVector[i] = deltaSteps;
        }

        // No-op?
        if (maxNumberSteps == 0) {
            return;
        }

        // Changes the unit vector to a normalized vector and returns the original length.
        unitVector.Normalize(numberAxis);
        float lengthInMm    = std::sqrt(totalLengthSqr);
        float invLengthInMm = 1.0f / lengthInMm;  // Inverse millimeters to remove multiple divides

        block.status         = 0;
        block.direction      = directionVector;
        block.totalStepCount = maxNumberSteps;
        block.millimeters    = lengthInMm;
        block.acceleration   = LimitAccelerationByAxes(unitVector, config);

        // Nominal rate can never exceed rapid rate
        block.nominalSpeed = LimitRateByAxes(unitVector, config);
        block.nominalSpeed = (block.nominalSpeed > feedRate) ? feedRate : block.nominalSpeed;

        // TODO FIXME: Add speed overrides *HERE* for nominal speed!

        const float MINIMUM_SPEED_RATE = 1.0f;
        if (block.nominalSpeed < MINIMUM_SPEED_RATE) {
            block.nominalSpeed = MINIMUM_SPEED_RATE;
        }

        float nominalSpeed = block.nominalSpeed;

        // Calculate inverse time for this move. No divide by zero due to previous checks.
        // Example: At 120mm/s a 60mm move takes 0.5s. So this will give 2.0.
        float inverse_secs = nominalSpeed * invLengthInMm;

        block.nominalSpeedSqr = (block.millimeters * inverse_secs) * (block.millimeters * inverse_secs);  // (mm/sec)^2 Always > 0
        block.nominalRate     = uint32_t(std::ceil(block.totalStepCount * inverse_secs));                 // (step/sec) Always > 0

        // Calculate acceleration

        // Compute and limit the acceleration rate for the trapezoid generator.
        float    stepsPerMm   = block.totalStepCount * invLengthInMm;
        uint32_t acceleration = UINT_MAX;

        for (int i = 0; i < numberAxis; ++i) {
            auto axisAcceleration = config._axis[i]->acceleration;
            if (stepsPerAxis[i] && axisAcceleration < acceleration) {
                uint32_t max_possible = uint32_t((axisAcceleration * block.totalStepCount) / stepsPerAxis[i]);
                acceleration          = minu(acceleration, max_possible);
            }
        }

        block.accelerationStepsPerS2 = acceleration;
        block.acceleration           = acceleration / stepsPerMm;

        float vMaxJunctionSqr;  // Initial limit on the segment entry velocity (mm/s)^2

        /**
         * Compute maximum allowable entry speed at junction by centripetal acceleration approximation.
         * Let a circle be tangent to both previous and current path line segments, where the junction
         * deviation is defined as the distance from the junction to the closest edge of the circle,
         * colinear with the circle center. The circular segment joining the two paths represents the
         * path of centripetal acceleration. Solve for max velocity based on max acceleration about the
         * radius of the circle, defined indirectly by junction deviation. This may be also viewed as
         * path width or max_jerk in the previous Grbl version. This approach does not actually deviate
         * from path, but used as a robust way to compute cornering speeds, as it takes into account the
         * nonlinearities of both the junction angle and junction velocity.
         *
         * NOTE: If the junction deviation value is finite, Grbl executes the motions in an exact path
         * mode (G61). If the junction deviation value is zero, Grbl will execute the motion in an exact
         * stop mode (G61.1) manner. In the future, if continuous mode (G64) is desired, the math here
         * is exactly the same. Instead of motioning all the way to junction point, the machine will
         * just follow the arc circle defined here. The Arduino doesn't have the CPU cycles to perform
         * a continuous mode path, but ARM-based microcontrollers most certainly do.
         *
         * NOTE: The max junction speed is a fixed value, since machine acceleration limits cannot be
         * changed dynamically during operation nor can the line move geometry. This must be kept in
         * memory in the event of a feedrate override changing the nominal speeds of blocks, which can
         * change the overall maximum entry speed conditions of all blocks.
         *
         * #######
         * https://github.com/MarlinFirmware/Marlin/issues/10341#issuecomment-388191754
         *
         * hoffbaked: on May 10 2018 tuned and improved the GRBL algorithm for Marlin:
              Okay! It seems to be working good. I somewhat arbitrarily cut it off at 1mm
              on then on anything with less sides than an octagon. With this, and the
              reverse pass actually recalculating things, a corner acceleration value
              of 1000 junction deviation of .05 are pretty reasonable. If the cycles
              can be spared, a better acos could be used. For all I know, it may be
              already calculated in a different place. */

        // Skip first block or when previous_nominal_speed is used as a flag for homing and offset cycles.
        if (buffer.scheduleIndex != buffer.writeIndex && previousNominalSpeedSqr >= 0.000001f) {
            // Compute cosine of angle between previous and current path. (prev_unit_vec is negative)
            // NOTE: Max junction velocity is computed without sin() or acos() by trig half angle identity.
            float junctionCosTheta = 0;
            for (int i = 0; i < numberAxis; ++i) {
                junctionCosTheta -= previousUnitVector[i] * unitVector[i];
            }

            const float junction_deviation_mm = 0.013f;  // (mm) Distance from real junction edge

            // NOTE: Computed without any expensive trig, sin() or acos(), by trig half angle identity of cos(theta).
            if (junctionCosTheta > 0.999999f) {
                // For a 0 degree acute junction, just set minimum junction speed.
                vMaxJunctionSqr = MINIMUM_PLANNER_SPEED * MINIMUM_PLANNER_SPEED;
            } else {
                junctionCosTheta = maxf(-0.999999f, junctionCosTheta);  // Check for numerical round-off to avoid divide by zero.

                // Convert delta vector to unit vector
                Vector<float> junction_unit_vec = unitVector - previousUnitVector;
                junction_unit_vec.Normalize(numberAxis);

                float junctionAcceleration = LimitAccelerationByAxes(junction_unit_vec, config, block.acceleration);
                float sin_theta_d2         = std::sqrt(0.5f * (1.0f - junctionCosTheta));  // Trig half angle identity. Always positive.

                vMaxJunctionSqr = junctionAcceleration * junction_deviation_mm * sin_theta_d2 / (1.0f - sin_theta_d2);

                // For small moves with >135° junction (octagon) find speed for approximate arc
                if (block.millimeters < 1 && junctionCosTheta < -0.7071067812f) {
                    // Fast acos(-t) approximation (max. error +-0.033rad = 1.89°)
                    // Based on MinMax polynomial published by W. Randolph Franklin, see
                    // https://wrf.ecse.rpi.edu/Research/Short_Notes/arcsin/onlyelem.html
                    //  acos( t) = pi / 2 - asin(x)
                    //  acos(-t) = pi - acos(t) ... pi / 2 + asin(x)

                    const float neg = junctionCosTheta < 0.0f ? -1.0f : 1.0f;

                    float t = neg * junctionCosTheta,
                          asinx =
                              0.032843707f +
                              t * (-1.451838349f +
                                   t * (29.66153956f + t * (-131.1123477f + t * (262.8130562f + t * (-242.7199627f + t * (84.31466202f)))))),
                          junction_theta = (90 * 3.14159265358979f / 180.0f) + neg * asinx;  // acos(-t)

                    // NOTE: junction_theta bottoms out at 0.033 which avoids divide by 0.

                    float limit_sqr = (block.millimeters * junctionAcceleration) / junction_theta;
                    vMaxJunctionSqr = minf(vMaxJunctionSqr, limit_sqr);
                }
            }

            // Get the lowest speed
            vMaxJunctionSqr = minf(minf(vMaxJunctionSqr, block.nominalSpeedSqr), previousNominalSpeedSqr);
        } else  // Init entry speed to zero. Assume it starts from rest. Planner will correct this later.
            vMaxJunctionSqr = 0;

        // Max entry speed of this block equals the max exit speed of the previous block.
        block.maxJunctionSpeedSqr = vMaxJunctionSqr;

        // Initialize block entry speed. Compute based on deceleration to user-defined MINIMUM_PLANNER_SPEED.
        float v_allowable_sqr = MaxAllowableSpeedSqr(-block.acceleration, MINIMUM_PLANNER_SPEED * MINIMUM_PLANNER_SPEED, block.millimeters);

        // If we are trying to add a split block, start with the
        // max. allowed speed to avoid an interrupted first move.
        block.entrySpeedSqr = (MINIMUM_PLANNER_SPEED * MINIMUM_PLANNER_SPEED) /* minf(vMaxJunctionSqr, v_allowable_sqr) */;

        // Initialize planner efficiency flags
        // Set flag if block will always reach maximum junction speed regardless of entry/exit speeds.
        // If a block can de/ac-celerate from nominal speed to zero within the length of the block, then
        // the current block and next block junction speeds are guaranteed to always be at their maximum
        // junction speeds in deceleration and acceleration, respectively. This is due to how the current
        // block nominal speed limits both the current and next maximum junction speeds. Hence, in both
        // the reverse and forward planners, the corresponding block junction speed will always be at the
        // the maximum junction speed and may always be ignored for any speed reduction checks.
        if (block.nominalSpeedSqr <= v_allowable_sqr) {
            block.SetNominalLength(true);
        }
        block.SetRecalculate(true);

        // Update previous info for next 'add' call.
        this->previousUnitVector      = unitVector;
        this->lastPositionVector      = targetPositionSteps;
        this->previousNominalSpeed    = nominalSpeed;
        this->previousNominalSpeedSqr = block.nominalSpeedSqr;

        buffer.IncrementWriteIndex();

        // Finish up by recalculating the plan with the new block.
        Recalculate();
    }
};
