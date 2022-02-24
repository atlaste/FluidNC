// Copyright (c) 2022 -	Stefan de Bruijn
// Use of this source code is governed by a GPLv3 license that can be found in the LICENSE file.

#pragma once

#include <cstdint>
#include <cmath>

const size_t MAX_AXIS = 9;

class Axes {
public:
    int   _numberAxis = 0;
    Axis* _axis[MAX_AXIS];

    float _arcTolerance      = 0.002f;
    float _junctionDeviation = 0.01f;
};

class Axis {
public:
    float stepsPerMm   = 320.0f;   // steps/mm
    float maxRate      = 1000.0f;  // mm/min
    float acceleration = 25.0f;    // mm/min^2
    float maxTravel    = 1000.0f;  // mm
};

template <typename T>
struct Vector {
    T value[MAX_AXIS];

public:
    Vector() { memset(value, 0, sizeof(value)); }
    Vector(Vector&& o) { memcpy(value, o.value, sizeof(value)); }
    Vector(const Vector& o) { memcpy(value, o.value, sizeof(value)); }
    Vector& operator=(Vector&& o) { memcpy(value, o.value, sizeof(value)); }
    Vector& operator=(const Vector& o) { memcpy(value, o.value, sizeof(value)); }

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
        auto invLength = (invLength > 0.0f) ? (1.0f / length) : 0.0f;

        for (int i = 0; i < numberAxis; ++i) {
            value[i] = value[i] * invLength;
        }
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
    static const size_t PlannerSize = 128;

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

    volatile size_t currentIndex  = 0;
    volatile size_t scheduleIndex = 0;
    volatile size_t writeIndex    = 0;

    bool IsBlockBusy(const PlannerBlock* const block) const {
        auto idx = block - (&(blocks[0]));
        return idx == scheduleIndex; 
        // TODO FIXME: I think...!! 
        // I think atomic int's with compare_exchange is easier to handle!
    }

    PlannerBlock& GrabWriteBlock() {
        // Can we write?
        while (writeIndex != currentIndex) {
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

    size_t LastWriteIndex() const { return (writeIndex + PlannerSize - 1) % PlannerSize; }
};

class Planner {
    PlannerBuffer   buffer;
    Vector<float>   previousUnitVector;
    Vector<int32_t> lastPositionVector;
    float           previousNominalSpeed;

    const float           MINIMUM_PLANNER_SPEED = 0.05f;  // (mm/s)
    static const uint32_t MINIMAL_STEP_RATE     = 80u;
    static const uint32_t STEPPER_TIMER_RATE    = 20'000'000u;

    float LimitAccelerationByAxes(Vector<float> unitVector, const Axes& axes) {
        int numberAxis = axes._numberAxis;

        float maxAcceleration = 1e38f;
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
            const float max_entry_speed_sqr = current->maxJunctionSpeedSqr;

            // Compute maximum entry speed decelerating over the current block from its exit speed.
            // If not at the maximum entry speed, or the previous block entry speed changed
            if (current->entrySpeedSqr != max_entry_speed_sqr || (next && next->Recalculate())) {
                // If nominal length true, max junction speed is guaranteed to be reached.
                // If a block can de/ac-celerate from nominal speed to zero within the length of the block, then
                // the current block and next block junction speeds are guaranteed to always be at their maximum
                // junction speeds in deceleration and acceleration, respectively. This is due to how the current
                // block nominal speed limits both the current and next maximum junction speeds. Hence, in both
                // the reverse and forward planners, the corresponding block junction speed will always be at the
                // the maximum junction speed and may always be ignored for any speed reduction checks.

                float new_entry_speed_sqr =
                    current->NominalLength() ?
                        max_entry_speed_sqr :
                        minf(max_entry_speed_sqr,
                             MaxAllowableSpeedSqr(-current->acceleration,
                                                  next ? next->entrySpeedSqr : (MINIMUM_PLANNER_SPEED * MINIMUM_PLANNER_SPEED),
                                                  current->millimeters));

                if (current->entrySpeedSqr != new_entry_speed_sqr) {
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
                        current->entrySpeedSqr = new_entry_speed_sqr;
                    }
                }
            }
        }
    }

    static float MaxAllowableSpeedSqr(float accel, float target_velocity_sqr, float distance) {
        return target_velocity_sqr - 2 * accel * distance;
    }

    void ReversePass() {
        // Initialize block index to the last block in the planner buffer.
        int block_index = buffer.LastWriteIndex();

        // Read the index of the last buffer planned block.
        // The ISR may change it so get a stable local copy.
        uint8_t planned_block_index = buffer.scheduleIndex;

        // If there was a race condition and buffer.scheduleIndex was incremented
        //  or was pointing at the head (queue empty) break loop now and avoid
        //  planning already consumed blocks
        if (planned_block_index == buffer.writeIndex)
            return;

        // Reverse Pass: Coarsely maximize all possible deceleration curves back-planning from the last
        // block in buffer. Cease planning when the last optimal planned or tail pointer is reached.
        // NOTE: Forward pass will later refine and correct the reverse pass to create an optimal plan.
        const PlannerBlock* next = nullptr;
        while (block_index != planned_block_index) {
            // Perform the reverse pass
            PlannerBlock* current = &(buffer.blocks[block_index]);

            // Only consider non sync-and-page blocks
            // if (!(current->status & BLOCK_MASK_SYNC) && !IS_PAGE(current)) {
            ReversePassKernel(current, next);
            next = current;
            // }

            // Advance to the next
            block_index = (block_index + PlannerBuffer::PlannerSize - 1) % PlannerBuffer::PlannerSize;

            // The ISR could advance the buffer.scheduleIndex while we were doing the reverse pass.
            // We must try to avoid using an already consumed block as the last one - So follow
            // changes to the pointer and make sure to limit the loop to the currently busy block
            while (planned_block_index != buffer.scheduleIndex) {
                // If we reached the busy block or an already processed block, break the loop now
                if (block_index == planned_block_index)
                    return;

                // Advance the pointer, following the busy block
                planned_block_index = (planned_block_index + 1) % PlannerBuffer::PlannerSize;
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
                const float new_entry_speed_sqr =
                    MaxAllowableSpeedSqr(-previous->acceleration, previous->entrySpeedSqr, previous->millimeters);

                // If true, current block is full-acceleration and we can move the planned pointer forward.
                if (new_entry_speed_sqr < current->entrySpeedSqr) {
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
                        current->entrySpeedSqr = new_entry_speed_sqr;  // Always <= max_entry_speed_sqr. Backward pass sets this.

                        // Set optimal plan pointer.
                        block_buffer_planned = block_index;
                    }
                }
            }

            // Any block set at its maximum entry speed also creates an optimal plan up to this
            // point in the buffer. When the plan is bracketed by either the beginning of the
            // buffer and a maximum entry speed or two maximum entry speeds, every block in between
            // cannot logically be further improved. Hence, we don't have to recompute them anymore.
            if (current->entrySpeedSqr == current->maxJunctionSpeedSqr) {
                block_buffer_planned = block_index;
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
        auto block_index = buffer.scheduleIndex;

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
    void recalculate_trapezoids() {
        // The tail may be changed by the ISR so get a local copy.
        int block_index      = buffer.writeIndex;
        int head_block_index = buffer.scheduleIndex;

        // Since there could be a sync block in the head of the queue, and the
        // next loop must not recalculate the head block (as it needs to be
        // specially handled), scan backwards to the first non-SYNC block.
        while (head_block_index != block_index) {
            // Go back (head always point to the first free block)
            int prev_index = (head_block_index + PlannerBuffer::PlannerSize - 1) % PlannerBuffer::PlannerSize;

            // Get the pointer to the block
            PlannerBlock* prev = &(buffer.blocks[prev_index]);

            // If not dealing with a sync block, we are done. The last block is not a SYNC block
            if (!(prev->status & BLOCK_MASK_SYNC))
                break;

            // Examine the previous block. This and all following are SYNC blocks
            head_block_index = prev_index;
        }

        // Go from the tail (currently executed block) to the first block, without including it)
        PlannerBlock* block               = nullptr;
        PlannerBlock* next                = nullptr;
        float         current_entry_speed = 0.0, next_entry_speed = 0.0;
        while (block_index != head_block_index) {
            next = &(buffer.blocks[block_index]);

            // Skip sync and page blocks
            // if (!(next->status & BLOCK_MASK_SYNC) && !IS_PAGE(next)) {
            next_entry_speed = std::sqrt(next->entrySpeedSqr);

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
                        calculate_trapezoid_for_block(block, current_entry_speed * nomr, next_entry_speed * nomr);
                    }

                    // Reset current only to ensure next trapezoid is computed - The
                    // stepper is free to use the block from now on.
                    block->SetRecalculate(false);
                }
            }

            block               = next;
            current_entry_speed = next_entry_speed;
            // }

            block_index = (block_index + 1) % PlannerBuffer::PlannerSize;
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
                calculate_trapezoid_for_block(next, next_entry_speed * nomr, float(MINIMUM_PLANNER_SPEED) * nomr);
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
    static float estimate_acceleration_distance(float initial_rate, float target_rate, float accel) {
        if (accel == 0)
            return 0;  // accel was 0, set acceleration distance to 0
        return ((target_rate * target_rate) - (initial_rate * initial_rate)) / (accel * 2);
    }

    /**
     * Return the point at which you must start braking (at the rate of -'accel') if
     * you start at 'initial_rate', accelerate (until reaching the point), and want to end at
     * 'final_rate' after traveling 'distance'.
     *
     * This is used to compute the intersection point between acceleration and deceleration
     * in cases where the "trapezoid" has no plateau (i.e., never reaches maximum speed)
     */
    static float intersection_distance(float initial_rate, float final_rate, float accel, float distance) {
        if (accel == 0)
            return 0;  // accel was 0, set intersection distance to 0
        return (accel * 2 * distance - (initial_rate * initial_rate) + (final_rate * final_rate)) / (accel * 4);
    }

    /**
 * Calculate trapezoid parameters, multiplying the entry- and exit-speeds
 * by the provided factors.
 **
 * ############ VERY IMPORTANT ############
 * NOTE that the PRECONDITION to call this function is that the block is
 * NOT BUSY and it is marked as RECALCULATE. That WARRANTIES the Stepper ISR
 * is not and will not use the block while we modify it, so it is safe to
 * alter its values.
 */
    void Planner::calculate_trapezoid_for_block(PlannerBlock* const block, float entry_factor, float exit_factor) {
        uint32_t initial_rate = std::ceil(block->nominalRate * entry_factor);
        uint32_t final_rate   = std::ceil(block->nominalRate * exit_factor);  // (steps per second)

        // Limit minimal step rate (Otherwise the timer will overflow.)
        if (initial_rate < MINIMAL_STEP_RATE) {
            initial_rate = MINIMAL_STEP_RATE;
        }
        if (final_rate < MINIMAL_STEP_RATE) {
            final_rate = MINIMAL_STEP_RATE;
        }

        uint32_t      cruise_rate = initial_rate;
        const int32_t accel       = block->accelerationStepsPerS2;

        // Steps required for acceleration, deceleration to/from nominal rate
        uint32_t accelerate_steps = ceil(estimate_acceleration_distance(initial_rate, block->nominalRate, accel));
        uint32_t decelerate_steps = floor(estimate_acceleration_distance(block->nominalRate, final_rate, -accel));

        // Steps between acceleration and deceleration, if any
        int32_t plateau_steps = block->totalStepCount - accelerate_steps - decelerate_steps;

        // Does accelerate_steps + decelerate_steps exceed step_event_count?
        // Then we can't possibly reach the nominal rate, there will be no cruising.
        // Use intersection_distance() to calculate accel / braking time in order to
        // reach the final_rate exactly at the end of this block.
        if (plateau_steps < 0) {
            float accelerate_steps_float = ceil(intersection_distance(initial_rate, final_rate, accel, block->totalStepCount));
            accelerate_steps             = minu(uint32_t(maxf(accelerate_steps_float, 0)), block->totalStepCount);
            plateau_steps                = 0;

            // We won't reach the cruising rate. Let's calculate the speed we will reach
            cruise_rate = final_speed(initial_rate, accel, accelerate_steps);
        } else  // We have some plateau time, so the cruise rate will be the nominal rate
            cruise_rate = block->nominalRate;

        // Jerk controlled speed requires to express speed versus time, NOT steps
        uint32_t acceleration_time = ((float)(cruise_rate - initial_rate) / accel) * (STEPPER_TIMER_RATE);
        uint32_t deceleration_time = ((float)(cruise_rate - final_rate) / accel) * (STEPPER_TIMER_RATE);

        // And to offload calculations from the ISR, we also calculate the inverse of those times here
        uint32_t acceleration_time_inverse = get_period_inverse(acceleration_time);
        uint32_t deceleration_time_inverse = get_period_inverse(deceleration_time);

        // Store new block parameters
        block->accelerateUntilStep     = accelerate_steps;
        block->decelerateAfterStep     = accelerate_steps + plateau_steps;
        block->initialRate             = initial_rate;
        block->accelerationTime        = acceleration_time;
        block->decelerationTime        = deceleration_time;
        block->accelerationTimeInverse = acceleration_time_inverse;
        block->decelerationTimeInverse = deceleration_time_inverse;
        block->cruiseRate              = cruise_rate;
        block->finalRate               = final_rate;
    }

    static float final_speed(float initial_velocity, float accel, float distance) {
        return std::sqrt((initial_velocity * initial_velocity) + 2 * accel * distance);
    }
    // This routine, for all other archs, returns 0x100000000 / d ~= 0xFFFFFFFF / d
    static uint32_t get_period_inverse(const uint32_t d) { return d ? 0xFFFFFFFF / d : 0xFFFFFFFF; }

    void Recalculate() {
        // Initialize block index to the last block in the planner buffer.
        int blockIndex    = buffer.LastWriteIndex();
        int scheduleIndex = buffer.scheduleIndex;

        // If there is just one block, no planning can be done.
        if (blockIndex != scheduleIndex) {
            ReversePass();
            ForwardPass();
        }
        recalculate_trapezoids();
    }

public:
    void Add(Vector<float> targetPosition, float feedRate, Axes& config) {
        // TODO: Kinematics!

        auto numberAxis = config._numberAxis;

        auto& last  = lastPositionVector;
        auto& block = buffer.GrabWriteBlock();

        Vector<int32_t>& targetPositionSteps = block.targetPosition;
        Vector<float>    unitVector;

        uint16_t directionVector = 0;  // A bitmask holding directions for all axis.
        uint32_t maxNumberSteps  = 0;  // This is the number of steps for the dominant axis.

        for (int i = 0; i < numberAxis; ++i) {
            auto axis = config._axis[i];

            targetPositionSteps[i] = targetPosition[i] * axis->stepsPerMm;

            // Max rate depends on the max rate per axis:
            auto deltaSteps = last[i] - targetPositionSteps[i];
            if (deltaSteps > maxNumberSteps) {
                maxNumberSteps = deltaSteps;
            }

            // Update the direction:
            directionVector |= uint16_t(int(deltaSteps < 0) << i);
            // TODO: If deltaSteps == 0, we can just use the old value and don't have to swap directions.

            auto delta    = deltaSteps * axis->stepsPerMm;
            unitVector[i] = delta;
        }

        // No-op?
        if (maxNumberSteps == 0) {
            return;
        }

        // Changes the unit vector to a normalized vector and returns the original length.
        float lengthInMm = unitVector.Normalize(numberAxis);

        block.millimeters  = lengthInMm;
        block.acceleration = LimitAccelerationByAxes(unitVector, config);
        block.nominalSpeed = LimitRateByAxes(unitVector, config);

        // Nominal rate can never exceed rapid rate
        block.nominalSpeed = (block.nominalSpeed > feedRate) ? feedRate : block.nominalSpeed;

        // TODO FIXME: Add speed overrides *HERE* for nominal speed!

        const float MINIMUM_SPEED_RATE = 1.0f;
        if (block.nominalSpeed < MINIMUM_SPEED_RATE) {
            block.nominalSpeed = MINIMUM_SPEED_RATE;
        }

        // TODO: Need to check this method handling zero junction speeds when starting from rest.
        if (buffer.Empty() /*|| (block.systemMotion) TODO FIXME!*/) {
            // Initialize block entry speed as zero. Assume it will be starting from rest. Planner will correct this later.
            // If system motion, the system motion block always is assumed to start from rest and end at a complete stop.
            block.entrySpeedSqr       = 0.0f;
            block.maxJunctionSpeedSqr = 0.0f;  // Starting from rest. Enforce start from zero velocity.
        } else {
            // Compute maximum allowable entry speed at junction by centripetal acceleration approximation.
            // Let a circle be tangent to both previous and current path line segments, where the junction
            // deviation is defined as the distance from the junction to the closest edge of the circle,
            // colinear with the circle center. The circular segment joining the two paths represents the
            // path of centripetal acceleration. Solve for max velocity based on max acceleration about the
            // radius of the circle, defined indirectly by junction deviation. This may be also viewed as
            // path width or max_jerk in the previous Grbl version. This approach does not actually deviate
            // from path, but used as a robust way to compute cornering speeds, as it takes into account the
            // nonlinearities of both the junction angle and junction velocity.
            //
            // NOTE: If the junction deviation value is finite, the motions are executed in exact path
            // mode (G61). If the junction deviation value is zero, the motions are executed in exact
            // stop mode (G61.1) manner. In the future, if continuous mode (G64) is desired, the math here
            // is exactly the same. Instead of motioning all the way to junction point, the machine will
            // just follow the arc circle defined here. The Arduino doesn't have the CPU cycles to perform
            // a continuous mode path, but ARM-based microcontrollers most certainly do.
            //
            // NOTE: The max junction speed is a fixed value, since machine acceleration limits cannot be
            // changed dynamically during operation nor can the line move geometry. This must be kept in
            // memory in the event of a feedrate override changing the nominal speeds of blocks, which can
            // change the overall maximum entry speed conditions of all blocks.
            Vector<float> junctionUnitVector;
            float         cosTheta = 0.0f;

            for (int i = 0; i < numberAxis; ++i) {
                cosTheta -= previousUnitVector[i] * unitVector[i];
                junctionUnitVector[i] = unitVector[i] - previousUnitVector[i];
            }
            for (int i = 0; i < numberAxis; i++) {
                cosTheta -= previousUnitVector[i] * unitVector[i];
                junctionUnitVector[i] = unitVector[i] - previousUnitVector[i];
            }

            // NOTE: Computed without any expensive trig, sin() or acos(), by trig half angle identity of cos(theta).
            const float MINIMUM_JUNCTION_SPEED = 0.0f;  // (mm/min)
            const float minJunctionSpeedSq     = MINIMUM_JUNCTION_SPEED * MINIMUM_JUNCTION_SPEED;

            if (cosTheta > 0.999999f) {
                //  For a 0 degree acute junction, just set minimum junction speed.
                block.maxJunctionSpeedSqr = minJunctionSpeedSq;
            } else {
                if (cosTheta < -0.999999) {
                    // Junction is a straight line or 180 degrees. Junction speed is infinite.
                    block.maxJunctionSpeedSqr = 1e38f;
                } else {
                    junctionUnitVector.Normalize(numberAxis);
                    float junctionAcceleration = LimitAccelerationByAxes(junctionUnitVector, config);
                    float sinThetaD2           = float(std::sqrt(0.5f * (1.0f - cosTheta)));  // Trig half angle identity. Always positive.

                    if (block.maxJunctionSpeedSqr < minJunctionSpeedSq) {
                        block.maxJunctionSpeedSqr = minJunctionSpeedSq;
                    }
                    block.maxJunctionSpeedSqr = (junctionAcceleration * config._junctionDeviation * sinThetaD2) / (1.0f - sinThetaD2);
                }
            }

            // Block system motion from updating this data to ensure next g-code motion is computed correctly.
            // if (!(block->motion.systemMotion)) { // TODO FIXME!

            float nominalSpeed = block.nominalSpeed;

            // Computes and updates the max entry speed (sqr) of the block, based on the minimum of the junction's
            // previous and current nominal speeds and max junction speed.
            //
            // Compute the junction maximum entry based on the minimum of the junction speed and neighboring nominal speeds.
            if (nominalSpeed > previousNominalSpeed) {
                block.maxJunctionSpeedSqr = previousNominalSpeed * previousNominalSpeed;
            } else {
                block.maxJunctionSpeedSqr = nominalSpeed * nominalSpeed;
            }

            // Update previous path unit_vector and planner position.
            this->previousUnitVector   = unitVector;
            this->lastPositionVector   = targetPositionSteps;
            this->previousNominalSpeed = nominalSpeed;

            buffer.IncrementWriteIndex();

            // Finish up by recalculating the plan with the new block.
            Recalculate();
        }
    }
};
