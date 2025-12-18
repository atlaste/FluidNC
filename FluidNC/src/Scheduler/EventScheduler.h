#pragma once

#include "Platform.h"
#include "Timer.h"
#include "Timepoint.h"
#include "Timespan.h"
#include "Event.h"

#include <algorithm>  // heap
#include <cstring>    // memset
#include <cassert>

namespace Scheduler {
    class EventScheduler {
        static INLINE bool IRAM CompareEventTime(const Event* a, const Event* b) { return a->deadline_ > b->deadline_; }

        static const int MaxNumberHeapEntries = 128;

    protected:
        Event* heap_[MaxNumberHeapEntries + 1];
        size_t heapCount_ = 0;

    public:
        EventScheduler() : heap_(), heapCount_(0) { memset(heap_, 0, sizeof(heap_)); }

        EventScheduler(const EventScheduler&) = delete;
        EventScheduler(EventScheduler&&)      = default;

        EventScheduler& operator=(const EventScheduler&) = delete;
        EventScheduler& operator=(EventScheduler&&)      = default;

        Event** IRAM AllEvents() { return heap_; }
        size_t IRAM  NumberEvents() const { return heapCount_; }

        virtual void updateTimeDelta(int64_t delta) {
            if (delta < 0) {
                // forward to back keeps it sorted:
                for (size_t it = 0; it < heapCount_; ++it) {
                    heap_[it]->deadline_ += delta;
                }
            } else {
                // back to forward keeps it sorted:
                for (size_t it = heapCount_; it > 0; --it) {
                    heap_[it - 1]->deadline_ += delta;
                }
            }
        }

        INLINE bool IRAM add(Event* evt) {
            // Check if this is already scheduled. If so, that's not good.
            for (size_t i = 0; i < heapCount_; ++i) {
                if (heap_[i] == evt) {
                    return false;
                }
            }

            if (heapCount_ >= MaxNumberHeapEntries) {
                return false;
            }

            heap_[heapCount_++] = evt;
            std::push_heap(heap_, heap_ + heapCount_, CompareEventTime);
            return (heap_[0] == evt);
        }

        INLINE bool IRAM remove(Event* evt) {
            // Find the event in the heap
            for (size_t i = 0; i < heapCount_; ++i) {
                if (heap_[i] == evt) {
                    // Replace with last element and decrease size
                    heap_[i] = heap_[heapCount_ - 1];
                    --heapCount_;

                    // Restore heap property if heap is not empty
                    // NOTE: Not needed because of sentinel.
                    if (heapCount_ > 0) {
                        std::make_heap(heap_, heap_ + heapCount_, CompareEventTime);
                    }

                    return true;
                }
            }

            return false;  // Event not found
        }

        INLINE Event* IRAM first() {
            assert(heapCount_ != 0 && "Heap should never get empty");

            // The heap will never get empty, because of the sentinel.
            return heap_[0];
        }

        INLINE void IRAM removeFirst() {
            assert(heapCount_ != 0 && "Heap should never get empty");

            // Remove first item from the heap:
            std::pop_heap(heap_, heap_ + heapCount_, CompareEventTime);
            --heapCount_;
        }

        INLINE Event* IRAM grabEvent() {
            assert(heapCount_ != 0 && "Heap should never get empty");

            // Only the scheduler grabs events here. The heap will never get empty, because of the sentinel.
            auto result = heap_[0];

            // Remove first item from the heap:
            std::pop_heap(heap_, heap_ + heapCount_, CompareEventTime);
            --heapCount_;

            return result;
        }

        virtual ~EventScheduler() = default;
    };
}
