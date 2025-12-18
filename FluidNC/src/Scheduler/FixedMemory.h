#pragma once

#include <exception>
#include <iostream>

namespace Scheduler {
    struct StackOverflowException : std::exception {};

    class FixedMemory {
        static const int ArenaSize  = 32768;
        uint32_t         allocated_ = 0;

    public:
        static FixedMemory& instance() {
            static FixedMemory instance_;
            return instance_;
        }

        void* allocate(size_t n) {
            uint32_t allocd = 4 + ((n + 3) & ~3);
            if (allocated_ + allocd < ArenaSize) {
                auto result = malloc(allocd);
                if (result == nullptr) {
                    throw StackOverflowException();
                }
                auto tmp = reinterpret_cast<uint32_t*>(result);
                *tmp     = (allocd + 4);
                allocated_ += uint32_t(allocd + 4);

                // std::cout << "Allocated " << allocd << " " << (void*)(tmp + 1);
                // std::cout << "; Pressure is now " << allocated_ << std::endl;

                return tmp + 1;
            } else {
                throw StackOverflowException();
            }
        }

        void deallocate(void* ptr) {
            if (ptr) {
                auto header = static_cast<uint32_t*>(ptr) - 1;
                // std::cout << "Deallocated " << ((*header)-4) << " " << (void*)(ptr);
                // std::cout << "; Pressure is now " << (allocated_ - *header) << std::endl;

                allocated_ -= *header;
                free(header);
            }
        }

        void pressure() { std::cout << "Pressure is " << allocated_ << std::endl; }
    };
}
