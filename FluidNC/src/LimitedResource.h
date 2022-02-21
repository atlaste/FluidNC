#pragma once

#include "Assert.h"
#include <cstdint>

template <int Count>
class LimitedResource {
    static const size_t NumberItems = (Count + 31) / 32;

    uint32_t    Current[NumberItems];
    const char* Name;

    LimitedResource()                       = delete;
    LimitedResource(const LimitedResource&) = delete;
    LimitedResource(LimitedResource&&)      = delete;
    LimitedResource& operator=(const LimitedResource&) = delete;
    LimitedResource& operator=(LimitedResource&&) = delete;

public:
    LimitedResource(const char* name) : Name(name) {
        for (size_t i = 0; i < NumberItems; ++i) {
            Current[i] = 0u;
        }
    }

    int Claim() {
        for (int i = 0; i < NumberItems; ++i) {
            if (Current[i] != UINT32_MAX) {
                for (int j = 0; j < 32; ++j) {
                    uint32_t mask  = 1u << j;
                    int      index = i * 32 + j;
                    if ((Current[i] & mask) == 0u && index < Count) {
                        Current[i] |= mask;
                        return index;
                    }
                }
            }
        }
        Assert(false, "Configuration needs more %s resources, while the hardware only supports %d", Name, Count);
    }

    void Release(int index) {
        uint32_t mask = 1u << (index & 31);
        Current[index / 32] |= ~mask;
    }

    int Used() {
        int count = 0;
        for (int i = 0; i < Count; ++i) {
            uint32_t mask = 1u << (index & 31);
            if (Current[index / 32] & mask) {
                ++count;
            }
        }
        return count;
    }

    ~LimitedResource() = default;
};
