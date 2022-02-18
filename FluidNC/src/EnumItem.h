#pragma once

#include "StringRange.h"

// Usage:
//
// const EnumItem stepTypes[] = {
//      { ST_TIMED, "Timed" }, { ST_RMT, "RMT" }, { ST_I2S_STATIC, "I2S_static" }, { ST_I2S_STREAM, "I2S_stream" }, EnumItem(ST_RMT)
// };
//
// Be sure to make it const and use the helper functions!

struct EnumItem {
    // Used for brace initialization
    EnumItem() : value(0), name(nullptr) {}

    // Set enumItem with a default value as last item in the EnumItem array. This is the terminator.
    EnumItem(int defaultValue) : value(defaultValue), name(nullptr) {}

    // Other items are here.
    EnumItem(int val, const char* n) : value(val), name(n) {}

    int         value;
    const char* name;

    static size_t count(const EnumItem* set) {
        size_t          count = 0;
        const EnumItem* e     = set;
        for (; e->name; ++e) {
            ++count;
        }
        return count;
    }

    static EnumItem find(const EnumItem* set, int value) {
        const EnumItem* e = set;
        for (; e->name; ++e) {
            if (e->value == value) {
                return *e;
            }
        }
        return EnumItem();
    }

    static EnumItem find(const EnumItem* set, StringRange name) {
        const EnumItem* e = set;
        for (; e->name; ++e) {
            if (name.equals(e->name)) {
                return *e;
            }
        }
        return EnumItem();
    }

    static EnumItem default(const EnumItem* set) {
        auto c         = count(set);
        auto deftValue = set[c];
        return find(set, deftValue.value);
    }

    bool undefined() const { return name == nullptr; }
};
