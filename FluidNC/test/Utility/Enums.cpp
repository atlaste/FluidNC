#include "../TestFramework.h"

#include <src/EnumItem.h>
#include <src/StringStream.h>
#include <src/MyIOStream.h>

namespace Configuration {
    Test(UtilEnum, Basics) {
        // Straight from the comment:
        const EnumItem stepTypes[] = { { 2, "Timed" }, { 3, "RMT" }, { 4, "I2S_static" }, { 5, "I2S_stream" }, EnumItem(3) };

        Assert(EnumItem::count(stepTypes) == 4);
        {
            auto i2s = EnumItem::find(stepTypes, "I2S_static");
            Assert(i2s.value == 4);
            Assert(String(i2s.name) == "I2S_static");
            Assert(!i2s.undefined());
        }
        {
            auto i2s = EnumItem::find(stepTypes, 4);
            Assert(i2s.value == 4);
            Assert(String(i2s.name) == "I2S_static");
            Assert(!i2s.undefined());
        }
        {
            auto rmt = EnumItem::default(stepTypes);
            Assert(rmt.value == 3);
            Assert(String(rmt.name) == "RMT");
            Assert(!rmt.undefined());
        }
        {
            auto undef = EnumItem::find(stepTypes, 14);
            Assert(undef.value == 0);
            Assert(undef.name == nullptr);
            Assert(undef.undefined());
        }
        {
            auto undef = EnumItem::find(stepTypes, "aap");
            Assert(undef.value == 0);
            Assert(undef.name == nullptr);
            Assert(undef.undefined());
        }
    }
}
