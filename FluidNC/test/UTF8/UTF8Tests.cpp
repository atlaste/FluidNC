#include "TestFramework.h"

#include <UTF8.h>
#include <vector>

namespace UTF8Tests {

    Test(UTF8, AsciiSingleByte) {
        UTF8 utf8;
        uint32_t value = 0;
        int8_t   r;

        r = utf8.decode(0x00, value);
        Assert(r == 1, "decode 0x00");
        Assert(value == 0, "value 0");

        r = utf8.decode(0x41, value);
        Assert(r == 1, "decode 'A'");
        Assert(value == 0x41, "value 'A'");

        r = utf8.decode(0x7f, value);
        Assert(r == 1, "decode 0x7f");
        Assert(value == 0x7f, "value 0x7f");
    }

    Test(UTF8, TwoByteSequence) {
        UTF8     utf8;
        uint32_t value = 0;

        int8_t r0 = utf8.decode(0xc2, value);
        Assert(r0 == 0, "first byte of 2-byte sequence returns 0");
        int8_t r1 = utf8.decode(0x80, value);
        Assert(r1 == 1, "second byte completes");
        Assert(value == 0x80, "U+80");
    }

    Test(UTF8, ThreeByteSequence) {
        UTF8     utf8;
        uint32_t value = 0;

        Assert(utf8.decode(0xe0, value) == 0, "byte 1");
        Assert(utf8.decode(0xa0, value) == 0, "byte 2");
        Assert(utf8.decode(0x80, value) == 1, "byte 3");
        Assert(value == 0x800, "U+800");
    }

    Test(UTF8, FourByteSequence) {
        UTF8     utf8;
        uint32_t value = 0;

        Assert(utf8.decode(0xf0, value) == 0, "byte 1");
        Assert(utf8.decode(0x90, value) == 0, "byte 2");
        Assert(utf8.decode(0x80, value) == 0, "byte 3");
        Assert(utf8.decode(0x80, value) == 1, "byte 4");
        Assert(value == 0x10000, "U+10000");
    }

    Test(UTF8, EncodeDecodeRoundTrip) {
        UTF8 utf8;
        std::vector<uint32_t> codepoints = { 0, 0x41, 0x7f, 0x80, 0xff, 0x100, 0x800, 0xffff, 0x10000, 0x10ffff };

        for (uint32_t cp : codepoints) {
            auto encoded = utf8.encode(cp);
            if (cp >= 0x110000) {
                Assert(encoded.empty(), "encode invalid returns empty");
                continue;
            }
            Assert(!encoded.empty(), "encode produces bytes");
            uint32_t decoded;
            bool     ok = utf8.decode(encoded, decoded);
            Assert(ok, "decode encoded succeeds");
            Assert(decoded == cp, "round-trip value match");
        }
    }

    Test(UTF8, EncodeOutOfRangeReturnsEmpty) {
        UTF8 utf8;
        auto v = utf8.encode(0x110000);
        Assert(v.empty(), "0x110000 returns empty");
    }

    Test(UTF8, VectorDecodeIncompleteSequence) {
        UTF8     utf8;
        uint32_t value;
        bool     ok = utf8.decode(std::vector<uint8_t> { 0xc2 }, value);
        Assert(!ok, "incomplete 2-byte sequence fails");
    }

    Test(UTF8, VectorDecodeInvalidContinuation) {
        UTF8     utf8;
        uint32_t value;
        bool     ok = utf8.decode(std::vector<uint8_t> { 0xc0, 0x30 }, value);
        Assert(!ok, "non-continuation byte in sequence fails");
    }

    Test(UTF8, VectorDecodeExtraBytesAfterSequence) {
        UTF8     utf8;
        uint32_t value;
        bool     ok = utf8.decode(std::vector<uint8_t> { 0xc2, 0x80, 0x41 }, value);
        Assert(!ok, "extra bytes after valid sequence fails");
    }

    Test(UTF8, InvalidStartByte) {
        UTF8     utf8;
        uint32_t value;
        int8_t   r = utf8.decode(0xf8, value);
        Assert(r == -1, "0xf8 invalid start");
        r = utf8.decode(0xff, value);
        Assert(r == -1, "0xff invalid start");
    }
}
