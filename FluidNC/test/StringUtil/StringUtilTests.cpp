#include "TestFramework.h"

#include <string_util.h>
#include <string>

namespace StringUtil {

    Test(StringUtil, ToLower) {
        Assert(string_util::tolower('A') == 'a', "A -> a");
        Assert(string_util::tolower('Z') == 'z', "Z -> z");
        Assert(string_util::tolower('a') == 'a', "a unchanged");
        Assert(string_util::tolower('0') == '0', "digit unchanged");
    }

    Test(StringUtil, EqualIgnoreCase) {
        Assert(string_util::equal_ignore_case("abc", "abc"), "same");
        Assert(string_util::equal_ignore_case("ABC", "abc"), "upper lower");
        Assert(string_util::equal_ignore_case("", ""), "empty");
        Assert(!string_util::equal_ignore_case("ab", "abc"), "different length");
        Assert(!string_util::equal_ignore_case("abd", "abc"), "different content");
    }

    Test(StringUtil, StartsWithIgnoreCase) {
        Assert(string_util::starts_with_ignore_case("Hello World", "hello"), "prefix match");
        Assert(string_util::starts_with_ignore_case("HELLO", "hello"), "full match");
        Assert(!string_util::starts_with_ignore_case("Hi", "hello"), "no match");
        Assert(!string_util::starts_with_ignore_case("Hel", "hello"), "shorter than prefix");
        Assert(string_util::starts_with_ignore_case("", ""), "empty");
    }

    Test(StringUtil, Trim) {
        Assert(string_util::trim("  x  ") == "x", "both sides");
        Assert(string_util::trim("  x") == "x", "leading");
        Assert(string_util::trim("x  ") == "x", "trailing");
        Assert(string_util::trim(" \t\n\r\f\v ") == "", "all whitespace");
        Assert(string_util::trim("") == "", "empty");
        Assert(string_util::trim("x") == "x", "no trim");
    }

    Test(StringUtil, FromXdigit) {
        uint8_t v;
        Assert(string_util::from_xdigit('0', v) && v == 0, "0");
        Assert(string_util::from_xdigit('9', v) && v == 9, "9");
        Assert(string_util::from_xdigit('a', v) && v == 10, "a");
        Assert(string_util::from_xdigit('f', v) && v == 15, "f");
        Assert(string_util::from_xdigit('A', v) && v == 10, "A");
        Assert(!string_util::from_xdigit('g', v), "invalid g");
        Assert(!string_util::from_xdigit(' ', v), "invalid space");
    }

    Test(StringUtil, FromHex) {
        uint8_t v;
        Assert(string_util::from_hex("0", v) && v == 0, "0");
        Assert(string_util::from_hex("ff", v) && v == 255, "ff");
        Assert(string_util::from_hex("1a", v) && v == 0x1a, "1a");
        Assert(!string_util::from_hex("", v), "empty");
        Assert(!string_util::from_hex("1g", v), "invalid char");
        Assert(!string_util::from_hex("123", v), "too long");
    }

    Test(StringUtil, FromDecimalUint32) {
        uint32_t v;
        Assert(string_util::from_decimal("0", v) && v == 0, "0");
        Assert(string_util::from_decimal("123", v) && v == 123, "123");
        Assert(string_util::from_decimal("999", v) && v == 999, "999");
        Assert(!string_util::from_decimal("", v), "empty");
        Assert(!string_util::from_decimal("12a", v), "invalid");
    }

    Test(StringUtil, FromDecimalInt32) {
        int32_t v;
        Assert(string_util::from_decimal("0", v) && v == 0, "0");
        Assert(string_util::from_decimal("-1", v) && v == -1, "-1");
        Assert(string_util::from_decimal("42", v) && v == 42, "42");
        Assert(!string_util::from_decimal("", v), "empty");
        Assert(!string_util::from_decimal("1.5", v), "float invalid");
    }

    Test(StringUtil, FromFloat) {
        float v;
        Assert(string_util::from_float("0", v) && v == 0.0f, "0");
        Assert(string_util::from_float("1.5", v) && v > 1.4f && v < 1.6f, "1.5");
        Assert(string_util::from_float("-0.5", v) && v > -0.6f && v < -0.4f, "-0.5");
        Assert(!string_util::from_float("", v), "empty");
        Assert(!string_util::from_float("1.5x", v), "invalid tail");
    }

    Test(StringUtil, Split) {
        std::string_view input;
        std::string_view next;

        input = "a:b";
        Assert(string_util::split(input, next, ':') == true, "split returns true");
        Assert(input == "a", "left part");
        Assert(next == "b", "right part");

        input = "no-delim";
        Assert(string_util::split(input, next, ':') == false, "no delim");
        Assert(next == "", "next empty");

        input = "a:";
        Assert(string_util::split(input, next, ':') == true, "trailing delim");
        Assert(input == "a", "left");
        Assert(next == "", "next empty after delim");
    }

    Test(StringUtil, SplitPrefix) {
        std::string_view rest;
        std::string_view prefix;

        rest = "a:b:c";
        Assert(string_util::split_prefix(rest, prefix, ':') == true, "split_prefix true");
        Assert(prefix == "a", "prefix");
        Assert(rest == "b:c", "rest");

        string_util::split_prefix(rest, prefix, ':');
        Assert(prefix == "b", "prefix b");
        Assert(rest == "c", "rest c");

        Assert(string_util::split_prefix(rest, prefix, ':') == true, "no delim in rest");
        Assert(prefix == "c", "prefix whole");
        Assert(rest == "", "rest empty");

        rest = "";
        Assert(string_util::split_prefix(rest, prefix, ':') == false, "empty rest");
    }
}
