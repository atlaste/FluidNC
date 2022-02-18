#include "../TestFramework.h"

#include <src/Configuration/Tokenizer.h>
#include <src/Configuration/Parser.h>
#include <src/Configuration/ParseException.h>
#include <tuple>
#include <vector>

namespace Configuration {
    struct UartInfo {
        UartData   wordLength;
        UartParity parity;
        UartStop   stop;
    };

    template <typename T>
    struct YamlSpecificParser;

    template <>
    struct YamlSpecificParser<bool> {
        static bool get(Configuration::Parser& p) { return p.boolValue(); }
    };

    template <>
    struct YamlSpecificParser<int> {
        static int get(Configuration::Parser& p) { return p.intValue(); }
    };

    template <>
    struct YamlSpecificParser<float> {
        static float get(Configuration::Parser& p) { return p.floatValue(); }
    };

    template <>
    struct YamlSpecificParser<std::vector<speedEntry>> {
        static std::vector<speedEntry> get(Configuration::Parser& p) { return p.speedEntryValue(); }
    };

    template <>
    struct YamlSpecificParser<Pin> {
        static Pin get(Configuration::Parser& p) { return p.pinValue(); }
    };

    template <>
    struct YamlSpecificParser<IPAddress> {
        static IPAddress get(Configuration::Parser& p) { return p.ipValue(); }
    };

    template <>
    struct YamlSpecificParser<StringRange> {
        static StringRange get(Configuration::Parser& p) { return p.stringValue(); }
    };

    template <>
    struct YamlSpecificParser<UartInfo> {
        static UartInfo get(Configuration::Parser& p) {
            UartInfo info;
            p.uartMode(info.wordLength, info.parity, info.stop);
            return info;
        }
    };

    template <>
    struct YamlSpecificParser<EnumItem> {
        static EnumItem get(Configuration::Parser& p) {
            EnumItem items[3] = { EnumItem(2, "boom"), EnumItem(3, "roos"), EnumItem(4, "vis") };
            auto     ei       = p.enumValue(items);
            return EnumItem(ei);
        }
    };

    template <typename T>
    struct ParseHelper {
        static T ParseCorrect(const char* test) {
            Configuration::Parser parser(test, test + strlen(test));
            parser.Tokenize();
            Assert(!parser.eof(), "Did not expect EOF");
            Assert(parser.key().equals("key"), "Expected 'key' as key");
            T result = YamlSpecificParser<T>::get(parser);
            parser.Tokenize();
            Assert(parser.eof(), "Expected EOF");
            return result;
        }

        static void ParseError(const char* test) {
            Configuration::Parser parser(test, test + strlen(test));
            parser.Tokenize();
            Assert(!parser.eof(), "Did not expect EOF");
            Assert(parser.key().equals("key"), "Expected 'key' as key");
            try {
                T result = YamlSpecificParser<T>::get(parser);
                Assert(false, "Expected error because type is incorrect.");
            } catch (ParseException&) {}
        }
    };

    template <typename Correct, typename Tuple>
    struct ParseAllHelper;

    template <typename Correct, typename... Errors>
    struct ParseAllHelper<Correct, std::tuple<Correct, Errors...>> {
        static Correct TestAll(const char* str) {
            // Except the correct case from the errors
            return ParseAllHelper<Correct, std::tuple<Errors...>>::TestAll(str);
        }
    };

    template <typename Correct, typename Head, typename... Errors>
    struct ParseAllHelper<Correct, std::tuple<Head, Errors...>> {
        static Correct TestAll(const char* str) {
            // Handle all errors first, recursively:
            ParseHelper<Head>::ParseError(str);
            return ParseAllHelper<Correct, std::tuple<Errors...>>::TestAll(str);
        }
    };

    template <typename Correct>
    struct ParseAllHelper<Correct, std::tuple<>> {
        static Correct TestAll(const char* str) {
            // Handle all errors first, recursively:
            return ParseHelper<Correct>::ParseCorrect(str);
        }
    };

    using AllNonStringTypes = std::tuple<bool, int, float, std::vector<speedEntry>, Pin, IPAddress, UartInfo>;

    Test(YamlProperties, StringValues) {
        {
            auto aap = ParseAllHelper<StringRange, AllNonStringTypes>::TestAll("key: aap");
            Assert(aap.equals("aap"));
        }

        {
            auto aap = ParseAllHelper<StringRange, AllNonStringTypes>::TestAll("key: 'aap'");
            Assert(aap.equals("aap"));
        }

        {
            auto aap = ParseAllHelper<StringRange, AllNonStringTypes>::TestAll("key: \"aap\"");
            Assert(aap.equals("aap"));
        }
    }

    using AllNonNumericTypes = std::tuple<bool, std::vector<speedEntry>, Pin, IPAddress, UartInfo>;

    Test(YamlProperties, IntValues) {
        {
            auto aap = ParseAllHelper<int, AllNonNumericTypes>::TestAll("key: 12");
            Assert(aap == 12);
        }
    }

    Test(YamlProperties, FloatValues) {
        {
            auto aap = ParseAllHelper<float, AllNonNumericTypes>::TestAll("key: 12");
            Assert(aap == 12);
        }
        {
            auto aap = ParseAllHelper<float, AllNonNumericTypes>::TestAll("key: 12.01");
            Assert(aap == 12.01f);
        }
        {
            auto aap = ParseAllHelper<float, AllNonNumericTypes>::TestAll("key: 1234567890");
            Assert(aap == 1234567890.0f);
        }
    }

    using AllNonBooleanTypes = std::tuple<int, float, std::vector<speedEntry>, Pin, IPAddress, UartInfo>;

    Test(YamlProperties, BoolValues) {
        {
            auto aap = ParseAllHelper<bool, AllNonBooleanTypes>::TestAll("key: true");
            Assert(aap == true);
        }

        {
            auto aap = ParseAllHelper<bool, AllNonBooleanTypes>::TestAll("key: false");
            Assert(aap == false);
        }
    }

    Test(YamlProperties, PinValues) {
        {
            auto aap = ParseAllHelper<Pin, AllNonStringTypes>::TestAll("key: gpio.12");
            Assert(aap.defined());
        }

        {
            auto aap = ParseAllHelper<Pin, AllNonStringTypes>::TestAll("key: gpio.12:pu");
            Assert(aap.defined());
        }
    }

    using AllUartData      = std::tuple<bool, int, float, std::vector<speedEntry>, Pin, IPAddress>;
    using AllNonUartTypes1 = std::tuple<int, float, std::vector<speedEntry>, Pin, IPAddress, UartInfo>;

    // Float and int are both numbers. If we have 8E1 then it's parsed as float as 8e1=80, and rounded to 80 for ints.
    using AllNonUartTypes2 = std::tuple<std::vector<speedEntry>, Pin, IPAddress, UartInfo>;

    Test(YamlProperties, UartValues) {
        {
            for (int i = 5; i <= 8; ++i) {
                for (int j = 0; j < 3; ++j) {
                    for (int k = 0; k < 3; ++k) {
                        std::ostringstream oss;
                        oss << "key: ";

                        // 5E1.5 f.ex. Maps to the enums.
                        oss << i;
                        oss << ("NEO"[j]);
                        if (k == 1) {
                            oss << "1.5";
                        } else {
                            oss << (1 + k / 2);
                        }

                        auto str = oss.str();

                        UartInfo uart;
                        if (j == 1) {
                            uart = ParseAllHelper<UartInfo, AllNonUartTypes2>::TestAll(str.c_str());
                        } else {
                            uart = ParseAllHelper<UartInfo, AllNonUartTypes1>::TestAll(str.c_str());
                        }

                        Assert(uart.stop == UartStop(k + 1));
                        Assert(uart.parity == UartParity(j == 0 ? 0 : (j + 1)));
                        Assert(uart.wordLength == UartData(i - 5));
                    }
                }
            }
        }
    }

}
