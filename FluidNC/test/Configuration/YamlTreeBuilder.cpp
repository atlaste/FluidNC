#include "../TestFramework.h"

#include <string>

#include <src/Configuration/Tokenizer.h>
#include <src/Configuration/Parser.h>
#include <src/Configuration/ParserHandler.h>
#include <src/Configuration/Validator.h>
#include <src/Configuration/AfterParse.h>
#include <src/Configuration/Configurable.h>
#include <src/StringStream.h>

namespace Configuration {
    class TestBasic : public Configurable {
    public:
        String a;
        String b;
        String c;

        void group(HandlerBase& handler) {
            handler.item("a", a);
            handler.item("b", b);
            handler.item("c", c);
        }

        const char* name() const override { return "test"; }
    };

    class TestBasic2 : public Configurable {
    public:
        String aap;
        int    banaan = 0;

        void group(HandlerBase& handler) {
            handler.item("aap", aap);
            handler.item("banaan", banaan);
        }

        const char* name() const override { return "test"; }
    };

    enum stepper_id_t {
        ST_TIMED = 0,
        ST_RMT,
        ST_I2S_STREAM,
        ST_I2S_STATIC,
    };

    EnumItem stepTypes[] = {
        { ST_TIMED, "Timed" }, { ST_RMT, "RMT" }, { ST_I2S_STATIC, "I2S_static" }, { ST_I2S_STREAM, "I2S_stream" }, EnumItem(ST_RMT)
    };

    class TestBasicEnum : public Configurable {
    public:
        int aap;
        int value;
        int banaan;

        void group(HandlerBase& handler) override {
            handler.item("aap", aap);
            handler.item("type", value, stepTypes);
            handler.item("banaan", banaan);
        }

        const char* name() const override { return "test"; }
    };

    class TestHierarchical : public Configurable {
    public:
        TestBasic*  n1  = nullptr;
        TestBasic2* n2  = nullptr;
        int         foo = 0;

        void group(HandlerBase& handler) override {
            handler.section("n1", n1);
            handler.section("n2", n2);
            handler.item("foo", foo);
        }

        const char* name() const override { return "test"; }
    };

    struct Helper {
        template <typename T>
        static inline void Parse(const char* config, T& test) {
            Parser        p(config, config + strlen(config));
            ParserHandler handler(p);

            handler.enterSection("machine", &test);
            // test.group(handler);
            // for (; !p.Eof(); handler.moveNext()) {
            //     test.group(handler);
            // }
        }
    };

    Test(YamlTreeBuilder, BasicProperties) {
        const char* config = "a: aap\n"
                             "b: banaan\n"
                             "\n"
                             "c: chocolade\n";

        TestBasic test;
        Helper::Parse(config, test);

        Assert(test.a == "aap");
        Assert(test.b == "banaan");
        Assert(test.c == "chocolade");
    }

    Test(YamlTreeBuilder, BasicPropertiesInvert) {
        const char* config = "c: chocolade\n"
                             "b: banaan\n"
                             "a: aap\n";

        TestBasic test;
        Helper::Parse(config, test);

        Assert(test.a == "aap");
        Assert(test.b == "banaan");
        Assert(test.c == "chocolade");
    }

    Test(YamlTreeBuilder, BasicProperties2) {
        const char* config = "aap: aap\n"
                             "banaan: 2\n";

        TestBasic2 test;
        Helper::Parse(config, test);

        Assert(test.aap == "aap");
        Assert(test.banaan == 2);
    }

    Test(YamlTreeBuilder, BasicPropertiesInvert2) {
        const char* config = "banaan: 2\n"
                             "aap: aap\n";

        TestBasic2 test;
        Helper::Parse(config, test);

        Assert(test.aap == "aap");
        Assert(test.banaan == 2);
    }

    Test(YamlTreeBuilder, Hierarchical1) {
        const char* config = "n1:\n"
                             "  a: aap\n"
                             "  b: banaan\n"
                             "  \n"
                             "  c: chocolade\n"
                             "n2:\n"
                             "  banaan: 2\n"
                             "  aap: aap\n"
                             "foo: 2\n";

        TestHierarchical test;
        Helper::Parse(config, test);

        {
            Assert(test.n1 != nullptr);
            Assert(test.n1->a == "aap");
            Assert(test.n1->b == "banaan");
            Assert(test.n1->c == "chocolade");
        }

        {
            Assert(test.n2 != nullptr);
            Assert(test.n2->banaan == 2);
            Assert(test.n2->aap == "aap");
        }
        Assert(test.foo == 2);
    }

    Test(YamlTreeBuilder, Hierarchical2) {
        const char* config = "n2:\n"
                             "  banaan: 2\n"
                             "  aap: aap\n"
                             "n1:\n"
                             "  a: aap\n"
                             "  b: banaan\n"
                             "  \n"
                             "  c: chocolade\n"
                             "foo: 2\n";

        TestHierarchical test;
        Helper::Parse(config, test);

        {
            Assert(test.n1 != nullptr);
            Assert(test.n1->a == "aap");
            Assert(test.n1->b == "banaan");
            Assert(test.n1->c == "chocolade");
        }

        {
            Assert(test.n2 != nullptr);
            Assert(test.n2->banaan == 2);
            Assert(test.n2->aap == "aap");
        }
        Assert(test.foo == 2);
    }

    Test(YamlTreeBuilder, Hierarchical3) {
        const char* config = "foo: 2\n"
                             "n2:\n"
                             "  banaan: 2\n"
                             "  aap: aap\n"
                             "n1:\n"
                             "  a: aap\n"
                             "  b: banaan\n"
                             "  \n"
                             "  c: chocolade\n";

        TestHierarchical test;
        Helper::Parse(config, test);

        {
            Assert(test.n1 != nullptr);
            Assert(test.n1->a == "aap");
            Assert(test.n1->b == "banaan");
            Assert(test.n1->c == "chocolade");
        }

        {
            Assert(test.n2 != nullptr);
            Assert(test.n2->banaan == 2);
            Assert(test.n2->aap == "aap");
        }
        Assert(test.foo == 2);
    }

    Test(YamlTreeBuilder, Enum1) {
        {
            const char*   config = "aap: 1\ntype: Timed\nbanaan: 2\n";
            TestBasicEnum test;
            Helper::Parse(config, test);
            Assert(test.value == int(ST_TIMED));
        }

        {
            const char*   config = "aap: 1\ntype: RMT\nbanaan: 2\n";
            TestBasicEnum test;
            Helper::Parse(config, test);
            Assert(test.value == int(ST_RMT));
        }
        {
            const char*   config = "aap: 1\ntype: I2S_static\nbanaan: 2\n";
            TestBasicEnum test;
            Helper::Parse(config, test);
            Assert(test.value == int(ST_I2S_STATIC));
        }
        {
            const char*   config = "aap: 1\ntype: I2S_stream\nbanaan: 2\n";
            TestBasicEnum test;
            Helper::Parse(config, test);
            Assert(test.value == int(ST_I2S_STREAM));
        }
    }

    class TestCompleteTypes : public Configurable {
    public:
        bool                    myBool  = false;
        int                     myInt   = 0;
        float                   myFloat = 0.0f;
        std::vector<speedEntry> mySpeedMap;
        Pin                     myPin;
        IPAddress               myIp;
        UartData                myUartData;
        UartParity              myUartParity;
        UartStop                myUartStop;
        int                     myEnum = 0;
        String                  myString;

        bool hasValidated = false;

        void validate(Validator& handler) override {
            handler.item("bool", myBool);
            handler.item("int", myInt, 1, 10);
            handler.item("float", myFloat, 1.0f, 100.0f);
            handler.item("speedMap", mySpeedMap);
            handler.item("pin", myPin);
            handler.item("ip", myIp);
            handler.item("uart", myUartData, myUartParity, myUartStop);
            handler.item("enum", myEnum, stepTypes);
            handler.item("string", myString, 1, 10);

            // Set has validated.
            this->hasValidated = true;
            Configurable::validate(handler);
        }

        void group(HandlerBase& handler) override {
            handler.item("bool", myBool);
            handler.item("int", myInt);
            handler.item("float", myFloat);
            handler.item("speedMap", mySpeedMap);
            handler.item("pin", myPin);
            handler.item("ip", myIp);
            handler.item("uart", myUartData, myUartParity, myUartStop);
            handler.item("enum", myEnum, stepTypes);
            handler.item("string", myString);
        }

        void afterParse(HandlerBase& handler) {
            if (myString.length() == 0) {
                myString = "aap";
            }
            Configurable::afterParse(handler);
        }

        const char* name() const override { return "test"; }
    };

    Test(YamlTreeBuilder, Composite1) {
        {
            const char* config = "pin: gpio.12:pu\n"
                                 "float: 12.34\n"
                                 "speedMap: 20=0% 100=100%\n"
                                 "bool: true\n"
                                 "int: 2\n"
                                 "ip:127.0.0.1\n"
                                 "uart: 8e1\n"
                                 "enum: I2S_static\n";
            TestCompleteTypes test;
            Helper::Parse(config, test);

            Assert(test.myBool == true);
            Assert(test.myInt == 2);
            Assert(test.myFloat >= 12.33f && test.myFloat <= 12.35f);
            Assert(test.mySpeedMap.size() == 2);
            Assert(test.mySpeedMap[0].speed == 20);
            Assert(test.mySpeedMap[1].speed == 100);
            Assert(test.mySpeedMap[0].percent == 0);
            Assert(test.mySpeedMap[1].percent == 100);
            Assert(test.myPin.name() == "gpio.12:pu");
            Assert(test.myIp.toString() == "127.0.0.1");
            Assert(test.myUartData == UartData::Bits8);
            Assert(test.myUartParity == UartParity::Even);
            Assert(test.myUartStop == UartStop::Bits1);
        }
    }

    Test(YamlTreeBuilder, Composite2) {
        const char* config = "pin: gpio.12:pu\n"
                             "float: 12.34\n"
                             "speedMap: 20=0% 100=100%\n"
                             "bool: true\n"
                             "int: 2\n"
                             "ip:127.0.0.1\n"
                             "uart: 8e1\n"
                             "enum: I2S_static\n"
                             "string: 'aapjes kijken'\n";

        const char* correct = "bool: true\n"
                              "int: 2\n"
                              "float: 12.340\n"
                              "speedMap: 20=0.000% 100=100.000%\n"
                              "pin: gpio.12:pu\n"
                              "ip: 127.0.0.1\n"
                              "uart: 8E1\n"
                              "enum: I2S_static\n"
                              "string: 'aapjes kijken'\n";
        TestCompleteTypes test;
        Helper::Parse(config, test);

        StringStream ss;
        Generator    gen(ss);
        test.group(gen);

        auto s = ss.str();
        Assert(s.equals(correct));
    }

    class TestCompleteTypes2 : public Configurable {
    public:
        TestCompleteTypes* child = nullptr;

        bool hasValidated = false;

        void validate(Validator& validator) {
            // Set has validated.
            hasValidated = true;

            // defer to default behavior
            Configurable::validate(validator);
        }

        void group(HandlerBase& handler) { handler.section("child", child); }

        const char* name() const override { return "test"; }

        ~TestCompleteTypes2() { delete child; }
    };

    Test(YamlTreeBuilder, Composite3) {
        {
            const char* config = "child:\n"
                                 "  pin: gpio.12:pu\n"
                                 "  float: 12.34\n"
                                 "  speedMap: 20=0% 100=100%\n"
                                 "  bool: true\n"
                                 "  int: 2\n"
                                 "  ip:127.0.0.1\n"
                                 "  uart: 8n1\n"
                                 "  enum: I2S_static\n";
            TestCompleteTypes2 container;
            Helper::Parse(config, container);

            Assert(container.child != nullptr);

            auto& test = *container.child;

            Assert(test.myBool == true);
            Assert(test.myInt == 2);
            Assert(test.myFloat >= 12.33f && test.myFloat <= 12.35f);
            Assert(test.mySpeedMap.size() == 2);
            Assert(test.mySpeedMap[0].speed == 20);
            Assert(test.mySpeedMap[1].speed == 100);
            Assert(test.mySpeedMap[0].percent == 0);
            Assert(test.mySpeedMap[1].percent == 100);
            Assert(test.myPin.name() == "gpio.12:pu");
            Assert(test.myIp.toString() == "127.0.0.1");
            Assert(test.myUartData == UartData::Bits8);
            Assert(test.myUartParity == UartParity::None);
            Assert(test.myUartStop == UartStop::Bits1);
        }
    }

    Test(YamlTreeBuilder, Composite4) {
        const char* config = "child:\n"
                             "  pin: gpio.12:pu\n"
                             "  float: 12.34\n"
                             "  speedMap: 20=0% 100=100%\n"
                             "  bool: true\n"
                             "  int: 2\n"
                             "  ip:127.0.0.1\n"
                             "  uart: 8e1\n"
                             "  enum: I2S_static\n";

        const char* correct = "child:\n"
                              "  bool: true\n"
                              "  int: 2\n"
                              "  float: 12.340\n"
                              "  speedMap: 20=0.000% 100=100.000%\n"
                              "  pin: gpio.12:pu\n"
                              "  ip: 127.0.0.1\n"
                              "  uart: 8E1\n"
                              "  enum: I2S_static\n"
                              "\n";
        TestCompleteTypes2 test;
        Helper::Parse(config, test);

        StringStream ss;
        Generator    gen(ss);
        test.group(gen);

        auto s = ss.str();
        Assert(s.equals(correct));
    }

    Test(YamlTreeBuilder, Generator1) {
        const char* config = "pin: gpio.2:pd\n"
                             "float: 12.34\n"
                             "speedMap: None\n"
                             "bool: true\n"
                             "int: 2\n"
                             "ip:127.0.0.1\n"
                             "uart: 5o1.5\n"
                             "enum: I2S_static\n";

        const char* correct = "bool: true\n"
                              "int: 2\n"
                              "float: 12.340\n"
                              "speedMap: none\n"
                              "pin: gpio.2:pd\n"
                              "ip: 127.0.0.1\n"
                              "uart: 5O1.5\n"
                              "enum: I2S_static\n";
        TestCompleteTypes test;
        Helper::Parse(config, test);

        StringStream ss;
        Generator    gen(ss);
        test.group(gen);

        auto s = ss.str();
        Assert(s.equals(correct));
    }

    Test(YamlTreeBuilder, Generator2) {
        const char* config = "pin: gpio.2:pd:low\n"
                             "float: 12.34\n"
                             "speedMap: none\n"
                             "bool: true\n"
                             "int: 2\n"
                             "ip:127.0.0.1\n"
                             "uart: 5E2\n"
                             "enum: aap\n";

        const char* correct = "bool: true\n"
                              "int: 2\n"
                              "float: 12.340\n"
                              "speedMap: none\n"
                              "pin: gpio.2:low:pd\n"
                              "ip: 127.0.0.1\n"
                              "uart: 5E2\n"
                              "enum: RMT\n";
        TestCompleteTypes test;
        Helper::Parse(config, test);

        StringStream ss;
        Generator    gen(ss);
        test.group(gen);

        auto s = ss.str();
        Assert(s.equals(correct));
    }

    Test(YamlTreeBuilder, Generator3) {
        const char* config = "pin: gpio.2:pd:low\n"
                             "float: 12.34\n"
                             "speedMap: none\n"
                             "bool: true\n"
                             "int: 2\n"
                             "ip:127.0.0.1\n"
                             "uart: 5N2\n"
                             "enum: aap\n";

        const char* correct = "bool: true\n"
                              "int: 2\n"
                              "float: 12.340\n"
                              "speedMap: none\n"
                              "pin: gpio.2:low:pd\n"
                              "ip: 127.0.0.1\n"
                              "uart: 5N2\n"
                              "enum: unknown\n";
        TestCompleteTypes test;
        Helper::Parse(config, test);

        test.myEnum = 14;

        StringStream ss;
        Generator    gen(ss);
        test.group(gen);

        auto s = ss.str();
        Assert(s.equals(correct));
    }

    Test(YamlTreeBuilder, Validator1) {
        const char* config = "child:\n"
                             "  pin: gpio.12:pu\n"
                             "  float: 12.34\n"
                             "  speedMap: 20=0% 100=100%\n"
                             "  bool: true\n"
                             "  int: 2\n"
                             "  ip:127.0.0.1\n"
                             "  uart: 8e1\n"
                             "  enum: I2S_static\n"
                             "  string: 'banaan'\n";

        TestCompleteTypes2 test;
        Helper::Parse(config, test);

        Assert(!test.hasValidated);
        Assert(test.child != nullptr && !test.child->hasValidated);

        Validator validator;
        test.validate(validator);

        Assert(test.hasValidated);
        Assert(test.child != nullptr && test.child->hasValidated);
    }

    Test(YamlTreeBuilder, Validator2) {
        const char* config = "child:\n"
                             "  pin: gpio.12:pu\n"
                             "  float: 10000\n"
                             "  speedMap: 20=0% 100=100%\n"
                             "  bool: true\n"
                             "  int: 2\n"
                             "  ip:127.0.0.1\n"
                             "  uart: 8e1\n"
                             "  enum: I2S_static\n"
                             "  string: 'banaan'\n";

        TestCompleteTypes2 test;
        Helper::Parse(config, test);

        Assert(!test.hasValidated);
        Assert(test.child != nullptr && !test.child->hasValidated);

        Validator validator;
        test.validate(validator);
        AssertThrow(validator.finishValidation());
    }

    Test(YamlTreeBuilder, Validator3) {
        const char* config = "child:\n"
                             "  pin: gpio.12:pu\n"
                             "  float: 10000\n"
                             "  speedMap: 20=0% 100=100%\n"
                             "  bool: true\n"
                             "  int: 2\n"
                             "  ip:127.0.0.1\n"
                             "  uart: 8e1\n"
                             "  enum: I2S_static\n"
                             "  string: 'banaan'\n";

        TestCompleteTypes2 test;
        Helper::Parse(config, test);

        Assert(!test.hasValidated);
        Assert(test.child != nullptr && !test.child->hasValidated);

        Validator validator;
        test.validate(validator);
        AssertThrow(validator.finishValidation());
    }

    Test(YamlTreeBuilder, Validator4) {
        const char* config = "child:\n"
                             "  pin: gpio.12:pu\n"
                             "  float: 2\n"
                             "  speedMap: 20=0% 100=100%\n"
                             "  bool: true\n"
                             "  int: 20000\n"
                             "  ip:127.0.0.1\n"
                             "  uart: 8e1\n"
                             "  enum: I2S_static\n"
                             "  string: 'banaan'\n";

        TestCompleteTypes2 test;
        Helper::Parse(config, test);

        Assert(!test.hasValidated);
        Assert(test.child != nullptr && !test.child->hasValidated);

        Validator validator;
        test.validate(validator);
        AssertThrow(validator.finishValidation());
    }

    Test(YamlTreeBuilder, Validator5) {
        const char* config = "child:\n"
                             "  pin: gpio.12:pu\n"
                             "  float: 2\n"
                             "  speedMap: 20=0% 100=200%\n"
                             "  bool: true\n"
                             "  int: 2\n"
                             "  ip:127.0.0.1\n"
                             "  uart: 8e1\n"
                             "  enum: I2S_static\n"
                             "  string: 'banaan'\n";

        TestCompleteTypes2 test;
        Helper::Parse(config, test);

        Assert(!test.hasValidated);
        Assert(test.child != nullptr && !test.child->hasValidated);

        Validator validator;
        test.validate(validator);
        AssertThrow(validator.finishValidation());
    }

    Test(YamlTreeBuilder, Validator6) {
        const char* config = "child:\n"
                             "  pin: gpio.12:pu\n"
                             "  float: 2\n"
                             "  speedMap: 20=0% 100=100%\n"
                             "  bool: true\n"
                             "  int: 2\n"
                             "  ip:127.0.0.1\n"
                             "  uart: 8e1\n"
                             "  enum: aap\n"
                             "  string: 'banaan'\n";

        TestCompleteTypes2 test;
        Helper::Parse(config, test);

        Assert(!test.hasValidated);
        Assert(test.child != nullptr && !test.child->hasValidated);
        test.child->myEnum = 1000;

        Validator validator;
        test.validate(validator);
        AssertThrow(validator.finishValidation());
    }

    Test(YamlTreeBuilder, AfterParse1) {
        const char* config = "child:\n"
                             "  int: 2\n";

        TestCompleteTypes2 test;
        Helper::Parse(config, test);

        AfterParse ap;
        test.afterParse(ap);

        Assert(test.child != nullptr && test.child->myString == "aap");
    }
}
