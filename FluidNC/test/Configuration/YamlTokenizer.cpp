#include "../TestFramework.h"

#include <src/Configuration/Tokenizer.h>
#include <src/Configuration/Parser.h>
#include <src/Configuration/ParseException.h>

namespace Configuration {
    class TokenizerBaseTest : public Tokenizer {
    public:
        TokenizerBaseTest(const char* start, const char* end) : Tokenizer(start, end) {}

        void RunTest() {
            Assert(Current() == 'a');
            Assert(!EndOfInput());
            Assert(IsAlpha());
            Assert(IsIdentifierChar());
            Assert(!IsWhiteSpace());
            Assert(!IsSpace());
            Assert(!IsEndLine());
            Assert(!IsDigit());
            Assert(EqualsCaseInsensitive("a1"));
            Inc();

            Assert(Current() == '1');
            Assert(!EndOfInput());
            Assert(!IsAlpha());
            Assert(IsIdentifierChar());
            Assert(!IsWhiteSpace());
            Assert(!IsSpace());
            Assert(!IsEndLine());
            Assert(IsDigit());
            Assert(EqualsCaseInsensitive("1"));
            Inc();

            Assert(Current() == ' ');
            Assert(!EndOfInput());
            Assert(!IsAlpha());
            Assert(!IsIdentifierChar());
            Assert(IsWhiteSpace());
            Assert(IsSpace());
            Assert(!IsEndLine());
            Assert(!IsDigit());
            Inc();

            Assert(Current() == '\t');
            Assert(!EndOfInput());
            Assert(!IsAlpha());
            Assert(!IsIdentifierChar());
            Assert(IsWhiteSpace());
            Assert(!IsSpace());
            Assert(!IsEndLine());
            Assert(!IsDigit());
            Inc();

            Assert(Current() == '\r');
            Assert(!EndOfInput());
            Assert(!IsAlpha());
            Assert(!IsIdentifierChar());
            Assert(IsWhiteSpace());
            Assert(!IsSpace());
            Assert(!IsEndLine());
            Assert(!IsDigit());
            Inc();

            Assert(Current() == '\n');
            Assert(!EndOfInput());
            Assert(!IsAlpha());
            Assert(!IsIdentifierChar());
            Assert(!IsWhiteSpace());
            Assert(!IsSpace());
            Assert(IsEndLine());
            Assert(!IsDigit());
            Inc();

            Assert(Current() == '\f');
            Assert(!EndOfInput());
            Assert(!IsAlpha());
            Assert(!IsIdentifierChar());
            Assert(IsWhiteSpace());
            Assert(!IsSpace());
            Assert(!IsEndLine());
            Assert(!IsDigit());
            Inc();

            Assert(Current() == '\n');
            Assert(!EndOfInput());
            Assert(!IsAlpha());
            Assert(!IsIdentifierChar());
            Assert(!IsWhiteSpace());
            Assert(!IsSpace());
            Assert(IsEndLine());
            Assert(!IsDigit());
            Inc();

            Assert(Current() == '\0');
            Assert(EndOfInput());
            Assert(!IsAlpha());
            Assert(!IsIdentifierChar());
            Assert(!IsWhiteSpace());
            Assert(!IsSpace());
            Assert(IsEndLine());
            Assert(!IsDigit());
        }
    };

    Test(YamlTokenizer, TokenizerBasics) {
        const char*       t = "a1 \t\r\n\f\n";
        TokenizerBaseTest tbt(t, t + strlen(t));
        tbt.RunTest();
    }

    // We actually test the tokenizer here, but don't want to go through the trouble of token handling in most cases.

    Test(YamlTokenizer, Tokenizer1) {
        const char* test = "--- aap noot mies\n"
                           "\n"
                           "fruit: apple\n";

        Parser parser(test, test + strlen(test));
        Assert(!parser.eof(), "Unexpected EOF");
        parser.Tokenize();
        Assert(parser.indent() == 0);
        Assert(parser.key().equals("fruit"));
        Assert(parser.stringValue().equals("apple"));
        Assert(!parser.eof(), "Unexpected EOF");
        parser.Tokenize();
        Assert(parser.eof(), "Expected EOF");
    }

    Test(YamlTokenizer, Tokenizer2) {
        const char* test = "fruit: apple\n";

        Parser parser(test, test + strlen(test));
        Assert(!parser.eof(), "Unexpected EOF");
        parser.Tokenize();
        Assert(parser.indent() == 0);
        Assert(parser.key().equals("fruit"));
        Assert(parser.stringValue().equals("apple"));
        Assert(!parser.eof(), "Unexpected EOF");
        parser.Tokenize();
        Assert(parser.eof(), "Expected EOF");
    }

    Test(YamlTokenizer, Tokenizer3) {
        const char* test = "fruit: apple";

        Parser parser(test, test + strlen(test));
        Assert(!parser.eof(), "Unexpected EOF");
        parser.Tokenize();
        Assert(parser.indent() == 0);
        Assert(parser.key().equals("fruit"));
        Assert(parser.stringValue().equals("apple"));
        Assert(!parser.eof(), "Unexpected EOF");
        parser.Tokenize();
        Assert(parser.eof(), "Expected EOF");
    }

    Test(YamlTokenizer, Tokenizer4) {
        const char* test = "aap:\n"
                           "  fruit: apple\n";

        Parser parser(test, test + strlen(test));
        Assert(!parser.eof(), "Unexpected EOF");
        parser.Tokenize();
        Assert(parser.indent() == 0);
        Assert(parser.key().equals("aap"), "Incorrect key");

        parser.Tokenize();
        auto k = parser.key();
        Assert(parser.indent() == 2);
        Assert(k.equals("fruit"));
        Assert(parser.stringValue().equals("apple"));
        Assert(!parser.eof(), "Unexpected EOF");
        parser.Tokenize();
        Assert(parser.eof(), "Expected EOF");
    }

    Test(YamlTokenizer, Tokenizer5) {
        const char* test = "--- aap noot mies\r\n"
                           "\r\n"
                           "fruit: apple\r\n";

        Parser parser(test, test + strlen(test));
        Assert(!parser.eof(), "Unexpected EOF");
        parser.Tokenize();
        Assert(parser.indent() == 0);
        Assert(parser.key().equals("fruit"));
        Assert(parser.stringValue().equals("apple"));
        Assert(!parser.eof(), "Unexpected EOF");
        parser.Tokenize();
        Assert(parser.eof(), "Expected EOF");
    }

    Test(YamlTokenizer, Tokenizer6) {
        const char* test = "fruit: apple\r\n";

        Parser parser(test, test + strlen(test));
        Assert(!parser.eof(), "Unexpected EOF");
        parser.Tokenize();
        Assert(parser.indent() == 0);
        Assert(parser.key().equals("fruit"));
        Assert(parser.stringValue().equals("apple"));
        Assert(!parser.eof(), "Unexpected EOF");
        parser.Tokenize();
        Assert(parser.eof(), "Expected EOF");
    }

    Test(YamlTokenizer, Tokenizer7) {
        const char* test = "aap:\r\n"
                           "  fruit: apple\r\n";

        Parser parser(test, test + strlen(test));
        Assert(!parser.eof(), "Unexpected EOF");
        parser.Tokenize();
        Assert(parser.indent() == 0);
        Assert(parser.key().equals("aap"), "Incorrect key");

        parser.Tokenize();
        auto k = parser.key();
        Assert(parser.indent() == 2);
        Assert(k.equals("fruit"));
        Assert(parser.stringValue().equals("apple"));
        Assert(!parser.eof(), "Unexpected EOF");
        parser.Tokenize();
        Assert(parser.eof(), "Expected EOF");
    }

    Test(YamlTokenizer, Tokenizer8) {
        const char* test = "aap:\n"
                           "\tfruit: apple\n";

        Parser parser(test, test + strlen(test));
        Assert(!parser.eof(), "Unexpected EOF");
        try {
            parser.Tokenize();
            Assert(parser.key().equals("aap"), "Incorrect key");

            parser.Tokenize();
            auto k = parser.key();
            Assert(k.equals("fruit"));
            Assert(false, "Tabs are not allowed in yaml for indentation; parse exception expected.");
        } catch (ParseException) {
            // OK, this is expected.
        }
    }

    Test(YamlTokenizer, Tokenizer9) {
        const char* test = "aap: #comment1\r\n"
                           "  fruit: apple\r\n"
                           "  #comment2\r\n"
                           "  fruit2: apple2\r\n";

        Parser parser(test, test + strlen(test));
        Assert(!parser.eof(), "Unexpected EOF");
        parser.Tokenize();
        Assert(parser.indent() == 0);
        Assert(parser.key().equals("aap"), "Incorrect key");

        parser.Tokenize();
        auto k = parser.key();
        Assert(parser.indent() == 2);
        Assert(k.equals("fruit"));
        Assert(parser.stringValue().equals("apple"));
        Assert(!parser.eof(), "Unexpected EOF");
        parser.Tokenize();
        auto k2 = parser.key();
        Assert(parser.indent() == 2);
        Assert(k2.equals("fruit2"));
        Assert(parser.stringValue().equals("apple2"));
        Assert(!parser.eof(), "Unexpected EOF");
        parser.Tokenize();
        Assert(parser.eof(), "Expected EOF");
    }

    void TestIncorrectYaml(const char* test) {
        Parser parser(test, test + strlen(test));
        try {
            while (!parser.eof()) {
                parser.Tokenize();
            }
            Assert(false, "Expected parser to fail.");
        } catch (ParseException) {}
    }

    Test(YamlTokenizer, IncorrectTokenizer1) {
        TestIncorrectYaml("aap#noot#mies:\n"
                          "fruit: banana\n");
    }

    Test(YamlTokenizer, IncorrectTokenizer2) {
        // # is not a valid identifier token.
        TestIncorrectYaml("fruit#wrong: banana\n"); }
    Test(YamlTokenizer, IncorrectTokenizer3) {
        // : is missing
        TestIncorrectYaml("aap  \n"
                          "  fruit: banana\n");
    }

    Test(YamlTokenizer, IncorrectTokenizer4) {
        // Incorrect quotation
        TestIncorrectYaml("aap:\n"
                          "  fruit: 'string\n");
    }

    Test(YamlTokenizer, IncorrectTokenizer5) {
        // Incorrect quotation
        TestIncorrectYaml("aap:\n"
                          "  fruit: \"string\n");
    }

}
