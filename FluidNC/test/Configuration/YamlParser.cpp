#include "TestFramework.h"

#include <Configuration/Tokenizer.h>
#include <Configuration/Parser.h>

namespace Configuration {
    // Helper to check if parser is at EOF
    inline bool isEof(const Parser& p) { return p._token._state == TokenState::Eof; }

    Test(YamlParser, BasicProperties) {
        const char* config = "a: aap\n"
                             "b: banaan\n"
                             "\n"
                             "c: chocolade\n";

        Parser p(config);
        p.Tokenize();
        {
            Assert(p.key() == "a", "Expected 'a'");
            Assert(p.stringValue() == "aap", "Expected 'aap'");
        }

        p.Tokenize();
        {
            Assert(p.key() == "b", "Expected 'b'");
            Assert(p.stringValue() == "banaan", "Expected 'banaan'");
        }

        p.Tokenize();
        {
            Assert(p.key() == "c", "Expected 'c'");
            Assert(p.stringValue() == "chocolade", "Expected 'chocolade'");
        }
        p.Tokenize();
        Assert(isEof(p), "EOF failed");
    }

    Test(YamlParser, SimpleSection) {
        const char* config = "a: aap\n"
                             "s:\n"
                             "  b: banaan\n"
                             "\n"
                             "c: chocolade\n";

        Parser p(config);
        p.Tokenize();
        {
            Assert(p.key() == "a", "Expected 'a'");
            Assert(p.stringValue() == "aap", "Expected 'aap'");
        }

        p.Tokenize();
        {
            Assert(p.key() == "s", "Expected 's'");
            {
                p.Tokenize();
                Assert(p.key() == "b", "Expected 'b'");
                Assert(p.stringValue() == "banaan", "Expected 'banaan'");
            }
        }

        p.Tokenize();
        {
            Assert(p.key() == "c", "Expected 'c'");
            Assert(p.stringValue() == "chocolade", "Expected 'chocolade'");
        }
        p.Tokenize();
        Assert(isEof(p), "EOF failed");
    }

    Test(YamlParser, TwoSequentialSections) {
        const char* config = "a: aap\n"
                             "s:\n"
                             "  b: banaan\n"
                             "t:\n"
                             "  c: chocolade\n"
                             "\n"
                             "w: wipwap\n";

        Parser p(config);
        p.Tokenize();
        {
            Assert(p.key() == "a", "Expected 'a'");
            Assert(p.stringValue() == "aap", "Expected 'aap'");
        }

        p.Tokenize();
        {
            Assert(p.key() == "s", "Expected 's'");
            p.Tokenize();
            {
                Assert(p.key() == "b", "Expected 'b'");
                Assert(p.stringValue() == "banaan", "Expected 'banaan'");
            }
        }

        p.Tokenize();
        {
            Assert(p.key() == "t", "Expected 't'");
            p.Tokenize();
            {
                Assert(p.key() == "c", "Expected 'c'");
                Assert(p.stringValue() == "chocolade", "Expected 'chocolade'");
            }
        }

        p.Tokenize();
        {
            Assert(p.key() == "w", "Expected 'w'");
            Assert(p.stringValue() == "wipwap", "Expected 'wipwap'");
        }
        p.Tokenize();
        Assert(isEof(p), "EOF failed");
    }

    Test(YamlParser, TwoSequentialSectionsInASection) {
        const char* config = "a: aap\n"
                             "r:\n"
                             "  s:\n"
                             "    b: banaan\n"
                             "    d: dinges\n"
                             "  t:\n"
                             "    c: chocolade\n"
                             "    e: eventjes\n"
                             "\n"
                             "w: wipwap\n";

        Parser p(config);
        p.Tokenize();
        {
            Assert(p.key() == "a", "Expected 'a'");
            Assert(p.stringValue() == "aap", "Expected 'aap'");
        }

        p.Tokenize();
        {
            Assert(p.key() == "r", "Expected 'r'");
            p.Tokenize();

            {
                Assert(p.key() == "s", "Expected 's'");
                p.Tokenize();
                {
                    Assert(p.key() == "b", "Expected 'b'");
                    Assert(p.stringValue() == "banaan", "Expected 'banaan'");
                }
                p.Tokenize();
                {
                    Assert(p.key() == "d", "Expected 'd'");
                    Assert(p.stringValue() == "dinges", "Expected 'dinges'");
                }
            }

            p.Tokenize();
            {
                Assert(p.key() == "t", "Expected 't'");
                p.Tokenize();
                {
                    Assert(p.key() == "c", "Expected 'c'");
                    Assert(p.stringValue() == "chocolade", "Expected 'chocolade'");
                }
                p.Tokenize();
                {
                    Assert(p.key() == "e", "Expected 'e'");
                    Assert(p.stringValue() == "eventjes", "Expected 'eventjes'");
                }
            }
        }

        p.Tokenize();
        {
            Assert(p.key() == "w", "Expected 'w'");
            Assert(p.stringValue() == "wipwap", "Expected 'wipwap'");
        }
        p.Tokenize();
        Assert(isEof(p), "EOF failed");
    }
}
