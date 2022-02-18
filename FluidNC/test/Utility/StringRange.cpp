#include "../TestFramework.h"

#include <src/StringRange.h>

namespace Configuration {
    Test(UtilStringRange, Basics) {
        {
            const char* test = "aap noot mies";
            StringRange sr(test, test + 3);
            Assert(!sr.equals(test));
            Assert(sr.equals("aap"));
            Assert(!sr.equals("aap "));
            Assert(!sr.equals("aa"));

            Assert(sr.str() == "aap");
            Assert(sr.length() == 3);
        }
        {
            const char* test = "aap noot mies";
            StringRange sr(test + 4, test + 4 + 4);
            Assert(!sr.equals(test));
            Assert(sr.equals("noot"));
            Assert(!sr.equals("noot "));

            Assert(sr.str() == "noot");
            Assert(sr.length() == 4);
        }
        {
            const char* test = "aap noot mies";
            StringRange sr(test);
            Assert(sr.equals(test));
            Assert(sr.equals("aap noot mies"));
            Assert(!sr.equals("aap "));
            Assert(!sr.equals("aa"));

            Assert(sr.str() == "aap noot mies");
            Assert(sr.length() == strlen(test));
        }
        {
            String      test = "aap noot mies";
            StringRange sr(test);
            Assert(sr.equals(test));
            Assert(sr.equals("aap noot mies"));
            Assert(!sr.equals("aap "));
            Assert(!sr.equals("aa"));

            Assert(sr.str() == "aap noot mies");
            Assert(sr.length() == test.length());
        }
        {
            const char* test = "aap noot mies";
            char        buf[100];
            strcpy(buf, test);

            StringRange sr(buf);
            Assert(sr.equals(test));
            Assert(sr.equals("aap noot mies"));
            Assert(!sr.equals("aap "));
            Assert(!sr.equals("aa"));

            Assert(sr.str() == "aap noot mies");
            Assert(sr.length() == strlen(test));

            buf[2] = 's';
            Assert(!sr.equals(test));
            Assert(sr.equals("aas noot mies"));
            Assert(!sr.equals("aas "));
            Assert(!sr.equals("aa"));

            Assert(sr.str() == "aas noot mies");
            Assert(sr.length() == strlen(test));
        }
        {
            StringRange sr;
            Assert(sr.equals(""));
            Assert(sr.str().equals(""));
            Assert(!sr.equals("aap "));
            Assert(!sr.equals("aa"));
            Assert(sr.length() == 0);
        }
    }

    Test(UtilStringRange, Substring) {
        const char* test = "aap noot mies";
        StringRange sr(test);

        {
            auto sr2 = sr.subString(0, 3);
            Assert(sr2.equals("aap"));
        }
        {
            auto sr2 = sr.subString(-1, 3);
            Assert(sr2.equals("aap"));
        }
        {
            auto sr2 = sr.subString(4, 4);
            Assert(sr2.equals("noot"));
        }
        {
            auto sr2 = sr.subString(9, 4);
            Assert(sr2.equals("mies"));
        }
        {
            auto sr2 = sr.subString(9, 10);
            Assert(sr2.equals("mies"));
        }
        {
            auto sr2 = sr.subString(100, 10);
            Assert(sr2.equals(""));
            Assert(sr2.str().equals(""));
        }

        Assert(sr.str() == "aap noot mies");
    }

    Test(UtilStringRange, Find) {
        const char* test = "aap noot mies";
        StringRange sr(test);
        Assert(sr.find('a') == 0);
        Assert(sr.find('p') == 2);
        Assert(sr.find('q') == -1);
    }

    Test(UtilStringRange, NextWord) {
        {
            const char* test = "aap noot mies";
            StringRange sr(test);
            auto        word = sr.nextWord();
            Assert(word.str() == "aap");

            word = sr.nextWord();
            Assert(word.str() == "noot");

            word = sr.nextWord();
            Assert(word.str() == "mies");

            word = sr.nextWord();
            Assert(word.length() == 0);
        }
        {
            const char* test = "aap noot mies";
            StringRange sr(test);
            auto        word = sr.nextWord('=');
            Assert(word.str() == "aap noot mies");

            word = sr.nextWord('=');
            Assert(word.length() == 0);
        }
        {
            const char* test = "aap=noot mies";
            StringRange sr(test);
            auto        word = sr.nextWord('=');
            Assert(word.str() == "aap");

            word = sr.nextWord('=');
            Assert(word.str() == "noot mies");

            word = sr.nextWord('=');
            Assert(word.length() == 0);
        }
    }
}
