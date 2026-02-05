#include "TestFramework.h"

#include <Configuration/Tokenizer.h>
#include <Configuration/Parser.h>

namespace Configuration {
    // Helper to check if parser is at EOF
    inline bool isEof(const Parser& p) { return p._token._state == TokenState::Eof; }

    Test(YamlComplete, Test) {
        const char* config = "name: \"ESP32 Dev Controller V4\"\n"
                             "board: \"ESP32 Dev Controller V4\"\n"
                             "yaml_wiki: \"https://github.com/bdring/FluidNC/wiki/YAML-Config-File\"\n"
                             "\n"
                             "idle_time: 250\n"
                             "engine: rmt\n"
                             "dir_delay_microseconds: 1\n"
                             "pulse_microseconds: 2\n"
                             "disable_delay_us: 0\n"
                             "homing_init_lock: false\n"
                             "\n"
                             "axes:\n"
                             "  number_axis: 3\n"
                             "  shared_stepper_disable_pin: gpio.13:low\n"
                             "  \n"
                             "  x:\n"
                             "\n"
                             "  y:\n"
                             "\n"
                             "  z:\n"
                             "\n"
                             "\n"
                             "coolant:\n"
                             "  flood: gpio.25:low\n"
                             "  mist:  gpio.21\n"
                             "\n"
                             "comms:\n"
                             "    wifi_sta:\n"
                             "        ssid: StefanMieke\n"
                             "\n"
                             "    wifi_ap:\n"
                             "        ip_address: \"192.168.0.1\"\n"
                             "        ssid: ScratchThat\n"
                             "        \n"
                             "probe:\n"
                             "    pin: gpio.32:high:pu\n"
                             "\n";

        Parser p(config);
        p.Tokenize();
        {
            Assert(!isEof(p), "No EOF expected");
            Assert(p.key() == "name", "Expected 'name'");
            p.Tokenize();

            Assert(!isEof(p), "No EOF expected");
            Assert(p.key() == "board", "Expected 'board'");
            p.Tokenize();

            Assert(!isEof(p), "No EOF expected");
            Assert(p.key() == "yaml_wiki", "Expected 'yaml_wiki'");
            p.Tokenize();

            Assert(!isEof(p), "No EOF expected");
            Assert(p.key() == "idle_time", "Expected 'idle_time'");
            p.Tokenize();

            Assert(!isEof(p), "No EOF expected");
            Assert(p.key() == "engine", "Expected 'engine'");
            p.Tokenize();

            Assert(!isEof(p), "No EOF expected");
            Assert(p.key() == "dir_delay_microseconds", "Expected 'dir_delay_microseconds'");
            p.Tokenize();

            Assert(!isEof(p), "No EOF expected");
            Assert(p.key() == "pulse_microseconds", "Expected 'pulse_microseconds'");
            p.Tokenize();

            Assert(!isEof(p), "No EOF expected");
            Assert(p.key() == "disable_delay_us", "Expected 'disable_delay_us'");
            p.Tokenize();

            Assert(!isEof(p), "No EOF expected");
            Assert(p.key() == "homing_init_lock", "Expected 'homing_init_lock'");
            p.Tokenize();

            Assert(!isEof(p), "No EOF expected");
            Assert(p.key() == "axes", "Expected 'axes'");
            {
                p.Tokenize();

                Assert(!isEof(p), "No EOF expected");
                Assert(p.key() == "number_axis", "Expected 'number_axis'");
                p.Tokenize();
                Assert(!isEof(p), "No EOF expected");
                Assert(p.key() == "shared_stepper_disable_pin", "Expected 'shared_stepper_disable_pin'");
                p.Tokenize();
                Assert(!isEof(p), "No EOF expected");
                Assert(p.key() == "x", "Expected 'x'");
                p.Tokenize();
                Assert(!isEof(p), "No EOF expected");
                Assert(p.key() == "y", "Expected 'y'");
                p.Tokenize();
                Assert(!isEof(p), "No EOF expected");
                Assert(p.key() == "z", "Expected 'z'");
            }
            Assert(!isEof(p), "No EOF expected");
            p.Tokenize();

            Assert(p.key() == "coolant", "Expected 'coolant'");
            {
                p.Tokenize();

                Assert(!isEof(p), "No EOF expected");
                Assert(p.key() == "flood", "Expected 'flood'");
                p.Tokenize();
                Assert(!isEof(p), "No EOF expected");
                Assert(p.key() == "mist", "Expected 'mist'");
            }
            Assert(!isEof(p), "No EOF expected");
            p.Tokenize();

            Assert(p.key() == "comms", "Expected 'comms'");
            {
                p.Tokenize();
                Assert(!isEof(p), "No EOF expected");
                Assert(p.key() == "wifi_sta", "Expected 'wifi_sta'");
                {
                    p.Tokenize();
                    Assert(!isEof(p), "No EOF expected");
                    Assert(p.key() == "ssid", "Expected 'ssid'");
                }
                Assert(!isEof(p), "No EOF expected");
                p.Tokenize();
                Assert(p.key() == "wifi_ap", "Expected 'wifi_ap'");
                {
                    p.Tokenize();
                    Assert(!isEof(p), "No EOF expected");
                    Assert(p.key() == "ip_address", "Expected 'ip_address'");
                    p.Tokenize();
                    Assert(!isEof(p), "No EOF expected");
                    Assert(p.key() == "ssid", "Expected 'ssid'");
                }
            }
            Assert(!isEof(p), "No EOF expected");
            p.Tokenize();

            Assert(p.key() == "probe", "Expected 'probe'");
            {
                p.Tokenize();
                Assert(!isEof(p), "No EOF expected");
                Assert(p.key() == "pin", "Expected 'pin'");
            }
            p.Tokenize();
            Assert(isEof(p), "EOF expected");
        }
    }
}
