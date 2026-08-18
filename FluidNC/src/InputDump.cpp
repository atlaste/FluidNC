// Copyright (c) 2026 -  FluidNC contributors
// Use of this source code is governed by a GPLv3 license that can be found in the LICENSE file.

#include "InputDump.h"

#include "Channel.h"
#include "Config.h"
#include "Driver/fluidnc_gpio.h"
#include "Extenders/Extenders.h"
#include "Logging.h"
#include "Machine/MachineConfig.h"
#include "Module.h"
#include "Pin.h"
#include "Pins/GPIOPinDetail.h"
#include "Serial.h"  // allChannels

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include <algorithm>
#include <string>
#include <vector>

namespace {
    // How often a row goes out.  Fast enough to catch a switch being pressed by hand, slow enough to
    // read the table as it scrolls and to cost a shared I2C bus nothing worth counting.
    const int32_t rowIntervalMs = 250;

    // Rows between repeats of the column names, so that the header a table this wide needs is never
    // far above whatever is on screen.
    const int rowsPerHeader = 20;

    // Columns are grouped so that one can be counted off against the header without having to follow
    // a long unbroken run of identical characters.
    const size_t columnsPerGroup = 8;

    // Places one character per column with the grouping every line of the table shares, so a column
    // always lands in the same place whether the line is a header, a separator or a row of values.
    template <typename CharForColumn>
    std::string layout(size_t columns, CharForColumn charFor) {
        std::string line;
        for (size_t i = 0; i < columns; ++i) {
            if (i != 0 && (i % columnsPerGroup) == 0) {
                line += ' ';
            }
            line += charFor(i);
        }
        return line;
    }

    class InputDumpModule : public Module {
        struct Column {
            std::string name;
            Pin         pin;

            // Set for a native GPIO so that stopping can put the pad back out of use.  Extender pins
            // have nothing to undo; releasing the claim is enough.
            int gpio = -1;
        };

        std::vector<Column> _columns;

        Channel* _out             = nullptr;
        int32_t  _nextRow         = 0;
        int      _rowsUntilHeader = 0;

        void collect();
        void add(const std::string& name, Pins::PinAttributes attr, int gpio);
        void writeHeader();
        void writeRow();

    public:
        InputDumpModule(const char* name);

        void poll() override;

        bool running() const { return _out != nullptr; }
        void start(Channel& out);
        void stop();
    };

    InputDumpModule* theDump = nullptr;

    InputDumpModule::InputDumpModule(const char* name) : Module(name) {
        theDump = this;
    }

    void InputDumpModule::add(const std::string& name, Pins::PinAttributes attr, int gpio) {
        Column column;
        column.name = name;
        column.gpio = gpio;

        // Building a pin throws if it turns out not to be usable after all.  One pin that has to be
        // left out is no reason to abandon the other forty, and Pin::create has already said why.
        try {
            column.pin = Pin::create(name);
            column.pin.setAttr(attr);
        } catch (...) {
            return;
        }

        _columns.push_back(std::move(column));
    }

    void InputDumpModule::collect() {
        // Pins the configuration has taken are left out.  They are the ones already known, and more to
        // the point reconfiguring one would disturb whatever owns it - which on a board like this means
        // the I2S shift register chain, the I2C bus and the SPI bus, none of which would survive having
        // a pin turned into a pulled-up input underneath them.
        for (pinnum_t gpio = 0; gpio < MAX_N_GPIO; ++gpio) {
            auto capabilities = Pins::GPIOPinDetail::defaultCapabilities(gpio);
            if (!capabilities.has(Pins::PinCapabilities::Input) || Pins::GPIOPinDetail::isClaimed(gpio) ||
                gpio_reserved_by_platform(gpio)) {
                continue;
            }

            // Pulled up, so an unconnected pin sits at a steady X and a wire pulled to ground reads as
            // _.  Left floating it would report noise, and a column of noise cannot be told apart from
            // a column that is responding to something.
            auto attributes = Pin::Attr::Input;
            if (capabilities.has(Pins::PinCapabilities::PullUp)) {
                attributes = attributes | Pin::Attr::PullUp;
            }

            add("gpio." + std::to_string(int(gpio)), attributes, gpio);
        }

        // Pin names only carry a single extender digit, so pinext0 through pinext9 is the whole range
        // that can be addressed.
        auto extenders = config->_extenders;
        if (extenders == nullptr) {
            return;
        }
        for (int device = 0; device < 10; ++device) {
            auto holder = extenders->_pinDrivers[device];
            if (holder == nullptr || holder->_driver == nullptr) {
                continue;
            }

            auto driver = holder->_driver;
            for (pinnum_t index = 0; index < driver->pinCount(); ++index) {
                if (!driver->pinAvailable(index)) {
                    continue;
                }

                // No pull-up to ask for here; whether an extender pin has one is the chip's business
                // and not something the pin system can express.
                add("pinext" + std::to_string(device) + "." + std::to_string(int(index)), Pin::Attr::Input, -1);
            }
        }
    }

    void InputDumpModule::writeHeader() {
        size_t width = 0;
        for (auto& column : _columns) {
            width = std::max(width, column.name.size());
        }

        // The names read downward above their columns, there being no room to write them across a table
        // this wide.  They are bottom-aligned so that the pin numbers, the part that distinguishes one
        // column from its neighbour, all land on the last line or two.
        for (size_t row = 0; row < width; ++row) {
            auto line = layout(_columns.size(), [&](size_t i) {
                const std::string& name = _columns[i].name;
                size_t             top  = width - name.size();
                return row >= top ? name[row - top] : ' ';
            });
            *_out << line.c_str() << '\n';
        }

        *_out << layout(_columns.size(), [](size_t) { return '-'; }).c_str() << '\n';
    }

    void InputDumpModule::writeRow() {
        auto line = layout(_columns.size(), [&](size_t i) { return _columns[i].pin.read() ? 'X' : '_'; });
        *_out << line.c_str() << '\n';
    }

    void InputDumpModule::start(Channel& out) {
        stop();  // Re-running the command moves the dump to this channel rather than claiming twice.

        collect();
        if (_columns.empty()) {
            log_error_to(out, "No unused input pins to watch");
            return;
        }

        _out             = &out;
        _nextRow         = int32_t(xTaskGetTickCount() * portTICK_PERIOD_MS);
        _rowsUntilHeader = 0;

        log_stream(out, "Diag/Inputs: watching " << _columns.size() << " unused pins, X = high, _ = low");
        log_string(out, "Pins the configuration uses are not listed; run $Diag/Inputs again to stop");
    }

    void InputDumpModule::stop() {
        _out = nullptr;

        for (auto& column : _columns) {
            // Destroying the pin releases the claim but leaves the pad configured, so hand a native one
            // back in the state of a pin nothing is using: input path off and no pull.
            column.pin = Pin();
            if (column.gpio >= 0) {
                gpio_mode(pinnum_t(column.gpio), false, false, false, false, false);
            }
        }
        _columns.clear();
    }

    void InputDumpModule::poll() {
        if (_out == nullptr) {
            return;
        }

        // A telnet or websocket channel is destroyed when its connection closes, taking with it the
        // only thing that would have stopped the dump.
        if (!allChannels.isRegistered(_out)) {
            stop();
            return;
        }

        int32_t now = int32_t(xTaskGetTickCount() * portTICK_PERIOD_MS);
        if ((now - _nextRow) < 0) {
            return;
        }
        _nextRow = now + rowIntervalMs;

        if (_rowsUntilHeader <= 0) {
            writeHeader();
            _rowsUntilHeader = rowsPerHeader;
        }
        --_rowsUntilHeader;

        writeRow();
    }

    ModuleFactory::InstanceBuilder<InputDumpModule> input_dump_module("input_dump", true);
}

namespace InputDump {
    bool running() {
        return theDump != nullptr && theDump->running();
    }

    void start(Channel& out) {
        if (theDump) {
            theDump->start(out);
        }
    }

    void stop() {
        if (theDump) {
            theDump->stop();
        }
    }
}
