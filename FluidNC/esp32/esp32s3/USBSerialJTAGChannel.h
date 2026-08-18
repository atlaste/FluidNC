// Copyright (c) 2026 -  FluidNC contributors
// Use of this source code is governed by a GPLv3 license that can be found in the LICENSE file.
//
// Console channel over the ESP32-S3's USB-Serial-JTAG peripheral, for boards whose only link to a
// host is the chip's native USB port.  UART0 is useless as a console on such a board: nothing is
// wired to its pins, so FluidNC's log output and every gcode reply are driven into GPIOs that go
// nowhere, while the ESP_LOG output still appears over USB and makes it look as if the firmware
// simply stopped.
//
// USB-Serial-JTAG is not the peripheral that USBCDCChannel drives; that one is USB-OTG, through
// TinyUSB.  The two share a single USB PHY and only one can own the port at a time, so starting
// TinyUSB switches the PHY over and takes the ROM console down with it, losing both esptool's reset
// and the ESP_LOG output on the one connector the board has.  Speaking to USB-Serial-JTAG directly
// keeps flashing, the IDF log and this channel all on that connector.

#pragma once

#include <sdkconfig.h>

#ifdef CONFIG_ESP_CONSOLE_USB_SERIAL_JTAG

#    include "Channel.h"
#    include "lineedit.h"

class USBSerialJTAGChannel : public Channel {
public:
    explicit USBSerialJTAGChannel(bool addCR = false);

    void init() override;

    size_t write(uint8_t c) override;
    size_t write(const uint8_t* buffer, size_t length) override;

    int available() override;
    int peek() override;
    int read() override;

    int  rx_buffer_available() override;
    void flushRx() override;

    size_t timedReadBytes(char* buffer, size_t length, TickType_t timeout) override;
    size_t timedReadBytes(uint8_t* buffer, size_t length, TickType_t timeout) {
        return timedReadBytes(reinterpret_cast<char*>(buffer), length, timeout);
    }

    bool realtimeOkay(char c) override;
    bool lineComplete(char* line, char c) override;

private:
    // The driver consumes whatever it hands back, but Channel needs peek(), so received bytes are
    // staged here before anything looks at them.
    static constexpr size_t rxBufferSize = 256;

    uint8_t _rx[rxBufferSize] = {};
    size_t  _head             = 0;
    size_t  _tail             = 0;
    size_t  _count            = 0;

    bool _installed = false;

    Lineedit* _lineedit;

    // Move whatever the driver is holding into _rx without blocking, and return the bytes staged.
    size_t fill();
};

#endif
