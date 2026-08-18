// Copyright (c) 2026 -  FluidNC contributors
// Use of this source code is governed by a GPLv3 license that can be found in the LICENSE file.

#include "USBSerialJTAGChannel.h"

#ifdef CONFIG_ESP_CONSOLE_USB_SERIAL_JTAG

#    include "Serial.h"  // allChannels
#    include "Logging.h"

#    include "driver/usb_serial_jtag.h"
#    include "driver/usb_serial_jtag_vfs.h"

// The driver rejects an RX ring that is not larger than the 64 byte endpoint.
static const uint32_t driverRxBufferSize = 1024;
static const uint32_t driverTxBufferSize = 1024;

// How long a write waits for room in the driver's TX ring.  With no host attached, or one that has
// stopped reading, the ring stays full indefinitely; console output is dropped rather than allowed
// to hold up gcode execution for a listener that may not exist.
static const TickType_t writeTimeout = pdMS_TO_TICKS(10);

USBSerialJTAGChannel::USBSerialJTAGChannel(bool addCR) : Channel("usb_serial_jtag", addCR) {
    _lineedit = new Lineedit(this, _line, Channel::maxLine - 1);
}

void USBSerialJTAGChannel::init() {
    usb_serial_jtag_driver_config_t cfg = {
        .tx_buffer_size = driverTxBufferSize,
        .rx_buffer_size = driverRxBufferSize,
    };

    esp_err_t err = usb_serial_jtag_driver_install(&cfg);
    if (err != ESP_OK) {
        // There is no channel to log to yet, and the ROM console still works at this point, so the
        // failure has to go out through stdio.
        ::printf("USB-Serial-JTAG driver install failed: %d\n", (int)err);
        return;
    }
    _installed = true;

    // Point stdio at the driver too.  ESP_LOG otherwise writes straight to the hardware FIFO, which
    // would interleave with this channel's writes partway through a line.
    usb_serial_jtag_vfs_use_driver();

    allChannels.registration(this);

    // Announce the restart the way UartChannel does, so a listener sees the [MSG:RST] it expects.
    // The leading newline ends any partial line left behind by the bootloader.
    print("\n");
    log_msg_to(*this, "RST");
}

size_t USBSerialJTAGChannel::fill() {
    if (!_installed) {
        return _count;
    }

    // Two reads at most: one to the end of the ring, one wrapped round to _tail.
    for (int pass = 0; pass < 2 && _count < rxBufferSize; pass++) {
        size_t chunk = rxBufferSize - _head;
        if (chunk > rxBufferSize - _count) {
            chunk = rxBufferSize - _count;
        }

        int got = usb_serial_jtag_read_bytes(&_rx[_head], chunk, 0);
        if (got <= 0) {
            break;
        }
        _head = (_head + size_t(got)) % rxBufferSize;
        _count += size_t(got);

        if (size_t(got) < chunk) {
            // A short read means the driver had nothing left, so wrapping round would be wasted.
            break;
        }
    }
    return _count;
}

size_t USBSerialJTAGChannel::write(uint8_t c) {
    if (!_installed) {
        return 0;
    }
    int written = usb_serial_jtag_write_bytes(&c, 1, writeTimeout);
    return written > 0 ? size_t(written) : 0;
}

size_t USBSerialJTAGChannel::write(const uint8_t* buffer, size_t length) {
    if (!_installed || length == 0) {
        return 0;
    }

    if (!_addCR) {
        int written = usb_serial_jtag_write_bytes(buffer, length, writeTimeout);
        return written > 0 ? size_t(written) : 0;
    }

    // Replace \n with \r\n, in bounded chunks as UartChannel does.
    size_t rem      = length;
    size_t j        = 0;
    char   lastchar = '\0';
    while (rem) {
        const size_t bufsize = 80;
        uint8_t      modbuf[bufsize];
        // bufsize-1 in case the last character is \n
        size_t k = 0;
        while (rem && k < (bufsize - 1)) {
            char c = char(buffer[j++]);
            if (c == '\n' && lastchar != '\r') {
                modbuf[k++] = '\r';
            }
            lastchar    = c;
            modbuf[k++] = uint8_t(c);
            --rem;
        }
        usb_serial_jtag_write_bytes(modbuf, k, writeTimeout);
    }
    return length;
}

int USBSerialJTAGChannel::available() {
    return int(fill());
}

int USBSerialJTAGChannel::peek() {
    if (fill() == 0) {
        return -1;
    }
    return _rx[_tail];
}

int USBSerialJTAGChannel::read() {
    if (fill() == 0) {
        return -1;
    }
    uint8_t c = _rx[_tail];
    _tail     = (_tail + 1) % rxBufferSize;
    --_count;
    return c;
}

int USBSerialJTAGChannel::rx_buffer_available() {
    // Only the staging buffer is counted, ignoring the driver's larger ring behind it, so a sender
    // that respects this figure cannot overrun us.
    return int(rxBufferSize - _count);
}

void USBSerialJTAGChannel::flushRx() {
    // Discard what the driver still holds as well, otherwise a reset would be followed by whatever
    // was typed before it.
    if (_installed) {
        uint8_t discard[64];
        while (usb_serial_jtag_read_bytes(discard, sizeof(discard), 0) > 0) {}
    }
    _head  = 0;
    _tail  = 0;
    _count = 0;
    Channel::flushRx();
}

size_t USBSerialJTAGChannel::timedReadBytes(char* buffer, size_t length, TickType_t timeout) {
    size_t remlen = length;

    // Anything pushed into the queue comes first.  As in UartChannel this is normally empty, because
    // timedReadBytes is only used when the channel is not carrying gcode.
    while (remlen && _queue.size()) {
        *buffer++ = char(_queue.front());
        _queue.pop();
        --remlen;
    }

    // Then whatever peek() or available() has already staged.
    while (remlen && _count) {
        *buffer++ = char(_rx[_tail]);
        _tail     = (_tail + 1) % rxBufferSize;
        --_count;
        --remlen;
    }

    // Only then wait on the driver, once, for the remainder.
    if (remlen && _installed) {
        int got = usb_serial_jtag_read_bytes(buffer, remlen, timeout);
        if (got > 0) {
            remlen -= size_t(got);
        }
    }

    return length - remlen;
}

bool USBSerialJTAGChannel::realtimeOkay(char c) {
    return _lineedit->realtime(c);
}

bool USBSerialJTAGChannel::lineComplete(char* line, char c) {
    if (_lineedit->step(c)) {
        _linelen        = _lineedit->finish();
        _line[_linelen] = '\0';
        strcpy(line, _line);
        _linelen = 0;
        return true;
    }
    return false;
}

#endif
