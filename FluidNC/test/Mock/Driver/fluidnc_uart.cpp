// Mock implementation of fluidnc_uart.h for unit tests.
// Delegates to UartMock callbacks or default buffer behavior.
// Use of this source code is governed by a GPLv3 license that can be found in the LICENSE file.

#include "Driver/fluidnc_uart.h"
#include "Driver/fluidnc_uart_mock.h"

void uart_init(uint32_t uart_num) {
    auto& m = UartMock::instance();
    if (m.onInit) { m.onInit(uart_num); }
}

void uart_mode(uint32_t uart_num, uint32_t baud, UartData dataBits, UartParity parity, UartStop stopBits) {
    auto& m = UartMock::instance();
    if (m.onMode) { m.onMode(uart_num, baud, uint32_t(dataBits), uint32_t(parity), uint32_t(stopBits)); }
}

bool uart_half_duplex(uint32_t uart_num) {
    auto& m = UartMock::instance();
    if (m.onHalfDuplex) { return m.onHalfDuplex(uart_num); }
    return false;  // success (no error)
}

int uart_read(uint32_t uart_num, uint8_t* buf, uint32_t len, uint32_t timeout_ms) {
    auto& m = UartMock::instance();
    if (m.onRead) { return m.onRead(uart_num, buf, len, timeout_ms); }
    return m.defaultRead(uart_num, buf, len, timeout_ms);
}

int uart_write(uint32_t uart_num, const uint8_t* buf, size_t len) {
    auto& m = UartMock::instance();
    if (m.onWrite) { return m.onWrite(uart_num, buf, len); }
    return m.defaultWrite(uart_num, buf, len);
}

void uart_xon(uint32_t) {}
void uart_xoff(uint32_t) {}
void uart_sw_flow_control(uint32_t, bool, uint32_t, uint32_t) {}

bool uart_pins(uint32_t uart_num, pinnum_t tx_pin, pinnum_t rx_pin, pinnum_t rts_pin, pinnum_t cts_pin) {
    auto& m = UartMock::instance();
    if (m.onPins) { return m.onPins(uart_num, tx_pin, rx_pin, rts_pin, cts_pin); }
    return false;  // success (no error)
}

int uart_buflen(uint32_t uart_num) {
    return UartMock::instance().available(uart_num);
}

int uart_bufavail(uint32_t uart_num) {
    // Return a large number to indicate space available for writing
    return 1024;
}

void uart_discard_input(uint32_t uart_num) {
    auto& m = UartMock::instance();
    if (m.onDiscardInput) { m.onDiscardInput(uart_num); return; }
    // Default: clear RX buffer
    if (uart_num < UartMock::MAX_UARTS) {
        std::lock_guard<std::mutex> lock(m.buffers[uart_num].mtx);
        m.buffers[uart_num].rx.clear();
    }
}

bool uart_wait_output(uint32_t uart_num, uint32_t timeout_ms) {
    auto& m = UartMock::instance();
    if (m.onWaitOutput) { return m.onWaitOutput(uart_num, timeout_ms); }
    return true;  // output flushed
}

void uart_register_input_pin(uint32_t, pinnum_t, InputPin*) {}
