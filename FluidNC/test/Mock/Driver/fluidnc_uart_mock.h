// Mock for fluidnc_uart.h -- callback-based UART mock for VFD spindle tests.
// Use of this source code is governed by a GPLv3 license that can be found in the LICENSE file.

#pragma once

#include <cstdint>
#include <cstring>
#include <functional>
#include <mutex>
#include <vector>

class UartMock {
public:
    // --- Callbacks (tests override these to interact with VFD communication) ---

    std::function<void(uint32_t uart_num)>                                                                             onInit;
    std::function<void(uint32_t uart_num, uint32_t baud, uint32_t dataBits, uint32_t parity, uint32_t stopBits)>       onMode;
    std::function<bool(uint32_t uart_num)>                                                                             onHalfDuplex;
    std::function<int(uint32_t uart_num, uint8_t* buf, uint32_t len, uint32_t timeout_ms)>                             onRead;
    std::function<int(uint32_t uart_num, const uint8_t* buf, size_t len)>                                              onWrite;
    std::function<bool(uint32_t uart_num, int tx_pin, int rx_pin, int rts_pin, int cts_pin)>                           onPins;
    std::function<void(uint32_t uart_num)>                                                                             onDiscardInput;
    std::function<bool(uint32_t uart_num, uint32_t timeout_ms)>                                                        onWaitOutput;

    // --- Default buffer behavior ---
    // Per-UART RX buffers: test loads data here, uart_read consumes it.
    // Per-UART TX buffers: uart_write appends here, test inspects it.

    static constexpr int MAX_UARTS = 4;

    struct UartBuffer {
        std::vector<uint8_t> rx;  // data available for reading (test loads this)
        std::vector<uint8_t> tx;  // data written by production code (test inspects this)
        std::mutex           mtx;
    };

    UartBuffer buffers[MAX_UARTS];

    // Load response data into the RX buffer for a given UART (test calls this before VFD reads)
    void loadResponse(uint32_t uart_num, const uint8_t* data, size_t len) {
        if (uart_num >= MAX_UARTS) return;
        std::lock_guard<std::mutex> lock(buffers[uart_num].mtx);
        buffers[uart_num].rx.insert(buffers[uart_num].rx.end(), data, data + len);
    }

    // Get what was written to a UART TX buffer (test reads this to verify VFD commands)
    std::vector<uint8_t> getWritten(uint32_t uart_num) {
        if (uart_num >= MAX_UARTS) return {};
        std::lock_guard<std::mutex> lock(buffers[uart_num].mtx);
        return buffers[uart_num].tx;
    }

    // Clear the TX buffer after inspection
    void clearWritten(uint32_t uart_num) {
        if (uart_num >= MAX_UARTS) return;
        std::lock_guard<std::mutex> lock(buffers[uart_num].mtx);
        buffers[uart_num].tx.clear();
    }

    // Get number of bytes available to read
    int available(uint32_t uart_num) {
        if (uart_num >= MAX_UARTS) return 0;
        std::lock_guard<std::mutex> lock(buffers[uart_num].mtx);
        return int(buffers[uart_num].rx.size());
    }

    // Reset everything
    void reset() {
        onInit         = nullptr;
        onMode         = nullptr;
        onHalfDuplex   = nullptr;
        onRead         = nullptr;
        onWrite        = nullptr;
        onPins         = nullptr;
        onDiscardInput = nullptr;
        onWaitOutput   = nullptr;
        for (int i = 0; i < MAX_UARTS; ++i) {
            std::lock_guard<std::mutex> lock(buffers[i].mtx);
            buffers[i].rx.clear();
            buffers[i].tx.clear();
        }
    }

    // --- Default read: consume from RX buffer ---
    int defaultRead(uint32_t uart_num, uint8_t* buf, uint32_t len, uint32_t /*timeout_ms*/) {
        if (uart_num >= MAX_UARTS) return 0;
        std::lock_guard<std::mutex> lock(buffers[uart_num].mtx);
        auto&  rx  = buffers[uart_num].rx;
        size_t n   = (len < rx.size()) ? len : rx.size();
        if (n > 0) {
            memcpy(buf, rx.data(), n);
            rx.erase(rx.begin(), rx.begin() + n);
        }
        return int(n);
    }

    // --- Default write: append to TX buffer ---
    int defaultWrite(uint32_t uart_num, const uint8_t* buf, size_t len) {
        if (uart_num >= MAX_UARTS) return 0;
        std::lock_guard<std::mutex> lock(buffers[uart_num].mtx);
        buffers[uart_num].tx.insert(buffers[uart_num].tx.end(), buf, buf + len);
        return int(len);
    }

    static UartMock& instance() {
        static UartMock inst;
        return inst;
    }

private:
    UartMock() = default;
};
