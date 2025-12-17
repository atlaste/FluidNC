// Copyright (c) 2024 - Proper ESP-IDF SPI implementation for TMC drivers
// Use of this source code is governed by a GPLv3 license that can be found in the LICENSE file.

// This implementation uses the ESP-IDF SPI driver properly instead of
// directly accessing hardware registers. This prevents corruption of
// other SPI devices (SD card, FRAM, etc.) that share the same bus.
//
// The previous implementation bypassed ESP-IDF and wrote directly to
// SPI hardware registers, which corrupted the driver's internal state
// and caused all SPI devices to fail when TMC drivers were present.

#include <sdkconfig.h>

#ifdef CONFIG_IDF_TARGET_ESP32S3

#include "Logging.h"
#include "NutsBolts.h"

#include <TMCStepper.h>
#include <driver/spi_master.h>
#include <cstring>

#include <sdkconfig.h>
#ifdef CONFIG_IDF_TARGET_ESP32S3
#    define HSPI_HOST SPI2_HOST
#endif

// TMC SPI device handle - lazily initialized on first use
static spi_device_handle_t tmc_spi_device = nullptr;

// Maximum supported daisy chain length (8 drivers * 5 bytes each = 40 bytes)
static const size_t TMC_MAX_CHAIN_LENGTH = 8;
static const size_t TMC_PACKET_LEN = 5;
static const size_t TMC_MAX_BUFFER_SIZE = TMC_MAX_CHAIN_LENGTH * TMC_PACKET_LEN;

// Initialize TMC as an SPI device on the bus
// Returns true if device is ready to use
static bool tmc_spi_init_device() {
    if (tmc_spi_device != nullptr) {
        return true;  // Already initialized
    }

    spi_device_interface_config_t devcfg = {};
    devcfg.mode          = 3;          // TMC uses SPI Mode 3 (CPOL=1, CPHA=1)
    devcfg.clock_speed_hz = 2000000;   // 2 MHz - safe for all TMC variants
    devcfg.spics_io_num  = -1;         // CS handled manually by TMCStepper library
    devcfg.queue_size    = 1;          // We do synchronous transfers
    devcfg.flags         = 0;
    devcfg.command_bits  = 0;
    devcfg.address_bits  = 0;
    devcfg.dummy_bits    = 0;

    esp_err_t ret = spi_bus_add_device(HSPI_HOST, &devcfg, &tmc_spi_device);
    if (ret != ESP_OK) {
        log_error("Failed to add TMC to SPI bus: " << esp_err_to_name(ret));
        tmc_spi_device = nullptr;
        return false;
    }

    log_debug("TMC SPI device initialized");
    return true;
}

// Perform a full-duplex SPI transfer using polling (no DMA)
// This avoids the DMA issues with non-standard packet lengths
static void tmc_spi_transfer(const uint8_t* tx_buf, uint8_t* rx_buf, size_t len) {
    if (!tmc_spi_init_device()) {
        return;
    }

    spi_transaction_t trans = {};
    trans.length    = len * 8;  // Length in bits
    trans.tx_buffer = tx_buf;
    trans.rx_buffer = rx_buf;

    // Use polling transmit to avoid DMA issues with 5-byte packets
    esp_err_t ret = spi_device_polling_transmit(tmc_spi_device, &trans);
    if (ret != ESP_OK) {
        log_error("TMC SPI transfer failed: " << esp_err_to_name(ret));
    }
}

// Replace the library's weak definition of TMC2130Stepper::write()
// This is executed in the object context so it has access to class
// data such as the CS pin that switchCSpin() uses
void TMC2130Stepper::write(uint8_t reg, uint32_t data) {
    log_verbose("TMC reg " << to_hex(reg) << " write " << to_hex(data));

    if (!tmc_spi_init_device()) {
        return;
    }

    // Calculate buffer size for daisy chain
    // link_index is the position in chain (1-indexed for chained, 0 or -1 for single)
    int before = link_index > 0 ? link_index - 1 : 0;
    size_t total_bytes = (before + 1) * TMC_PACKET_LEN;

    if (total_bytes > TMC_MAX_BUFFER_SIZE) {
        log_error("TMC daisy chain too long");
        return;
    }

    uint8_t tx_buf[TMC_MAX_BUFFER_SIZE];
    uint8_t rx_buf[TMC_MAX_BUFFER_SIZE];
    memset(tx_buf, 0, total_bytes);

    // Build the write command packet
    // Data goes at the start, trailing zeros push it through the chain
    tx_buf[0] = reg | 0x80;  // Set write bit
    tx_buf[1] = (data >> 24) & 0xFF;
    tx_buf[2] = (data >> 16) & 0xFF;
    tx_buf[3] = (data >> 8) & 0xFF;
    tx_buf[4] = data & 0xFF;

    switchCSpin(0);
    tmc_spi_transfer(tx_buf, rx_buf, total_bytes);
    switchCSpin(1);
}

// Replace the library's weak definition of TMC2130Stepper::read()
uint32_t TMC2130Stepper::read(uint8_t reg) {
    if (!tmc_spi_init_device()) {
        return 0;
    }

    int before = link_index > 0 ? link_index - 1 : 0;
    size_t total_bytes = (before + 1) * TMC_PACKET_LEN;

    if (total_bytes > TMC_MAX_BUFFER_SIZE) {
        log_error("TMC daisy chain too long");
        return 0;
    }

    uint8_t tx_buf[TMC_MAX_BUFFER_SIZE];
    uint8_t rx_buf[TMC_MAX_BUFFER_SIZE];
    memset(tx_buf, 0, total_bytes);

    // First cycle: send register address to latch data into output register
    tx_buf[0] = reg;

    switchCSpin(0);
    tmc_spi_transfer(tx_buf, rx_buf, total_bytes);
    switchCSpin(1);

    // Second cycle: clock out the latched data
    // For daisy chains, we need to clock enough bits for all chips after target
    size_t afterChips = link_index > 0 ? chain_length - link_index : 0;
    size_t dummy_in_bytes = afterChips * TMC_PACKET_LEN;
    size_t read_total_bytes = (afterChips + 1) * TMC_PACKET_LEN;

    if (read_total_bytes > TMC_MAX_BUFFER_SIZE) {
        log_error("TMC daisy chain too long");
        return 0;
    }

    uint8_t tx_buf2[TMC_MAX_BUFFER_SIZE];
    uint8_t rx_buf2[TMC_MAX_BUFFER_SIZE];
    memset(tx_buf2, 0, read_total_bytes);
    memset(rx_buf2, 0, read_total_bytes);

    switchCSpin(0);
    tmc_spi_transfer(tx_buf2, rx_buf2, read_total_bytes);
    switchCSpin(1);

    // Extract data from the correct position in the receive buffer
    // The dummy bytes from trailing chips appear first, target chip data at end
    uint8_t status = rx_buf2[dummy_in_bytes];

    uint32_t data = (uint32_t)rx_buf2[dummy_in_bytes + 1] << 24;
    data |= (uint32_t)rx_buf2[dummy_in_bytes + 2] << 16;
    data |= (uint32_t)rx_buf2[dummy_in_bytes + 3] << 8;
    data |= (uint32_t)rx_buf2[dummy_in_bytes + 4];

    log_verbose("TMC reg " << to_hex(reg) << " read " << to_hex(data) << " status " << to_hex(status));

    return data;
}

#endif // USE_ESPIDF_TMC_SPI

