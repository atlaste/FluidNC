// Copyright (c) 2025 - Stefan de Bruijn
// Use of this source code is governed by a GPLv3 license that can be found in the LICENSE file.

#pragma once

#include "Pin.h"

#include <cstdint>
#include <driver/spi_master.h>

class FM25VXX {
private:
    // Constants
    static constexpr uint32_t BASE_ADDRESS          = 0x00;
    static constexpr uint8_t  CONTINUATION_CODE_NUM = 6;
    static constexpr uint32_t WAKEUP_DELAY_US       = 450;
    static constexpr uint32_t POWERUP_DELAY_US      = 500;
    static constexpr uint32_t TEST_ADDRESS          = 0x10;  // Address for init test
    static constexpr uint8_t  TEST_VALUE1           = 0xAA;
    static constexpr uint8_t  TEST_VALUE2           = 0x55;

    // Storage density codes
    static constexpr uint8_t DENSITY_128K = 0x21;
    static constexpr uint8_t DENSITY_256K = 0x22;
    static constexpr uint8_t DENSITY_512K = 0x23;
    static constexpr uint8_t DENSITY_1M   = 0x24;
    static constexpr uint8_t DENSITY_2M   = 0x25;

    // Chip variant codes
    static constexpr uint8_t VARIANT_FM25V10  = 0x00;
    static constexpr uint8_t VARIANT_FM25VN10 = 0x01;

    // Max addresses for each variant
    static constexpr uint32_t FM25V01_MAX_ADDRESS = 0x3FFF;
    static constexpr uint32_t FM25V02_MAX_ADDRESS = 0x7FFF;
    static constexpr uint32_t FM25V05_MAX_ADDRESS = 0xFFFF;
    static constexpr uint32_t FM25V10_MAX_ADDRESS = 0x1FFFF;
    static constexpr uint32_t FM25V20_MAX_ADDRESS = 0x3FFFF;

    // FRAM opcodes
    enum class FM25VXX_Opcode : uint8_t {
        WREN  = 0x06,  // Write enable
        WRDI  = 0x04,  // Write disable
        RDSR  = 0x05,  // Read status register
        WRSR  = 0x01,  // Write status register
        READ  = 0x03,  // Read memory
        WRITE = 0x02,  // Write memory
        RDID  = 0x9F,  // Read device ID
        SLEEP = 0xB9,  // Enter sleep mode
    };

    // Status register bits
    enum class FM25VXX_StatusBit : uint8_t {
        WPEN = 7,  // Write protect enable
        BP1  = 3,  // Block protect 1
        BP0  = 2,  // Block protect 0
        WEL  = 1,  // Write enable latch
    };

    // Error codes
    enum class FM25VXXError {
        Success = 0,
        WritePastFence,
        ReadPastMaxAddress,
        UnsupportedDensity,
        UnknownManufacturer,
        ChipVariantError,
        InitTestFailed,
    };

    // Supported manufacturers
    enum class FM25VXXManufacturer {
        Unknown        = 0,
        CypressRamtron = 0xC2,
    };

    // Supported storage densities
    enum class FM25VXXFamilyDensity {
        Unknown = 0,
        K128,
        K256,
        K512,
        M1,
        M2,
    };

    // Supported chip variants
    enum class FM25VXXVariant {
        Unknown = 0,
        FM25V01,
        FM25V02,
        FM25V05,
        FM25V10,
        FM25VN10,
        FM25V20,
    };

    bool                _initialized;
    spi_device_handle_t _spi_device;
    Pin&                _writeProtectPin;
    Pin&                _holdPin;

    uint8_t  _statusRegister;
    uint32_t _currentAddress;
    uint32_t _fenceAddress;
    uint32_t _maxAddress;

    // Helper methods
    uint8_t        spi_transfer_byte(uint8_t data);
    void           spi_transfer_bytes(const uint8_t* tx_data, uint8_t* rx_data, size_t len);
    void           write_enable();
    inline uint8_t thirdByte(uint32_t addr) { return (addr >> 16) & 0xFF; }

public:
    FM25VXX(Pin& csPin, Pin& wPin, Pin& holdPin, uint32_t fence, uint32_t spiFreqHz);

    void         Initialize();
    bool         IsInitialized();
    void         Sleep();
    void         Wakeup();
    uint32_t     GetTheFence();
    void         MoveTheFence(uint32_t newFence);
    FM25VXXError ProtectStatusRegister(bool protect);
    FM25VXXError WriteProtectFM25VXX(bool quarter, bool half, bool all);
    void         WriteStatusRegister(uint8_t wpen, uint8_t bp0, uint8_t bp1, uint8_t wel);
    void         WriteStatusRegister(uint8_t sRegister);
    void         ReadStatusRegister(uint8_t* wpen, uint8_t* bp0, uint8_t* bp1, uint8_t* wel);
    uint8_t      ReadStatusRegister();
    FM25VXXError WriteByte(uint32_t address, uint8_t data);
    FM25VXXError WriteByte(uint8_t data);
    FM25VXXError WriteBlock(uint32_t address, uint32_t blockSize, uint32_t numBlocks, uint8_t* data);
    FM25VXXError WriteBlock(uint32_t blockSize, uint32_t numBlocks, uint8_t* data);
    FM25VXXError ReadByte(uint32_t address, uint8_t* data);
    FM25VXXError ReadByte(uint8_t* data);
    FM25VXXError ReadBlock(uint32_t address, uint32_t blockSize, uint32_t numBlocks, uint8_t* data);
    FM25VXXError ReadBlock(uint32_t blockSize, uint32_t numBlocks, uint8_t* data);
    FM25VXXError ReadManufacturer(FM25VXXManufacturer* manufacturer, FM25VXXFamilyDensity* storage, FM25VXXVariant* variant);

    ~FM25VXX();
};
