// Copyright (c) 2025 - Stefan de Bruijn
// Use of this source code is governed by a GPLv3 license that can be found in the LICENSE file.

#include "FM25VXX.h"
#include "Logging.h"
#include "NutsBolts.h"

#include <cstdint>
#include <cstring>
#include <driver/spi_master.h>

#ifdef CONFIG_IDF_TARGET_ESP32S3
#    define HSPI_HOST SPI2_HOST
#endif

/*
 * FM25VXX constructor
 */
FM25VXX::FM25VXX(Pin& csPin, Pin& wPin, Pin& holdPin, uint32_t fence, uint32_t spiFreqHz) :
    _initialized(false), _spi_device(nullptr), _writeProtectPin(wPin), _holdPin(holdPin), _statusRegister(0x00),
    _currentAddress(BASE_ADDRESS), _fenceAddress(fence), _maxAddress(0) {
    // Configure SPI device
    spi_device_interface_config_t spi_devcfg = {};
    spi_devcfg.mode                          = 0;  // SPI mode 0 for FRAM
    spi_devcfg.clock_speed_hz                = spiFreqHz;
    spi_devcfg.spics_io_num                  = csPin.getNative(Pin::Capabilities::Output);
    spi_devcfg.queue_size                    = 3;
    spi_devcfg.flags                         = 0;
    spi_devcfg.command_bits                  = 0;
    spi_devcfg.address_bits                  = 0;

    // Add device to SPI bus
    esp_err_t ret = spi_bus_add_device(HSPI_HOST, &spi_devcfg, &_spi_device);
    if (ret != ESP_OK) {
        log_error("Failed to add FRAM to SPI bus: " << esp_err_to_name(ret));
        _spi_device = nullptr;
    }
}

FM25VXX::~FM25VXX() {
    if (_spi_device) {
        spi_bus_remove_device(_spi_device);
        _spi_device = nullptr;
    }
}

// Helper method for single byte SPI transfer
uint8_t FM25VXX::spi_transfer_byte(uint8_t data) {
    if (!_spi_device)
        return 0;

    spi_transaction_t trans = {};
    trans.length            = 8;  // bits
    trans.flags             = SPI_TRANS_USE_TXDATA | SPI_TRANS_USE_RXDATA;
    trans.tx_data[0]        = data;

    spi_device_transmit(_spi_device, &trans);
    return trans.rx_data[0];
}

// Helper method for multi-byte SPI transfer
void FM25VXX::spi_transfer_bytes(const uint8_t* tx_data, uint8_t* rx_data, size_t len) {
    if (!_spi_device)
        return;

    spi_transaction_t trans = {};
    trans.length            = len * 8;  // bits
    trans.tx_buffer         = tx_data;
    trans.rx_buffer         = rx_data;

    spi_device_transmit(_spi_device, &trans);
}

// Enable writes - must be called before any write operation
void FM25VXX::write_enable() {
    if (!_spi_device)
        return;
    spi_transfer_byte(static_cast<uint8_t>(FM25VXX_Opcode::WREN));
}

void FM25VXX::Initialize() {
    FM25VXXManufacturer  m;
    FM25VXXFamilyDensity f;
    FM25VXXVariant       v;

    if (!_spi_device) {
        log_error("FRAM SPI device not initialized");
        return;
    }

    // Setup GPIO pins for WP and HOLD if defined
    if (_writeProtectPin.defined()) {
        _writeProtectPin.on();  // Disable write protect
    }
    if (_holdPin.defined()) {
        _holdPin.on();  // Disable hold
    }

    // Try to read manufacturer - some FRAM modules don't provide this
    FM25VXXError err = ReadManufacturer(&m, &f, &v);
    if (err != FM25VXXError::Success) {
        log_warn("FRAM manufacturer ID not available, assuming generic FRAM");
        // Default to FM25V10 (128Kbit)
        v = FM25VXXVariant::FM25V10;
        f = FM25VXXFamilyDensity::M1;
    } else {
        log_info("FRAM detected: manufacturer=" << static_cast<int>(m) << " density=" << static_cast<int>(f)
                                                << " variant=" << static_cast<int>(v));
    }

    // Set max address based on variant
    switch (v) {
        case FM25VXXVariant::FM25V01:
            _maxAddress = FM25V01_MAX_ADDRESS;
            break;
        case FM25VXXVariant::FM25V02:
            _maxAddress = FM25V02_MAX_ADDRESS;
            break;
        case FM25VXXVariant::FM25V05:
            _maxAddress = FM25V05_MAX_ADDRESS;
            break;
        case FM25VXXVariant::FM25V10:
        case FM25VXXVariant::FM25VN10:
            _maxAddress = FM25V10_MAX_ADDRESS;
            break;
        case FM25VXXVariant::FM25V20:
            _maxAddress = FM25V20_MAX_ADDRESS;
            break;
        case FM25VXXVariant::Unknown:
            log_error("Unknown FRAM variant");
            return;
    }

    // Test FRAM read/write functionality
    log_debug("Testing FRAM read/write at address 0x" << to_hex(TEST_ADDRESS));

    uint8_t original_value;
    if (ReadByte(TEST_ADDRESS, &original_value) != FM25VXXError::Success) {
        log_error("FRAM test read failed");
        return;
    }

    // Write test value 1
    if (WriteByte(TEST_ADDRESS, TEST_VALUE1) != FM25VXXError::Success) {
        log_error("FRAM test write 1 failed");
        return;
    }

    // Read back and verify
    uint8_t read_value;
    if (ReadByte(TEST_ADDRESS, &read_value) != FM25VXXError::Success) {
        log_error("FRAM test read after write 1 failed");
        return;
    }

    if (read_value != TEST_VALUE1) {
        log_error("FRAM test failed: wrote 0x" << to_hex(static_cast<uint32_t>(TEST_VALUE1)) << " but read 0x"
                                               << to_hex(static_cast<uint32_t>(read_value)));
        return;
    }

    // Write test value 2
    if (WriteByte(TEST_ADDRESS, TEST_VALUE2) != FM25VXXError::Success) {
        log_error("FRAM test write 2 failed");
        return;
    }

    // Read back and verify
    if (ReadByte(TEST_ADDRESS, &read_value) != FM25VXXError::Success) {
        log_error("FRAM test read after write 2 failed");
        return;
    }

    if (read_value != TEST_VALUE2) {
        log_error("FRAM test failed: wrote 0x" << to_hex(static_cast<uint32_t>(TEST_VALUE2)) << " but read 0x"
                                               << to_hex(static_cast<uint32_t>(read_value)));
        return;
    }

    // Restore original value
    WriteByte(TEST_ADDRESS, original_value);

    log_info("FRAM test passed");

    // Fix up fence if needed
    if (_fenceAddress == 0 || _fenceAddress > _maxAddress) {
        _fenceAddress = _maxAddress + 1;
    }

    // Initialize status register (no protection, WP pin enabled)
    WriteStatusRegister(1, 0, 0, 0);

    _currentAddress = BASE_ADDRESS;
    _initialized    = true;
}

bool FM25VXX::IsInitialized() {
    return (_initialized);

} /* IsInitialized() */

void FM25VXX::Sleep() {
    /*
    * Send the SLEEP opcode. CS is automatically handled by SPI driver.
    */
    if (!_spi_device)
        return;

    spi_transfer_byte(static_cast<uint8_t>(FM25VXX_Opcode::SLEEP));
}

void FM25VXX::Wakeup() {
    /*
    * After being put into SLEEP mode, the
    * device needs to be awakened.  There is
    * a period of time that this will require,
    * called the wakeup period.  The device will
    * not necessarily respond to an opcode within
    * the wakeup period.  Therefore we'll do a
    * dummy read and wait the maximum wakeup
    * period of 400us (tREC).  See the datasheet
    * (page 9) for more information.  Finally,
    * since ReadByte will set _currentAddress
    * to the address we'll specify in the dummy
    * read (0x00), we'll save _currentAddress
    * locally, then reset it prior to returning.
    */
    uint8_t  data;
    uint32_t cAddress = _currentAddress;

    ReadByte(0x00, &data);
    _currentAddress = cAddress;
    esp_rom_delay_us(WAKEUP_DELAY_US);
}

uint32_t FM25VXX::GetTheFence() {
    return (_fenceAddress);

} /* GetTheFence() */

void FM25VXX::MoveTheFence(uint32_t newFence) {
    if (newFence == 0x00 || newFence > _maxAddress) {
        _fenceAddress = _maxAddress + 1;

    } else {
        _fenceAddress = newFence;

    } /* fix up the fence */

} /* MoveTheFence() */

// uint32_t FM25VXX::GetTheCurrentAddress() {
//     return (_currentAddress);
//
// } /* GetTheCurrentAddress() */
//
// void FM25VXX::SetTheCurrentAddress(uint32_t newAddress) {
//     if (newAddress > _maxAddress) {
//         _currentAddress = _maxAddress;
//
//     } else {
//         _currentAddress = newAddress;
//
//     } /* set the new address */
//
// } /* SetTheCurrentAddress() */
//
// uint32_t FM25VXX::GetTheMaxAddress() {
//     return (_maxAddress);
//
// } /* GetTheMaxAddress() */
//
// FM25VXXError FM25VXX::Erase(FM25VXXErase whatToErase) {
//     uint8_t  nullData        = 0x00;
//     uint32_t numBytesToErase = 0x00;
//
//     if (whatToErase == FM25VXX_TO_THE_FENCE) {
//         numBytesToErase = _fenceAddress;
//     } else {
//         if (_fenceAddress <= _maxAddress) {
//             return (FM25VXX_ERASE_PAST_FENCE_REQUEST);
//         }
//
//         numBytesToErase = _maxAddress;
//
//     } /* if erasing to the fence or the whole chip */
//
//     _currentAddress = BASE_ADDRESS;
//
//     WriteBlock(_currentAddress, 1, numBytesToErase, &nullData);
//
//     return (FM25VXXError::Success);
//
// } /* Erase() */
//
// FM25VXXError FM25VXX::WriteProtectFM25VXX(FM25VXXProtection whatToProtect) {
//     switch (whatToProtect) {
//         case FM25VXX_PROTECT_UPPER_QUARTER:
//             WriteStatusRegister(1, 0, 1, 0);
//             break;
//         case FM25VXX_PROTECT_UPPER_HALF:
//             WriteStatusRegister(1, 1, 0, 0);
//             break;
//         case FM25VXX_PROTECT_ALL:
//             WriteStatusRegister(1, 1, 1, 0);
//             break;
//         case FM25VXX_PROTECT_NONE:
//             WriteStatusRegister(1, 0, 0, 0);
//             break;
//         default:
//             return (FM25VXX_INVALID_WRITE_PROTECT);
//             break;
//     }
//
//     return (FM25VXXError::Success);
//
// } /* WriteProtectFM25VXX() */

void FM25VXX::WriteStatusRegister(uint8_t wpen, uint8_t bp0, uint8_t bp1, uint8_t wel) {
    wpen == 1 ? (_statusRegister |= (1 << static_cast<uint8_t>(FM25VXX_StatusBit::WPEN))) :
                (_statusRegister &= ~(1 << static_cast<uint8_t>(FM25VXX_StatusBit::WPEN)));
    bp0 == 1 ? (_statusRegister |= (1 << static_cast<uint8_t>(FM25VXX_StatusBit::BP0))) :
               (_statusRegister &= ~(1 << static_cast<uint8_t>(FM25VXX_StatusBit::BP0)));
    bp1 == 1 ? (_statusRegister |= (1 << static_cast<uint8_t>(FM25VXX_StatusBit::BP1))) :
               (_statusRegister &= ~(1 << static_cast<uint8_t>(FM25VXX_StatusBit::BP1)));
    wel == 1 ? (_statusRegister |= (1 << static_cast<uint8_t>(FM25VXX_StatusBit::WEL))) :
               (_statusRegister &= ~(1 << static_cast<uint8_t>(FM25VXX_StatusBit::WEL)));

    uint8_t tx_buf[2] = { static_cast<uint8_t>(FM25VXX_Opcode::WREN) };
    spi_transfer_bytes(tx_buf, nullptr, 1);

    tx_buf[0] = static_cast<uint8_t>(FM25VXX_Opcode::WRSR);
    tx_buf[1] = _statusRegister;
    spi_transfer_bytes(tx_buf, nullptr, 2);
}

void FM25VXX::WriteStatusRegister(uint8_t sRegister) {
    _statusRegister = sRegister;

    spi_transfer_byte(static_cast<uint8_t>(FM25VXX_Opcode::WREN));

    spi_transfer_byte(static_cast<uint8_t>(FM25VXX_Opcode::WRSR));
    spi_transfer_byte(_statusRegister);
}

void FM25VXX::ReadStatusRegister(uint8_t* wpen, uint8_t* bp0, uint8_t* bp1, uint8_t* wel) {
    spi_transfer_byte(static_cast<uint8_t>(FM25VXX_Opcode::RDSR));
    _statusRegister = spi_transfer_byte(0x00);

    *wpen = (_statusRegister >> static_cast<uint8_t>(FM25VXX_StatusBit::WPEN)) & 1;
    *bp0  = (_statusRegister >> static_cast<uint8_t>(FM25VXX_StatusBit::BP0)) & 1;
    *bp1  = (_statusRegister >> static_cast<uint8_t>(FM25VXX_StatusBit::BP1)) & 1;
    *wel  = (_statusRegister >> static_cast<uint8_t>(FM25VXX_StatusBit::WEL)) & 1;
}

uint8_t FM25VXX::ReadStatusRegister() {
    spi_transfer_byte(static_cast<uint8_t>(FM25VXX_Opcode::RDSR));
    _statusRegister = spi_transfer_byte(0x00);

    return (_statusRegister);
}

FM25VXX::FM25VXXError FM25VXX::WriteByte(uint32_t address, uint8_t data) {
    /*
    * Basic sanity check for writing past the fence
    */
    if (address >= _fenceAddress) {
        return FM25VXXError::WritePastFence;
    }

    spi_transfer_byte(static_cast<uint8_t>(FM25VXX_Opcode::WREN));

    spi_transfer_byte(static_cast<uint8_t>(FM25VXX_Opcode::WRITE));
    if (_maxAddress >= FM25V10_MAX_ADDRESS) {
        spi_transfer_byte(thirdByte(address));
    }
    spi_transfer_byte((address >> 8) & 0xFF);  // highByte
    spi_transfer_byte(address & 0xFF);         // lowByte
    spi_transfer_byte(data);

    /*
    * Update current address
    */
    if ((address + 1) == _fenceAddress) {
        _currentAddress = BASE_ADDRESS;
    } else {
        _currentAddress = address + 1;
    }

    return FM25VXXError::Success;

} /* WriteByte() */

FM25VXX::FM25VXXError FM25VXX::WriteByte(uint8_t data) {
    return (WriteByte(_currentAddress, data));

} /* WriteByte() */

FM25VXX::FM25VXXError FM25VXX::WriteBlock(uint32_t address, uint32_t blockSize, uint32_t numBlocks, uint8_t* data) {
    uint32_t numBytesToWrite     = blockSize * numBlocks;
    uint32_t addressPlusNumBytes = address + numBytesToWrite;

    /*
    * Overwrite detection
    */
    if (numBytesToWrite + address > _fenceAddress) {
        return FM25VXXError::WritePastFence;
    }

    /*
    * Calculate the number of bytes up to
    * but not including the _fenceAddress.
    */
    uint32_t bytesToTheFence = _fenceAddress - address;

    spi_transfer_byte(static_cast<uint8_t>(FM25VXX_Opcode::WREN));

    spi_transfer_byte(static_cast<uint8_t>(FM25VXX_Opcode::WRITE));
    if (_maxAddress >= FM25V10_MAX_ADDRESS) {
        spi_transfer_byte(thirdByte(address));
    }
    spi_transfer_byte((address >> 8) & 0xFF);  // highByte
    spi_transfer_byte(address & 0xFF);         // lowByte

    /*
    * If the fence is <= _maxAddress and
    * we're gonna write past the fence then
    * separate the block write into two sections.
    * One for up to the fence and the other from
    * 0x00 forward. Update current address too.
    */
    if ((_fenceAddress <= _maxAddress) && (addressPlusNumBytes >= _fenceAddress)) {
        uint32_t i = 0;
        uint32_t j = 0;

        for (; i < bytesToTheFence; i++, j == blockSize - 1 ? j = 0 : j++) {
            spi_transfer_byte(data[j]);
        }

        spi_transfer_byte(static_cast<uint8_t>(FM25VXX_Opcode::WREN));

        spi_transfer_byte(static_cast<uint8_t>(FM25VXX_Opcode::WRITE));
        if (_maxAddress >= FM25V10_MAX_ADDRESS) {
            spi_transfer_byte(BASE_ADDRESS);
        }
        spi_transfer_byte(BASE_ADDRESS);
        spi_transfer_byte(BASE_ADDRESS);

        for (; i < numBytesToWrite; i++, j == blockSize - 1 ? j = 0 : j++) {
            spi_transfer_byte(data[j]);
        }

        _currentAddress = BASE_ADDRESS + numBytesToWrite - bytesToTheFence;

    } else {
        /*
       * The fence is greater than _maxAddress and/or
       * the block we wanna write doesn't take us
       * up to the fence, so we just need to write
       * the entire block as is.  The chip's internal
       * counter will overflow to 0x0000 automatically,
       * so let it.  Then we will simply update
       * _currentAddress after the block write.
       */

        for (uint32_t i = 0, j = 0; i < numBytesToWrite; i++, j == blockSize - 1 ? j = 0 : j++) {
            spi_transfer_byte(data[j]);
        }

        if (addressPlusNumBytes >= _fenceAddress) {
            _currentAddress = BASE_ADDRESS + numBytesToWrite - bytesToTheFence;
        } else {
            _currentAddress = addressPlusNumBytes;
        }

    } /* deal with the fence stuff */

    return (FM25VXXError::Success);

} /* WriteBlock() */

FM25VXX::FM25VXXError FM25VXX::WriteBlock(uint32_t blockSize, uint32_t numBlocks, uint8_t* data) {
    return (WriteBlock(_currentAddress, blockSize, numBlocks, data));

} /* WriteBlock() */

FM25VXX::FM25VXXError FM25VXX::ReadByte(uint32_t address, uint8_t* data) {
    /*
    * Basic sanity check for reading past the _maxAddress
    */
    if (address > _maxAddress) {
        return FM25VXXError::ReadPastMaxAddress;
    }

    spi_transfer_byte(static_cast<uint8_t>(FM25VXX_Opcode::READ));
    if (_maxAddress >= FM25V10_MAX_ADDRESS) {
        spi_transfer_byte(thirdByte(address));
    }
    spi_transfer_byte((address >> 8) & 0xFF);  // highByte
    spi_transfer_byte(address & 0xFF);         // lowByte
    *data = spi_transfer_byte(0x00);

    /*
    * Update current address
    */
    if ((address + 1) == _fenceAddress) {
        _currentAddress = BASE_ADDRESS;
    } else {
        _currentAddress = address + 1;
    }

    return (FM25VXXError::Success);

} /* ReadByte() */

FM25VXX::FM25VXXError FM25VXX::ReadByte(uint8_t* data) {
    return (ReadByte(_currentAddress, data));

} /* ReadByte() */

/*
 * It is assumed that the caller has allocated a memory
 * region of size >= blockSize, pointed to by data.  If
 * not, the behavior of this function is undefined.
 */
FM25VXX::FM25VXXError FM25VXX::ReadBlock(uint32_t address, uint32_t blockSize, uint32_t numBlocks, uint8_t* data) {
    uint32_t numBytesToRead      = blockSize * numBlocks;
    uint32_t addressPlusNumBytes = address + numBytesToRead;

    /*
    * Basic sanity check for reading past the _maxAddress
    */
    if (address + numBytesToRead > _maxAddress) {
        return FM25VXXError::ReadPastMaxAddress;
    }

    /*
    * Calculate the number of bytes up to
    * but not including the _fenceAddress.
    */
    uint32_t bytesToTheFence = _fenceAddress - address;

    spi_transfer_byte(static_cast<uint8_t>(FM25VXX_Opcode::READ));
    if (_maxAddress >= FM25V10_MAX_ADDRESS) {
        spi_transfer_byte(thirdByte(address));
    }
    spi_transfer_byte((address >> 8) & 0xFF);  // highByte
    spi_transfer_byte(address & 0xFF);         // lowByte

    /*
    * If the fence is <= _maxAddress and
    * we're gonna read past the fence then
    * separate the block read into two sections.
    * One for up to the fence and the other from
    * 0x00 forward. Update current address too.
    */
    if ((_fenceAddress <= _maxAddress) && (addressPlusNumBytes >= _fenceAddress)) {
        uint32_t i = 0;

        for (; i < bytesToTheFence; i++) {
            data[i] = spi_transfer_byte(0x00);
        }

        spi_transfer_byte(static_cast<uint8_t>(FM25VXX_Opcode::READ));
        if (_maxAddress >= FM25V10_MAX_ADDRESS) {
            spi_transfer_byte(BASE_ADDRESS);
        }
        spi_transfer_byte(BASE_ADDRESS);
        spi_transfer_byte(BASE_ADDRESS);

        for (; i < numBytesToRead; i++) {
            data[i] = spi_transfer_byte(0x00);
        }

        _currentAddress = BASE_ADDRESS + numBytesToRead - bytesToTheFence;

    } else {
        /*
       * The fence is greater than _maxAddress and/or
       * the block we wanna read doesn't take us
       * up to the fence, so we just need to write
       * the entire block as is.  The chip's internal
       * counter will overflow to 0x0000 automatically,
       * so let it.  Then we will simply update
       * _currentAddress after the block read.
       */
        for (uint32_t i = 0; i < numBytesToRead; i++) {
            data[i] = spi_transfer_byte(0x00);
        }

        if (addressPlusNumBytes >= _fenceAddress) {
            _currentAddress = BASE_ADDRESS + numBytesToRead - bytesToTheFence;
        } else {
            _currentAddress = addressPlusNumBytes;
        }

    } /* deal with the fence stuff */

    return (FM25VXXError::Success);

} /* ReadBlock() */

FM25VXX::FM25VXXError FM25VXX::ReadBlock(uint32_t blockSize, uint32_t numBlocks, uint8_t* data) {
    return (ReadBlock(_currentAddress, blockSize, numBlocks, data));

} /* ReadBlock() */

FM25VXX::FM25VXXError FM25VXX::ReadManufacturer(FM25VXXManufacturer* manufacturer, FM25VXXFamilyDensity* storage, FM25VXXVariant* variant) {
    *manufacturer = FM25VXXManufacturer::Unknown;
    *storage      = FM25VXXFamilyDensity::Unknown;
    *variant      = FM25VXXVariant::Unknown;

    spi_transfer_byte(static_cast<uint8_t>(FM25VXX_Opcode::RDID));

    /*
    * Read/ignore the 6 continuation codes
    */
    for (uint8_t i = 0; i < CONTINUATION_CODE_NUM; i++) {
        spi_transfer_byte(0x00);
    }

    /*
    * Read the manufacturer ID, if not
    * static_cast<uint8_t>(FM25VXXManufacturer::CypressRamtron) then
    * there's a serious problem.  Hah!
    */
    if (spi_transfer_byte(0x00) != static_cast<uint8_t>(FM25VXXManufacturer::CypressRamtron)) {
        return FM25VXXError::UnknownManufacturer;
    }
    *manufacturer = FM25VXXManufacturer::CypressRamtron;

    /*
    * Read the amount of storage
    */
    switch (spi_transfer_byte(0x00)) {
        case DENSITY_128K:
            *storage = FM25VXXFamilyDensity::K128;
            *variant = FM25VXXVariant::FM25V01;
            break;

        case DENSITY_256K:
            *storage = FM25VXXFamilyDensity::K256;
            *variant = FM25VXXVariant::FM25V02;
            break;

        case DENSITY_512K:
            *storage = FM25VXXFamilyDensity::K512;
            *variant = FM25VXXVariant::FM25V05;
            break;

        case DENSITY_1M:
            *storage = FM25VXXFamilyDensity::M1;
            *variant = FM25VXXVariant::FM25V10;
            break;

        case DENSITY_2M:
            *storage = FM25VXXFamilyDensity::M2;
            *variant = FM25VXXVariant::FM25V20;
            break;

        default:
            return FM25VXXError::UnsupportedDensity;
            break;

    } /* storage switch */

    /*
    * Read the chip variant.  As of 11/13,
    * only the 1M variant offered the unique
    * serial number feature, so that is the only
    * condition for which we will need to verify
    * the chip variant.  Update this to match the
    * Cypress product line if necessary.  Ordinarily
    * I would just move this up into the previous
    * switch, but leaving it here for a "cleaner"
    * functionality expansion should that
    * become necessary in the future.
    */
    if (*variant == FM25VXXVariant::Unknown) {
        switch (spi_transfer_byte(0x00)) {
            case VARIANT_FM25V10:
                *variant = FM25VXXVariant::FM25V10;
                break;

            case VARIANT_FM25VN10:
                *variant = FM25VXXVariant::FM25VN10;
                break;

            default:
                /*
             * Things went really South - couldn't determine variant
             */

                return FM25VXXError::ChipVariantError;
        }
    }

    return (FM25VXXError::Success);
}
