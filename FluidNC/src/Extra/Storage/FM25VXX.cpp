/*
 * Cypress-Ramtron FM25VXX library for Amtel/Arduino
 * Copyright (C) 2013 by Jerry Adair
 *
 * This Library is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This Library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with the Arduino FM25VXX Library.  If not, write to the
 * Free Software Foundation Inc., 51 Franklin St, Fifth Floor,
 * Boston, MA, USA 02110-1301 or <http://www.gnu.org/licenses/>.
 */

#include "FM25VXX.h"
#include "Logging.h"
#include <SPI.h>
#include <Arduino.h>
#include <inttypes.h>

/*
 * FM25VXX constructor
 */
FM25VXX::FM25VXX(uint8_t csPin, uint8_t wPin, uint8_t holdPin, uint32_t fence) :
    _initialized(false), _chipSelectPin(csPin), _writeProtectPin(wPin), _holdPin(holdPin), _statusRegister(0x00), _currentAddress(0x00),
    _fenceAddress(fence) {
    /*
    * According to the datasheet (pages 5, 12, & 14) we
    * need to wait at least 250us after power rises above
    * 2.0V before the first chip select (active low).  We'll
    * wait 500us.  The Arduinos have a steep power up
    * profile (turn on quickly), but let's give it plenty
    * of time.  We're talking microseconds (us) here, so
    * it's not a big deal.  Of course, it's entirely possible
    * that the chip has been powered up for some time prior
    * to class instantiation.  That said, we'll go with the
    * conservative approach and give it 500us anyway.
    */
    delayMicroseconds(FM25VXX_POWERUP_DELAY_uS);

} /* FM25VXX() */

void FM25VXX::Initialize() {
    FM25VXXManufacturer  m;
    FM25VXXFamilyDensity f;
    FM25VXXVariant       v;

    SPI.begin(12, 13, 11, -1);
    if (ReadManufacturer(&m, &f, &v) != FM25VXX_SUCCESS) {
        return;
    }

    switch (v) {
        case FM25V01:
            _maxAddress = FM25V01_MAX_ADDRESS;
            break;
        case FM25V02:
            _maxAddress = FM25V02_MAX_ADDRESS;
            break;
        case FM25V05:
            _maxAddress = FM25V05_MAX_ADDRESS;
            break;
        default:
        case FM25V10:
        case FM25VN10:
            _maxAddress = FM25V10_MAX_ADDRESS;
            break;
        case FM25V20:
            _maxAddress = FM25V20_MAX_ADDRESS;
            break;

    } /* chip variant switch */

    /*
    * If the user doesn't want a fence or if they
    * specify an invalid address for the fence then
    * just set the fence to the max address of the chip
    */
    if (_fenceAddress == 0x00 || _fenceAddress > _maxAddress) {
        _fenceAddress = _maxAddress + 1;

    } /* fix up the fence */

    /*
    * Status register value that we want to write during
    * initialization such that there are no protected
    * memory addresses and that the WP/ pin from the
    * microcontroller is not ignored. See page 6 of the
    * datasheet for more information regarding the layered
    * write protection scheme of the FM25VXX.  If the end
    * user wants something other than this default, then
    * they can make a separate call to WriteStatusRegister().
    */
    WriteStatusRegister(1, 0, 0, 0);

    _initialized = true;

} /* Initialize() */

bool FM25VXX::IsInitialized() {
    return (_initialized);

} /* IsInitialized() */

void FM25VXX::Sleep() {
    /*
    * Send the SLEEP opcode, pull the pin HIGH.
    * At the point that cs goes HIGH, that is
    * actually the point at which the device
    * begins to sleep, not before.  See the
    * datasheet (page 9) for more information.
    */
    digitalWrite(_chipSelectPin, LOW);
    SPI.transfer(FM25VXX_SLEEP);
    digitalWrite(_chipSelectPin, HIGH);

} /* Sleep() */

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

    ReadByte(FM25VXX_NULL, &data);
    _currentAddress = cAddress;
    delayMicroseconds(FM25VXX_WAKEUP_DELAY_uS);

} /* Wakeup() */

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

uint32_t FM25VXX::GetTheCurrentAddress() {
    return (_currentAddress);

} /* GetTheCurrentAddress() */

void FM25VXX::SetTheCurrentAddress(uint32_t newAddress) {
    if (newAddress > _maxAddress) {
        _currentAddress = _maxAddress;

    } else {
        _currentAddress = newAddress;

    } /* set the new address */

} /* SetTheCurrentAddress() */

uint32_t FM25VXX::GetTheMaxAddress() {
    return (_maxAddress);

} /* GetTheMaxAddress() */

FM25VXXError FM25VXX::Erase(FM25VXXErase whatToErase) {
    uint8_t  nullData        = FM25VXX_NULL;
    uint32_t numBytesToErase = FM25VXX_NULL;

    if (whatToErase == FM25VXX_TO_THE_FENCE) {
        numBytesToErase = _fenceAddress;
    } else {
        if (_fenceAddress <= _maxAddress) {
            return (FM25VXX_ERASE_PAST_FENCE_REQUEST);
        }

        numBytesToErase = _maxAddress;

    } /* if erasing to the fence or the whole chip */

    _currentAddress = FM25VXX_BASE_ADDRESS;

    WriteBlock(_currentAddress, 1, numBytesToErase, &nullData);

    return (FM25VXX_SUCCESS);

} /* Erase() */

FM25VXXError FM25VXX::WriteProtectFM25VXX(FM25VXXProtection whatToProtect) {
    switch (whatToProtect) {
        case FM25VXX_PROTECT_UPPER_QUARTER:
            WriteStatusRegister(1, 0, 1, 0);
            break;
        case FM25VXX_PROTECT_UPPER_HALF:
            WriteStatusRegister(1, 1, 0, 0);
            break;
        case FM25VXX_PROTECT_ALL:
            WriteStatusRegister(1, 1, 1, 0);
            break;
        case FM25VXX_PROTECT_NONE:
            WriteStatusRegister(1, 0, 0, 0);
            break;
        default:
            return (FM25VXX_INVALID_WRITE_PROTECT);
            break;
    }

    return (FM25VXX_SUCCESS);

} /* WriteProtectFM25VXX() */

void FM25VXX::WriteStatusRegister(uint8_t wpen, uint8_t bp0, uint8_t bp1, uint8_t wel) {
    wpen == 1 ? bitSet(_statusRegister, FM25VXX_WPEN) : bitClear(_statusRegister, FM25VXX_WPEN);

    bp0 == 1 ? bitSet(_statusRegister, FM25VXX_BP0) : bitClear(_statusRegister, FM25VXX_BP0);

    bp1 == 1 ? bitSet(_statusRegister, FM25VXX_BP1) : bitClear(_statusRegister, FM25VXX_BP1);

    wel == 1 ? bitSet(_statusRegister, FM25VXX_WEL) : bitClear(_statusRegister, FM25VXX_WEL);

    digitalWrite(_chipSelectPin, LOW);
    SPI.transfer(FM25VXX_WREN);
    digitalWrite(_chipSelectPin, HIGH);
    digitalWrite(_chipSelectPin, LOW);
    SPI.transfer(FM25VXX_WRSR);
    SPI.transfer(_statusRegister);
    digitalWrite(_chipSelectPin, HIGH);

} /* WriteStatusRegister() */

void FM25VXX::WriteStatusRegister(uint8_t sRegister) {
    _statusRegister = sRegister;

    digitalWrite(_chipSelectPin, LOW);
    SPI.transfer(FM25VXX_WREN);
    digitalWrite(_chipSelectPin, HIGH);
    digitalWrite(_chipSelectPin, LOW);
    SPI.transfer(FM25VXX_WRSR);
    SPI.transfer(_statusRegister);
    digitalWrite(_chipSelectPin, HIGH);

} /* WriteStatusRegister() */

void FM25VXX::ReadStatusRegister(uint8_t* wpen, uint8_t* bp0, uint8_t* bp1, uint8_t* wel) {
    digitalWrite(_chipSelectPin, LOW);
    SPI.transfer(FM25VXX_RDSR);
    _statusRegister = SPI.transfer(FM25VXX_NULL);
    digitalWrite(_chipSelectPin, HIGH);

    *wpen = bitRead(_statusRegister, FM25VXX_WPEN);

    *bp0 = bitRead(_statusRegister, FM25VXX_BP0);

    *bp1 = bitRead(_statusRegister, FM25VXX_BP1);

    *wel = bitRead(_statusRegister, FM25VXX_WEL);

} /* ReadStatusRegister() */

uint8_t FM25VXX::ReadStatusRegister() {
    digitalWrite(_chipSelectPin, LOW);
    SPI.transfer(FM25VXX_RDSR);
    _statusRegister = SPI.transfer(FM25VXX_NULL);
    digitalWrite(_chipSelectPin, HIGH);

    return (_statusRegister);

} /* ReadStatusRegister() */

FM25VXXError FM25VXX::WriteByte(uint32_t address, uint8_t data) {
    /*
    * Basic sanity check for writing past the fence
    */
    if (address >= _fenceAddress) {
        return (FM25VXX_WRITE_PAST_FENCE_REQUEST);
    }

    digitalWrite(_chipSelectPin, LOW);
    SPI.transfer(FM25VXX_WREN);
    digitalWrite(_chipSelectPin, HIGH);
    digitalWrite(_chipSelectPin, LOW);
    SPI.transfer(FM25VXX_WRITE);
    if (_maxAddress >= FM25V10_MAX_ADDRESS) {
        SPI.transfer(thirdByte(address));
    }
    SPI.transfer(highByte(address));
    SPI.transfer(lowByte(address));
    SPI.transfer(data);
    digitalWrite(_chipSelectPin, HIGH);
    /*
    * Update current address
    */
    if ((address + 1) == _fenceAddress) {
        _currentAddress = FM25VXX_BASE_ADDRESS;
    } else {
        _currentAddress = address + 1;
    }

    return (FM25VXX_SUCCESS);

} /* WriteByte() */

FM25VXXError FM25VXX::WriteByte(uint8_t data) {
    return (WriteByte(_currentAddress, data));

} /* WriteByte() */

FM25VXXError FM25VXX::WriteBlock(uint32_t address, uint32_t blockSize, uint32_t numBlocks, uint8_t* data) {
    uint32_t numBytesToWrite     = blockSize * numBlocks;
    uint32_t addressPlusNumBytes = address + numBytesToWrite;

    /*
    * Overwrite detection
    */
    if (numBytesToWrite > _fenceAddress) {
        return (FM25VXX_OVERWRITE_ARRAY_REQUEST);
    }

    /*
    * Basic sanity check for writing past the fence
    */
    if (address >= _fenceAddress) {
        return (FM25VXX_WRITE_PAST_FENCE_REQUEST);
    }

    /*
    * Calculate the number of bytes up to
    * but not including the _fenceAddress.
    */
    uint32_t bytesToTheFence = _fenceAddress - address;

    digitalWrite(_chipSelectPin, LOW);
    SPI.transfer(FM25VXX_WREN);
    digitalWrite(_chipSelectPin, HIGH);
    digitalWrite(_chipSelectPin, LOW);
    SPI.transfer(FM25VXX_WRITE);
    if (_maxAddress >= FM25V10_MAX_ADDRESS) {
        SPI.transfer(thirdByte(address));
    }
    SPI.transfer(highByte(address));
    SPI.transfer(lowByte(address));

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
            SPI.transfer(data[j]);
        }

        digitalWrite(_chipSelectPin, HIGH);
        digitalWrite(_chipSelectPin, LOW);
        SPI.transfer(FM25VXX_WREN);
        digitalWrite(_chipSelectPin, HIGH);
        digitalWrite(_chipSelectPin, LOW);
        SPI.transfer(FM25VXX_WRITE);
        if (_maxAddress >= FM25V10_MAX_ADDRESS) {
            SPI.transfer(FM25VXX_BASE_ADDRESS);
        }
        SPI.transfer(FM25VXX_BASE_ADDRESS);
        SPI.transfer(FM25VXX_BASE_ADDRESS);

        for (; i < numBytesToWrite; i++, j == blockSize - 1 ? j = 0 : j++) {
            SPI.transfer(data[j]);
        }

        _currentAddress = FM25VXX_BASE_ADDRESS + numBytesToWrite - bytesToTheFence;

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
            SPI.transfer(data[j]);
        }

        if (addressPlusNumBytes >= _fenceAddress) {
            _currentAddress = FM25VXX_BASE_ADDRESS + numBytesToWrite - bytesToTheFence;
        } else {
            _currentAddress = addressPlusNumBytes;
        }

    } /* deal with the fence stuff */

    digitalWrite(_chipSelectPin, HIGH);

    return (FM25VXX_SUCCESS);

} /* WriteBlock() */

FM25VXXError FM25VXX::WriteBlock(uint32_t blockSize, uint32_t numBlocks, uint8_t* data) {
    return (WriteBlock(_currentAddress, blockSize, numBlocks, data));

} /* WriteBlock() */

FM25VXXError FM25VXX::ReadByte(uint32_t address, uint8_t* data) {
    /*
    * Basic sanity check for reading past the _maxAddress
    */
    if (address > _maxAddress) {
        return (FM25VXX_READ_PAST_MAX_ADDRESS_REQUEST);
    }

    digitalWrite(_chipSelectPin, LOW);
    SPI.transfer(FM25VXX_READ);
    if (_maxAddress >= FM25V10_MAX_ADDRESS) {
        SPI.transfer(thirdByte(address));
    }
    SPI.transfer(highByte(address));
    SPI.transfer(lowByte(address));
    *data = SPI.transfer(FM25VXX_NULL);
    digitalWrite(_chipSelectPin, HIGH);

    /*
    * Update current address
    */
    if ((address + 1) == _fenceAddress) {
        _currentAddress = FM25VXX_BASE_ADDRESS;
    } else {
        _currentAddress = address + 1;
    }

    return (FM25VXX_SUCCESS);

} /* ReadByte() */

FM25VXXError FM25VXX::ReadByte(uint8_t* data) {
    return (ReadByte(_currentAddress, data));

} /* ReadByte() */

/*
 * It is assumed that the caller has allocated a memory
 * region of size >= blockSize, pointed to by data.  If
 * not, the behavior of this function is undefined.
 */
FM25VXXError FM25VXX::ReadBlock(uint32_t address, uint32_t blockSize, uint32_t numBlocks, uint8_t* data) {
    uint32_t numBytesToRead      = blockSize * numBlocks;
    uint32_t addressPlusNumBytes = address + numBytesToRead;

    /*
    * Overread detection
    */
    if (numBytesToRead > _fenceAddress) {
        return (FM25VXX_OVERREAD_ARRAY_REQUEST);
    }

    /*
    * Basic sanity check for reading past the _maxAddress
    */
    if (address > _maxAddress) {
        return (FM25VXX_READ_PAST_MAX_ADDRESS_REQUEST);
    }

    /*
    * Calculate the number of bytes up to
    * but not including the _fenceAddress.
    */
    uint32_t bytesToTheFence = _fenceAddress - address;

    digitalWrite(_chipSelectPin, LOW);
    SPI.transfer(FM25VXX_READ);
    if (_maxAddress >= FM25V10_MAX_ADDRESS) {
        SPI.transfer(thirdByte(address));
    }
    SPI.transfer(highByte(address));
    SPI.transfer(lowByte(address));

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
            data[i] = SPI.transfer(FM25VXX_NULL);
        }

        digitalWrite(_chipSelectPin, HIGH);
        digitalWrite(_chipSelectPin, LOW);
        SPI.transfer(FM25VXX_READ);
        if (_maxAddress >= FM25V10_MAX_ADDRESS) {
            SPI.transfer(FM25VXX_BASE_ADDRESS);
        }
        SPI.transfer(FM25VXX_BASE_ADDRESS);
        SPI.transfer(FM25VXX_BASE_ADDRESS);

        for (; i < numBytesToRead; i++) {
            data[i] = SPI.transfer(FM25VXX_NULL);
        }

        _currentAddress = FM25VXX_BASE_ADDRESS + numBytesToRead - bytesToTheFence;

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
            data[i] = SPI.transfer(FM25VXX_NULL);
        }

        if (addressPlusNumBytes >= _fenceAddress) {
            _currentAddress = FM25VXX_BASE_ADDRESS + numBytesToRead - bytesToTheFence;
        } else {
            _currentAddress = addressPlusNumBytes;
        }

    } /* deal with the fence stuff */

    digitalWrite(_chipSelectPin, HIGH);

    return (FM25VXX_SUCCESS);

} /* ReadBlock() */

FM25VXXError FM25VXX::ReadBlock(uint32_t blockSize, uint32_t numBlocks, uint8_t* data) {
    return (ReadBlock(_currentAddress, blockSize, numBlocks, data));

} /* ReadBlock() */

FM25VXXError FM25VXX::ReadManufacturer(FM25VXXManufacturer* manufacturer, FM25VXXFamilyDensity* storage, FM25VXXVariant* variant) {
    *manufacturer = FM25VXX_MAN_UNKNOWN;
    *storage      = FM25VXX_STORAGE_UNKNOWN;
    *variant      = FM25VXX_VARIANT_UNKNOWN;

    digitalWrite(_chipSelectPin, LOW);
    SPI.transfer(FM25VXX_RDID);

    /*
    * Read/ignore the 6 continuation codes
    */
    for (uint8_t i = 0; i < FM25VXX_CONTINUATION_CODE_NUM; i++) {
        SPI.transfer(FM25VXX_NULL);
    }

    /*
    * Read the manufacturer ID, if not
    * CYPRESS_RAMTRON_MANUFACTURER_ID then
    * there's a serious problem.  Hah!
    */
    if (SPI.transfer(FM25VXX_NULL) != CYPRESS_RAMTRON_MANUFACTURER_ID) {
        digitalWrite(_chipSelectPin, HIGH);
        return (FM25VXX_UNKNOWN_MANUFACTURER);
    }
    *manufacturer = FM25VXX_MAN_CYPRESS_RAMTRON;

    /*
    * Read the amount of storage
    */
    switch (SPI.transfer(FM25VXX_NULL)) {
        case FM25VXX_128K_STORAGE_CODE:
            *storage = FM25VXX_128K;
            *variant = FM25V01;
            break;

        case FM25VXX_256K_STORAGE_CODE:
            *storage = FM25VXX_256K;
            *variant = FM25V02;
            break;

        case FM25VXX_512K_STORAGE_CODE:
            *storage = FM25VXX_512K;
            *variant = FM25V05;
            break;

        case FM25VXX_1M_STORAGE_CODE:
            *storage = FM25VXX_1M;
            break;

        case FM25VXX_2M_STORAGE_CODE:
            *storage = FM25VXX_2M;
            *variant = FM25V20;
            break;

        default:
            digitalWrite(_chipSelectPin, HIGH);
            return (FM25VXX_UNSUPPORTED_STORAGE_DENSITY);
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
    if (*variant == FM25VXX_VARIANT_UNKNOWN) {
        switch (SPI.transfer(FM25VXX_NULL)) {
            case FM25V10_VARIANT_CODE:
                *variant = FM25V10;
                break;

            case FM25VN10_VARIANT_CODE:
                *variant = FM25VN10;
                break;

            default:
                /*
             * Things went really South and weirdly
             * so if you get here.  Shouldn't happen.
             */
                digitalWrite(_chipSelectPin, HIGH);
                return (FM25VXX_CHIP_VARIANT_ERROR);
                break;

        } /* variant switch */

    } /* if this chip has an FM25VNXX variant */

    digitalWrite(_chipSelectPin, HIGH);

    return (FM25VXX_SUCCESS);

} /* ReadManufacturer() */
