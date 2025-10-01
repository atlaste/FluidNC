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

#ifndef FM25VXX_H
#define FM25VXX_H

#include <inttypes.h>

/*
 * FM25VXX opcode definitions
 */
#define FM25VXX_WREN      0b00000110
#define FM25VXX_WRDI      0b00000100
#define FM25VXX_RDSR      0b00000101
#define FM25VXX_WRSR      0b00000001
#define FM25VXX_READ      0b00000011
#define FM25VXX_FSTRD     0b00001011
#define FM25VXX_WRITE     0b00000010
#define FM25VXX_SLEEP     0b10111001
#define FM25VXX_RDID      0b10011111
#define FM25VXX_SNR       0b11000011

/*
 * FM25VXX status register bit definitions
 */
#define FM25VXX_WPEN      7
#define FM25VXX_BP1       3
#define FM25VXX_BP0       2
#define FM25VXX_WEL       1

/*
 * FM25VXX miscellaneous definitions
 */
#define FM25VXX_NULL   0x00

/*
 * Power up period of 250us (tPU) * 2.  See
 * the datasheet (page 5) for more information.
 */
#define FM25VXX_POWERUP_DELAY_uS   500

/*
 * Wakeup period of 400us (tREC).  See
 * the datasheet (page 9) for more information.
 */
#define FM25VXX_WAKEUP_DELAY_uS    400

/*
 * Base and maximum address values for
 * the various Cypress FRAM chips that
 * are supported in this library.
 */
#define FM25VXX_BASE_ADDRESS   0x0000
#define FM25V01_MAX_ADDRESS    0x3FFF
#define FM25V02_MAX_ADDRESS    0x7FFF
#define FM25V05_MAX_ADDRESS    0xFFFF
#define FM25V10_MAX_ADDRESS    0x1FFFF
#define FM25V20_MAX_ADDRESS    0x3FFFF

/*
 * Macro to get the third byte for FM25V10,
 * FM25VN10 and FM25V20 memory array addressing.
 */
#define thirdByte( w ) ( ( uint8_t ) ( ( w ) >> 16 ) )

/*
 * The names of the various Cypress FRAM
 * chips that are supported in this library.
 */
typedef enum { FM25VXX_VARIANT_UNKNOWN,
               FM25V01,
               FM25V02,
               FM25V05,
               FM25V10,
               FM25VN10,
               FM25V20
             } FM25VXXVariant;

/*
 * Manufacturer ID stuff.  See the datasheet
 * (page 9) for more information.
 */
#define CYPRESS_RAMTRON_MANUFACTURER_ID   0xC2
#define FM25VXX_128K_STORAGE_CODE         0x21
#define FM25VXX_256K_STORAGE_CODE         0x22
#define FM25VXX_512K_STORAGE_CODE         0x23
#define FM25VXX_1M_STORAGE_CODE           0x24
#define FM25VXX_2M_STORAGE_CODE           0x25
#define FM25V10_VARIANT_CODE              0x00
#define FM25VN10_VARIANT_CODE             0x01
#define FM25VXX_CONTINUATION_CODE_NUM     0x06
typedef enum { FM25VXX_MAN_UNKNOWN,
               FM25VXX_MAN_CYPRESS_RAMTRON
             } FM25VXXManufacturer;

typedef enum { FM25VXX_STORAGE_UNKNOWN,
               FM25VXX_128K,
               FM25VXX_256K,
               FM25VXX_512K,
               FM25VXX_1M,
               FM25VXX_2M
             } FM25VXXFamilyDensity;

/*
 * Protected region definitions
 */
typedef enum { FM25VXX_PROTECT_NONE,
               FM25VXX_PROTECT_UPPER_QUARTER,
               FM25VXX_PROTECT_UPPER_HALF,
               FM25VXX_PROTECT_ALL
             } FM25VXXProtection;

/*
 * Used when erasing the chip
 */
typedef enum { FM25VXX_TO_THE_FENCE,
               FM25VXX_ENTIRE_ARRAY
             } FM25VXXErase;

/*
 * FM25VXX error code definitions
 */
typedef enum { FM25VXX_SUCCESS,
               FM25VXX_INVALID_WRITE_PROTECT,
               FM25VXX_ERASE_PAST_FENCE_REQUEST,
               FM25VXX_WRITE_PAST_FENCE_REQUEST,
               FM25VXX_READ_PAST_MAX_ADDRESS_REQUEST,
               FM25VXX_UNKNOWN_MANUFACTURER,
               FM25VXX_UNSUPPORTED_STORAGE_DENSITY,
               FM25VXX_CHIP_VARIANT_ERROR,
               FM25VXX_OVERWRITE_ARRAY_REQUEST,
               FM25VXX_OVERREAD_ARRAY_REQUEST
             } FM25VXXError;

/*
 * Class definition
 */
class FM25VXX
{
   public:

      FM25VXX( uint8_t          csPin,
               uint8_t          wPin,
               uint8_t          holdPin,
               uint32_t         fence    );

      void          Initialize();
      bool          IsInitialized();
      void          Sleep();
      void          Wakeup();
      uint32_t      GetTheFence();
      void          MoveTheFence( uint32_t newFence );
      uint32_t      GetTheCurrentAddress();
      void          SetTheCurrentAddress( uint32_t newAddress );
      uint32_t      GetTheMaxAddress();
      FM25VXXError  Erase( FM25VXXErase whatToErase );
      FM25VXXError  WriteProtectFM25VXX( FM25VXXProtection whatToProtect );
      void          WriteStatusRegister( uint8_t   wpen,
                                         uint8_t   bp0,
                                         uint8_t   bp1,
                                         uint8_t   wel   );
      void          WriteStatusRegister( uint8_t sRegister );
      void          ReadStatusRegister( uint8_t  *wpen,
                                        uint8_t  *bp0,
                                        uint8_t  *bp1,
                                        uint8_t  *wel   );
      uint8_t       ReadStatusRegister();
      FM25VXXError  WriteByte(  uint32_t address,
                                uint8_t  data     );
      FM25VXXError  WriteByte(  uint8_t  data );
      FM25VXXError  WriteBlock( uint32_t  address,
                                uint32_t  blockSize,
                                uint32_t  numBlocks,
                                uint8_t  *data       );
      FM25VXXError  WriteBlock( uint32_t  blockSize,
                                uint32_t  numBlocks,
                                uint8_t  *data       );
      FM25VXXError  ReadByte(   uint32_t  address,
                                uint8_t  *data     );
      FM25VXXError  ReadByte(   uint8_t  *data );
      FM25VXXError  ReadBlock(  uint32_t  address,
                                uint32_t  blockSize,
                                uint32_t  numBlocks,
                                uint8_t  *data       );
      FM25VXXError  ReadBlock(  uint32_t  blockSize,
                                uint32_t  numBlocks,
                                uint8_t  *data       );
      FM25VXXError  ReadManufacturer( FM25VXXManufacturer   *manufacturer,
                                      FM25VXXFamilyDensity  *storage,
                                      FM25VXXVariant        *variant       );

   private:

      bool          _initialized;
      uint8_t       _chipSelectPin;
      uint8_t       _writeProtectPin;
      uint8_t       _holdPin;
      uint8_t       _statusRegister;
      uint32_t      _currentAddress;
      uint32_t      _fenceAddress;
      uint32_t      _maxAddress;

};

#endif /* FM25VXX_H */