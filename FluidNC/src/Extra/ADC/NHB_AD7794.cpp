/*
  NHB_AD7794.cpp - Library for using the AD7794 ADC
  Original created by Jaimy Juliano, December 28, 2010

  This file is part of the NHB_AD7794 library.

  MIT License

  Copyright (C) 2010,2019  Jaimy Juliano, NHBSystems

  Permission is hereby granted, free of charge, to any person obtaining a copy
  of this software and associated documentation files (the "Software"), to deal
  in the Software without restriction, including without limitation the rights
  to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
  copies of the Software, and to permit persons to whom the Software is
  furnished to do so, subject to the following conditions:

  The above copyright notice and this permission notice shall be included in all
  copies or substantial portions of the Software.

  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
  IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
  AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
  LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
  OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
  SOFTWARE.
*/

#include "NHB_AD7794.h"
#include <SPI.h>
#include "UartChannel.h"

SPIClass* hspi;

AD7794::AD7794(uint8_t csPin, uint32_t spiFrequency, double refVoltage) {
    //pinMode(csPin, OUTPUT);
    CS = csPin;

    //Older versions of the ESP8266 core had a bug that required setting
    //SPI_MODE2 here. That seems to have been resolved.

    //Should be MODE3
    spiSettings = SPISettings(spiFrequency, MSBFIRST, SPI_MODE3);

    //Default register settings
    modeReg        = AD7794_DEFAULT_MODE_REG;  //Single conversion mode, Fadc = 470Hz
    confReg        = AD7794_DEFAULT_CONF_REG;  //CH 0 - Bipolar, Gain = 1, Input buffer enabled
    isSnglConvMode = true;

    for (int i = 0; i < AD7794_CHANNEL_COUNT - 2; i++) {
        Channel[i].vRef = refVoltage;

        if (refVoltage == AD7794_INTERNAL_REF_V) {
            Channel[i].refMode = AD7794_REF_INT;
        }
    }
}

void AD7794::begin() {
    pinMode(CS, OUTPUT);
    digitalWrite(CS, HIGH);
    delay(20);

    hspi = new SPIClass(HSPI);
    // hspi->begin(HSPI_SCK, HSPI_MISO, HSPI_MOSI, SD_CS);
    hspi->begin(12, 13, 11, -1);
    // hspi->begin(12, 13, 11, -1);

    for (int i = 0; i < 4; ++i) {
        reset();
        delay(2);  //4 times the recomended period
    }

    //Apply the defaults that were set up in the constructor
    //Should add a begin(,,) method that lets you override the defaults
    for (uint8_t i = 0; i < AD7794_CHANNEL_COUNT - 2; i++) {  //<-- Channel count stuff needs to be handled better!!
        setActiveCh(i);

        writeModeReg();
        writeConfReg();
    }

    setActiveCh(0);  //Set channel back to 0

    read(0);  //Take a through away reading because the very first value read is usually junk
}

//Write 32 1's to reset the chip
void AD7794::reset() {
    // Speed set to 4MHz, SPI mode set to MODE 3 and Bit order set to MSB first.
    digitalWrite(CS, LOW);  //Assert CS
    hspi->beginTransaction(spiSettings);
    for (uint8_t i = 0; i < 4; i++) {
        hspi->transfer(0xFF);
    }
    hspi->endTransaction();
    digitalWrite(CS, HIGH);
}

//Sets bipolar/unipolar mode for currently selected channel
void AD7794::setBipolar(uint8_t ch, bool isBipolar) {
    setActiveCh(ch);
    Channel[currentCh].isBipolar = isBipolar;
    buildConfReg();
    writeConfReg();
}

void AD7794::setInputBuffer(uint8_t ch, bool isBuffered) {
    Channel[currentCh].isBuffered = isBuffered;
}

void AD7794::setGain(uint8_t ch, uint8_t gain) {
    setActiveCh(ch);
    Channel[currentCh].gain = gain;
    buildConfReg();
    writeConfReg();
}

void AD7794::setEnabled(uint8_t ch, bool enabled) {
    Channel[currentCh].isEnabled = enabled;
}

/******************************************************
Sets low byte of the mode register. The high nibble
must be 0, the low nibble sets the mode (FS3->FS0)
[EXAMPLE '\x01' = 470Hz]
refer to datasheet for modes
  FS3 | FS2 | FS1 | FS0 | frequency (hz) | Tsettle (ms)
  0   |  0  |  0  |  0  |  X             |  X
  0   |  0  |  0  |  1  |  470           |  4
  0   |  0  |  1  |  0  |  242           |  8
  0   |  0  |  1  |  1  |  123           |  16
  0   |  1  |  0  |  0  |   62           |  32
  ....
*/
void AD7794::setUpdateRate(double rate) {
    uint8_t bitMask = 0;
    //Map requested to next available range
    if (rate <= 4.17) {
        bitMask = 0x0F;
    }  //0x0F = 74 dB (50 Hz and 60 Hz)
    else if (rate > 4.17 && rate <= 6.25) {
        bitMask = 0x0E;
    }  //0x0E = 72 dB (50 Hz and 60 Hz)
    else if (rate > 6.25 && rate <= 8.33) {
        bitMask = 0x0D;
    }  //0x0D = 70 dB (50 Hz and 60 Hz)
    else if (rate > 8.33 && rate <= 10) {
        bitMask = 0x0C;
    }  //0x0C = 69 dB (50 Hz and 60 Hz)
    else if (rate > 10 && rate <= 12.5) {
        bitMask = 0x0B;
    }                                      //0x0B = 66 dB (50 Hz and 60 Hz)
    else if (rate > 12.5 && rate <= 16.7)  //This ones wierd, have to pick one
    {
        bitMask = 0x0A;
    }  //0x0A = 65 dB(50Hx and 60Hz)
       //{bitMask = 0x09;}   //0x09 = 80 dB(50Hz)65 dB(50Hx and 60Hz)
    else if (rate > 16.7 && rate <= 19.6) {
        bitMask = 0x08;
    }  //0x08 = 90 dB(60 Hz only) *Should be best option in US
    else if (rate > 19.6 && rate <= 33.2) {
        bitMask = 0x07;
    } else if (rate > 33.2 && rate <= 39) {
        bitMask = 0x06;
    } else if (rate > 39 && rate <= 50) {
        bitMask = 0x05;
    } else if (rate > 50 && rate <= 62) {
        bitMask = 0x04;
    } else if (rate > 62 && rate <= 123) {
        bitMask = 0x03;
    } else if (rate > 123 && rate <= 242) {
        bitMask = 0x02;
    } else if (rate > 242) {
        bitMask = 0x01;
    }  //anything above 242 set to 470 Hz

    modeReg &= 0xFFF0;  //Zero off low nibble
    modeReg |= uint16_t(bitMask);

    writeModeReg();
    //TODO: Put table from datasheet in comments, in header file
}

//Deprecated
// void AD7794::setConvMode(bool isSingle)
// {
//   if(isSingle == true){
//     isSnglConvMode = true;
//     modeReg &= 0x00FF;
//     modeReg |= 0x2000;
//   }
//   else{
//     isSnglConvMode = false;
//     modeReg &= 0x00FF;
//   }

//   writeModeReg();
// }

void AD7794::setMode(AD7794_OperatingModes mode) {
    //Currently only continuous and single conversion mode are supported
    modeReg &= 0x1FFF;
    modeReg |= mode << 1;

    //Temporary hack, need to change all references to isSglConvMode
    if (mode == AD7794_OpMode_SingleConv) {
        isSnglConvMode = true;
    } else {
        isSnglConvMode = false;
    }

    writeModeReg();
}

//Enable internal bias voltage for specified channel
void AD7794::setVBias(uint8_t ch, bool isEnabled) {
    setActiveCh(ch);
    Channel[currentCh].vBiasEnabled = isEnabled;
    buildConfReg();
    writeConfReg();
}

//Set the votage reference source for the specified channel
void AD7794::setRefMode(uint8_t ch, uint8_t mode) {
    setActiveCh(ch);

    //Only 0,1,2 are valid
    if (mode < 3) {
        Channel[currentCh].refMode = mode;
        buildConfReg();
        writeConfReg();
    }
}

//Added 11-14-2021
void AD7794::setChopEnabled(bool enabled) {
    // 0x0210 = chopp disabled
    if (enabled == true) {
        modeReg &= ~AD7794_CHOP_DISABLE;
    } else {
        modeReg |= AD7794_CHOP_DISABLE;
    }

    writeModeReg();
}

// OK, for now I'm not going to handle the spi transaction inside the startConversion()
// and getConvResult() functions. They are very low level and the behavior is different
// depending on conversion mode (and other things ?). think I will make these private
// and create a higher level functions to get readings.
uint32_t AD7794::getConvResult() {
    uint8_t  inByte;
    uint32_t result = 0;

    //hspi->beginTransaction(spiSettings); //We should still be in our transaction
    hspi->transfer(AD7794_READ_DATA_REG);

    //Read 24 bits one byte at a time, and put in an unsigned long
    inByte = hspi->transfer(0xFF);  //dummy byte
    result = inByte;
    inByte = hspi->transfer(0xFF);  //dummy byte
    result = result << 8;
    result = result | inByte;
    inByte = hspi->transfer(0xFF);  //dummy byte
    result = result << 8;
    result = result | inByte;

    //De-assert CS if not in continous conversion mode
    if (isSnglConvMode) {
        digitalWrite(CS, HIGH);
    }

    return result;
}

//Experiment with reading all active channels, this may be the way I go in the future UNTESTED
void AD7794::read(float* buf, uint8_t bufSize) {
    uint8_t readingCnt = 0;

    for (int i = 0; i < AD7794_CHANNEL_COUNT; i++) {
        if (Channel[i].isEnabled) {
            if (readingCnt < bufSize) {
                buf[readingCnt] = read(i);
            } else {
                //buffer too small!
                //Maybe return an error code?
            }
            readingCnt++;
        }
    }
}

/* read - The single channel version
   Gets reading from a single channel. Right now it
   is in volts (default scailing), but I may have it use
   a scaling setting stored for each channel. I might also
   just save that for a higher level hardware abstraction class.
*/
float AD7794::read(uint8_t ch) {
    //Lets the conversion result
    uint32_t adcRaw = getReadingRaw(ch);

    float result;

    if (ch == 6) {  //Channel 6 is temperature, handle it differently due to 1.17 V internal Ref
        //return (((float)adcRaw / AD7794_ADC_MAX_BP - 1) * 1.17)*100; //Bipolar, not sure what mode for temp sensor
        return TempSensorRawToDegC(adcRaw);
    }

    //And convert to Volts, note: no error checking
    if (!Channel[currentCh].isBipolar) {
        result = (adcRaw * Channel[currentCh].vRef) / (AD7794_ADC_MAX_UP * Channel[currentCh].gain);  //Unipolar formula
    } else {
        result = (((float)adcRaw / AD7794_ADC_MAX_BP - 1) * Channel[currentCh].vRef) / Channel[currentCh].gain;  //Bipolar formula
    }

    return result - Channel[currentCh].offset;
}

/* Convert AD7794X on-chip temp sensor readings to Deg C */
float AD7794::TempSensorRawToDegC(uint32_t rawData) {
    float volts = ((float)rawData - 0x800000) * (AD7794_INTERNAL_REF_V / AD7794_ADC_MAX_BP);
    float degK  = volts / 0.00081;  //Sensitivity = 0.81 mv/DegC
    float degC  = degK - 273;
    return (degC);
}

/* zero - Record offsets to apply to the channel (if active) */
void AD7794::zero(uint8_t ch) {
    if (Channel[ch].isEnabled == true) {
        //read(ch); //Take a throw away reading first -This is now done in begin,shouldn't be needed here anymore

        Channel[ch].offset = read(ch);
    }
}

/* zero - Record offsets to apply to all active
   channels. (NOT temperature an AVDD monitor)
*/
void AD7794::zero() {
    for (int i = 0; i < AD7794_CHANNEL_COUNT - 2; i++) {
        zero(i);
    }
}

/* offset - Returns channel offset value. I'm not sure if this will be needed
   in the long run, but it's usefull for testing now.
*/
float AD7794::offset(uint8_t ch) {
    return Channel[ch].offset;
}

//Added 11-14-2021
int AD7794::waitForConvReady(uint32_t timeout) {
    uint8_t  inByte;
    uint32_t t = millis();

    while ((millis() - t) <= timeout) {
        hspi->transfer(AD7794_READ_STATUS_REG);

        //Read status register
        inByte = hspi->transfer(0xFF);  //dummy byte
        if ((inByte & 0x80) == 0) {
            //bit cleared, conversion is ready
            return 0;
        }
    }
    return -1;
}

// This function is BLOCKING. I'm not sure if it is even possible to make a
// non blocking version with this chip on a shared SPI bus.
uint32_t AD7794::getReadingRaw(uint8_t ch) {
    static bool contConvStarted = false;
    // setActiveCh() also calls hspi->beginTransaction(), This causes a lockup on esp32
    // so the call has been moved down.
    //hspi->beginTransaction(spiSettings);

    if (isSnglConvMode || !contConvStarted) {  //Added 11-14-2021

        setActiveCh(ch);

        hspi->beginTransaction(spiSettings);

        startConv();
        contConvStarted = true;
    }

    if (!isSnglConvMode) {  //Added 11-14-2021
        digitalWrite(
            CS, LOW);  //It seems I need to re-assert this. Not sure why, I don't think it gets set HIGH anywhere when in continuous read mode ????
    }
    uint32_t t = millis();

    // #if defined (ESP8266) //Workaround until I figure out how to read the MISO pin status on the ESP8266
    delay(10);  //This delay is only appropriate for the fastest rate (470 Hz)
                // #else

    //Commented out to test status reg read method
    while (digitalRead(13) == HIGH) {
        if ((millis() - t) >= convTimeout) {
            break;
        }
    }

    //Test status read method NOTE: Should do something with return value (-1 if timeout)
    waitForConvReady(convTimeout);

    // #endif

    uint32_t adcRaw = getConvResult();

    if (isSnglConvMode) {  //Added 11-14-2021
        hspi->endTransaction();
    }
    return adcRaw;
}

void AD7794::setActiveCh(uint8_t ch) {
    if (ch < AD7794_CHANNEL_COUNT) {
        currentCh = ch;
        buildConfReg();
        writeConfReg();
    }
}

// OK, for now I'm not going to handle the spi transaction inside the startConversion()
// and getConvResult() functions. They are very low level and the behavior is different
// depending on conversion mode (and other things ?). think I will make these private
// and create a higher level functions to get readings.
void AD7794::startConv() {
    //Write out the mode reg, but leave CS asserted (LOW)
    //hspi->beginTransaction(spiSettings);
    digitalWrite(CS, LOW);
    hspi->transfer(AD7794_WRITE_MODE_REG);
    // hspi->transfer(highByte(modeReg));
    // hspi->transfer(lowByte(modeReg));

    hspi->transfer16(modeReg);

    //Don't end the transaction yet. for now this is going to have to hold on
    //to the buss until the conversion is complete. not sure if there is a way around this
}

//////// Private helper functions/////////////////

//This has been changed and is untested
void AD7794::buildConfReg() {
    confReg = AD7794_DEFAULT_CONF_REG;  //wipe it back to default

    confReg = (getGainBits(Channel[currentCh].gain) << 8) | currentCh;

    if (Channel[currentCh].vBiasEnabled && (currentCh < 3)) {
        uint8_t biasBits = currentCh + 1;
        confReg |= (biasBits << 14);
        confReg |= (1 << 11);  //Lets also set the boost bit
    }

    confReg |= (!Channel[currentCh].isBipolar << 12);
    confReg |= (Channel[currentCh].isBuffered << 4);

    //Set reference select bits
    confReg |= (Channel[currentCh].refMode << 6);
}

void AD7794::writeConfReg() {
    hspi->beginTransaction(spiSettings);
    digitalWrite(CS, LOW);
    hspi->transfer(AD7794_WRITE_CONF_REG);

    hspi->transfer16(confReg);

    digitalWrite(CS, HIGH);
    hspi->endTransaction();
}

void AD7794::writeModeReg() {
    hspi->beginTransaction(spiSettings);
    digitalWrite(CS, LOW);
    hspi->transfer(AD7794_WRITE_MODE_REG);

    hspi->transfer16(modeReg);

    digitalWrite(CS, HIGH);
    hspi->endTransaction();
}

byte AD7794::getGainBits(uint8_t gain) {
    uint8_t gainBits = 0;

    switch (gain) {
        case 1:
            gainBits |= 0x00;
            break;
        case 2:
            gainBits |= 0x01;
            break;
        case 4:
            gainBits |= 0x02;
            break;
        case 8:
            gainBits |= 0x03;
            break;
        case 16:
            gainBits |= 0x04;
            break;
        case 32:
            gainBits |= 0x05;
            break;
        case 64:
            gainBits |= 0x06;
            break;
        case 128:
            gainBits |= 0x07;
            break;
        default:
            //OOPS!, shouldn't be here, well just set it to 0x00 (gain = 1)
            gainBits |= 0x00;
            break;
    }
    return gainBits;
}
