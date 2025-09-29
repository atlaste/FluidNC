#pragma once

#include <freertos/FreeRTOS.h>
#include "Print.h"

class Stream : public Print {
    unsigned long timeout_ = 1000;

public:
    virtual int peek()      = 0;
    virtual int read()      = 0;
    virtual int available() = 0;

    virtual void flush() {}

    // TODO FIXME SdB: We need to do something sensible with this. A big delay doesn't make much sense for real time motion control.

    // sets maximum milliseconds to wait for stream data, default is 1 second
    void           setTimeout(unsigned long timeout) { timeout_ = timeout; }
    unsigned long  getTimeout(void) { return timeout_; }
    virtual size_t timedReadBytes(char* buffer, size_t length, TickType_t timeout) = 0;

    virtual size_t readBytes(char* buffer, size_t length) {
        // read chars from stream into buffer

        return timedReadBytes(buffer, length, pdMS_TO_TICKS(timeout_));
    }
    virtual size_t readBytes(uint8_t* buffer, size_t length) { return readBytes((char*)buffer, length); }
};
