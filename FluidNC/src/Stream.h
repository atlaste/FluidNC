#pragma once

#include "Print.h"

class Stream : public Print {
public:
    virtual int  peek()      = 0;
    virtual int  read()      = 0;
    virtual int  available() = 0;

    virtual void flush() {}

    // TODO FIXME SdB: We need to do something sensible with this. A big delay doesn't make much sense for real time motion control.

    void           setTimeout(unsigned long timeout);  // sets maximum milliseconds to wait for stream data, default is 1 second
    unsigned long  getTimeout(void);
    virtual size_t readBytes(char* buffer, size_t length);  // read chars from stream into buffer
    virtual size_t readBytes(uint8_t* buffer, size_t length) { return readBytes((char*)buffer, length); }
};
