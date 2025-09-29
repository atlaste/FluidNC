#pragma once

#include <cstddef>
#include <cstdint>

// Print definition

class Print
{
public:
    virtual size_t write(uint8_t c) = 0;
    virtual size_t write(const uint8_t* buffer, size_t length);

    inline size_t print(char c) { return write(uint8_t(c)); }
    size_t       print(const char* str);
    size_t       print(const char* str, size_t length);
    size_t       print(int val);
    size_t       print(long long int val); 
    size_t       print(double val, int scale = 2);
};
