#include "Print.h"

#include <cmath>
#include <cstdint>

size_t Print::print(const char* str) {
    size_t n = 0;
    for (const char* c = str; *c; ++c) {
        if (!print(*c)) {
            return n;
        }
        ++n;
    }
    return n;
}

size_t Print::write(const uint8_t* str, size_t length) {
    for (size_t i = 0; i < length; ++i) {
        if (!write(str[i])) {
            return i;
        }
    }
    return length;
}

size_t Print::print(const char* str, size_t length) {
    return write(reinterpret_cast<const uint8_t*>(str), length);
}

size_t Print::print(int val) {
    return print(static_cast<long long int>(val));
}

size_t Print::print(long long int n) {
    char  buf[8 * sizeof(n) + 1];  // Assumes 8-bit chars plus zero byte.
    char* str = &buf[sizeof(buf) - 1];

    *str = '\0';

    // prevent crash if called with base == 1
    int base = 10;

    do {
        auto m = n;
        n /= base;
        char c = m - base * n;

        *--str = c < 10 ? c + '0' : c + 'A' - 10;
    } while (n);

    return print(str);
}

size_t Print::print(double number, int digits) {
    size_t n = 0;

    if (std::isnan(number)) {
        return print("nan");
    }
    if (std::isinf(number)) {
        return print("inf");
    }
    if (number > 4294967040.0) {
        return print("ovf");  // constant determined empirically
    }
    if (number < -4294967040.0) {
        return print("ovf");  // constant determined empirically
    }

    // Handle negative numbers
    if (number < 0.0) {
        n += print('-');
        number = -number;
    }

    // Round correctly so that print(1.999, 2) prints as "2.00"
    double rounding = 0.5;
    for (uint8_t i = 0; i < digits; ++i) {
        rounding /= 10.0;
    }

    number += rounding;

    // Extract the integer part of the number and print it
    unsigned long int_part  = (unsigned long)number;
    double        remainder = number - (double)int_part;
    n += print(static_cast<long long int>(int_part));

    // Print the decimal point, but only if there are digits beyond
    if (digits > 0) {
        n += print(".");
    }

    // Extract digits from the remainder one at a time
    while (digits-- > 0) {
        remainder *= 10.0;
        int toPrint = int(remainder);
        n += print(toPrint);
        remainder -= toPrint;
    }

    return n;
}
