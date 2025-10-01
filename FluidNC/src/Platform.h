#pragma once

// This contains definitions of "very platform specific defines", that cannot be dealth with some other way.

#ifdef ESP_PLATFORM

#    define WEAK_LINK __attribute__((weak))

#else

#    define WEAK_LINK

#endif

#ifdef _MSC_VER

#    define WINDOWS_PLATFORM 1

#    define PACK(__Declaration__) __pragma(pack(push, 1)) __Declaration__ __pragma(pack(pop))

#    define IRAM
#    define INLINE __forceinline

#    ifndef likely
#        define likely(x) (x)
#    endif
#    ifndef unlikely
#        define unlikely(x) (x)
#    endif

#elif ESP_PLATFORM

#    include <esp_attr.h>
#    include <esp_compiler.h>

#    define IRAM IRAM_ATTR
#    define INLINE inline __attribute__((always_inline))

#    define PACK(__Declaration__) __Declaration__ __attribute__((__packed__))

#    if defined(CONFIG_IDF_TARGET_ESP32S3)
#        include "esp32s3/Platform.h"
#    elif defined(CONFIG_IDF_TARGET_ESP32)
#        include "esp32s2/Platform.h"
#    else
#        include "Platform.h"
#    endif

#else

#    define LINUX_PLATFORM 1

#    define IRAM
#    define INLINE inline

#    ifndef likely
#        define likely(x) (x)
#    endif
#    ifndef unlikely
#        define unlikely(x) (x)
#    endif

#    define PACK(__Declaration__) __Declaration__ __attribute__((__packed__))

#endif

#ifdef IDFBUILD
// Compatibility for older compilers versions.
#define memory_order_seq_cst seq_cst
#endif
