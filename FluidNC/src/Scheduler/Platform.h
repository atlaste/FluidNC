#pragma once

// #define CONFIG_IDF_TARGET_ESP32S3 1
// #define ESP_PLATFORM 1

#ifdef __cplusplus
#    include <cstdint>
#    include <cstring>
#else
#    include <stdint.h>
#    include <string.h>

#endif

#define HW_Win32 0x0000001     // ... bitmask
#define HW_Win64 0x0000002     // ... bitmask
#define HW_Linux 0x0000004     // ... bitmask
#define HW_ESP32 0x0000010     // ... bitmask
#define HW_ESP32_S2 0x0000020  // ... bitmask
#define HW_ESP32_S3 0x0000040  // ... bitmask
#define HW_RPI 0x0000100       // ... bitmask
#define HW_All 0xFFFFFFF       // Basically everything rest...

#if defined(ESP_PLATFORM) || defined(ESP32)

#    include <esp_attr.h>
#    include <esp_compiler.h>
#    include <esp_task_wdt.h>
#    include <sdkconfig.h>

#    define IRAM IRAM_ATTR
#    define INLINE inline __attribute__((always_inline))

#    define PACK(__Declaration__) __Declaration__ __attribute__((__packed__))

#    ifdef CONFIG_IDF_TARGET_ESP32S3

#        ifdef SHOW_SELECTED_PLATFORM
#            pragma message("Selected ESP32-S3 platform")
#        endif
#        define CURRENT_PLATFORM HW_ESP32_S3

#    elif defined(CONFIG_IDF_TARGET_ESP32S2)

#        ifdef SHOW_SELECTED_PLATFORM
#            pragma message("Selected ESP32-S2 platform")
#        endif
#        define CURRENT_PLATFORM HW_ESP32_S2

#    elif defined(CONFIG_IDF_TARGET_ESP32)

#        ifdef SHOW_SELECTED_PLATFORM
#            pragma message("Selected ESP32 platform")
#        endif
#        define CURRENT_PLATFORM HW_ESP32

#    endif

// TODO FIXME: Figure out some way to make this work properly.
#    define reset_wdt()

#elif defined(_WIN32)

#    ifdef SHOW_SELECTED_PLATFORM
#        pragma message("Selected Win32 platform")
#    endif

#    if _WIN64
#        define CURRENT_PLATFORM HW_Win64
#    else
#        define CURRENT_PLATFORM HW_Win32
#    endif

#    define PACK(__Declaration__) __pragma(pack(push, 1)) __Declaration__ __pragma(pack(pop))

#    define IRAM
#    define INLINE __forceinline

#    ifndef likely
#        define likely(x) (x)
#    endif
#    ifndef unlikely
#        define unlikely(x) (x)
#    endif

#    define reset_wdt()

#elif defined(bcm2835)

#    ifdef SHOW_SELECTED_PLATFORM
#        pragma message("Selected Raspberry PI platform")
#    endif
#    define CURRENT_PLATFORM HW_RPI

#    define IRAM
#    define INLINE __inline__ __attribute__((always_inline))

#    ifndef likely
#        define likely(x) (x)
#    endif
#    ifndef unlikely
#        define unlikely(x) (x)
#    endif

#    define PACK(__Declaration__) __Declaration__ __attribute__((__packed__))

#    ifdef SHOW_SELECTED_PLATFORM
#        pragma message("Selected Linux platform")
#    endif
#    define CURRENT_PLATFORM HW_Linux

#    define IRAM
#    define INLINE inline

#    ifndef likely
#        define likely(x) (x)
#    endif
#    ifndef unlikely
#        define unlikely(x) (x)
#    endif

#    define PACK(__Declaration__) __Declaration__ __attribute__((__packed__))

#    define reset_wdt()

#endif

#define IS_PLATFORM(platform) ((CURRENT_PLATFORM) & (platform))

void start();
void ensure_blocking_iostream();

#define INIT_STRUCTURE(type, name)                                                                                                         \
    type name;                                                                                                                             \
    memset(&name, 0, sizeof(name));

#define HAS_FLAG(value, flag) ((uint32_t(value) & uint32_t(flag)) != 0u)

#define COMBINE1(X, Y) X##Y  // helper macro
#define COMBINE(X, Y) COMBINE1(X, Y)

#ifndef ESP_PLATFORM

#    ifndef ESP_LOGI

#        define ESP_LOGI(tag, fmt, ...) printf("I %s: " fmt "\n", tag, __VA_ARGS__)
#        define ESP_LOGW(tag, fmt, ...) printf("W %s: " fmt "\n", tag, __VA_ARGS__)
#        define ESP_LOGE(tag, fmt, ...) printf("E %s: " fmt "\n", tag, __VA_ARGS__)

#    endif

#else

#    include <esp_log.h>

#    ifdef __cplusplus
extern "C"
{
    void app_main(void);
}
#    endif

#endif

// Define the standard calling convention for the platform
#if IS_PLATFORM(HW_Win32 | HW_Win64)
#    define CCONV __cdecl
#else
#    define CCONV
#endif
