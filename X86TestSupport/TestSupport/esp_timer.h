// Stub for Windows unit tests - ESP32 esp_timer API
#pragma once

#include <cstdint>

// Returns microseconds since boot (defined in Arduino.cpp for unit tests).
int64_t esp_timer_get_time();
