// Mock implementation of esp_err functions for Windows unit testing

#include <esp_err.h>
#include <cstdio>

const char* esp_err_to_name(esp_err_t code) {
    switch (code) {
        case ESP_OK: return "ESP_OK";
        case ESP_FAIL: return "ESP_FAIL";
        case ESP_ERR_NO_MEM: return "ESP_ERR_NO_MEM";
        case ESP_ERR_INVALID_ARG: return "ESP_ERR_INVALID_ARG";
        case ESP_ERR_INVALID_STATE: return "ESP_ERR_INVALID_STATE";
        case ESP_ERR_INVALID_SIZE: return "ESP_ERR_INVALID_SIZE";
        case ESP_ERR_NOT_FOUND: return "ESP_ERR_NOT_FOUND";
        case ESP_ERR_NOT_SUPPORTED: return "ESP_ERR_NOT_SUPPORTED";
        case ESP_ERR_TIMEOUT: return "ESP_ERR_TIMEOUT";
        case ESP_ERR_INVALID_RESPONSE: return "ESP_ERR_INVALID_RESPONSE";
        case ESP_ERR_INVALID_CRC: return "ESP_ERR_INVALID_CRC";
        case ESP_ERR_INVALID_VERSION: return "ESP_ERR_INVALID_VERSION";
        case ESP_ERR_INVALID_MAC: return "ESP_ERR_INVALID_MAC";
        case ESP_ERR_NOT_FINISHED: return "ESP_ERR_NOT_FINISHED";
        default: return "UNKNOWN_ERROR";
    }
}

const char* esp_err_to_name_r(esp_err_t code, char* buf, size_t buflen) {
    snprintf(buf, buflen, "%s", esp_err_to_name(code));
    return buf;
}

void _esp_error_check_failed(esp_err_t rc, const char* file, int line, const char* function, const char* expression) {
    printf("ESP_ERROR_CHECK failed: %s at %s:%d (%s) - %s\n", 
           esp_err_to_name(rc), file, line, function, expression);
}

void _esp_error_check_failed_without_abort(esp_err_t rc, const char* file, int line, const char* function, const char* expression) {
    printf("ESP_ERROR_CHECK_WITHOUT_ABORT: %s at %s:%d (%s) - %s\n", 
           esp_err_to_name(rc), file, line, function, expression);
}
