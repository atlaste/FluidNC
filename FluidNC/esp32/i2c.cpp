// Copyright 2022 - Mitch Bradley
// Use of this source code is governed by a GPLv3 license that can be found in the LICENSE file.

#include "Platform.h"
#if !USE_ARDUINO_I2C_DRIVER

#    include <driver/i2c.h>

#    include "Driver/fluidnc_i2c.h"
#    include "Logging.h"

// cppcheck-suppress unusedFunction
bool i2c_master_init(objnum_t bus_number, pinnum_t sda_pin, pinnum_t scl_pin, uint32_t frequency) {
    i2c_config_t conf     = {};
    conf.mode             = I2C_MODE_MASTER;
    conf.sda_io_num       = (gpio_num_t)sda_pin;
    conf.scl_io_num       = (gpio_num_t)scl_pin;
    conf.sda_pullup_en    = GPIO_PULLUP_ENABLE;
    conf.scl_pullup_en    = GPIO_PULLUP_ENABLE;
    conf.master.clk_speed = frequency;

    esp_err_t ret = i2c_param_config((i2c_port_t)bus_number, &conf);
    if (ret != ESP_OK) {
        log_error("i2c_param_config failed");
        return true;
    }
    ret = i2c_driver_install((i2c_port_t)bus_number, conf.mode, 0, 0, 0);
    if (ret != ESP_OK) {
        return true;
    }

    // Source: esp32-hal-i2c.c

    // Clock Stretching Timeout: 20b:esp32, 5b:esp32-c3, 24b:esp32-s2
    //
    // #ifdef CONFIG_IDF_TARGET_ESP32S3
    //     i2c_set_timeout((i2c_port_t)bus_number, 0x0000001FU);
    // #else
    //     i2c_set_timeout((i2c_port_t)bus_number, 0xfffff);
    // #endif
    return false;
}

// Turn an esp_err_t into the convention the callers expect: the byte count on success, otherwise a
// negative value carrying the cause.  Collapsing every failure to -1, as this used to, made a device
// that is absent indistinguishable from one that is holding the bus, which is most of the information
// you want when an I2C device does not answer.  esp_err_t codes are positive apart from ESP_FAIL,
// which is already -1, so negating keeps them clear of any count.  arduino_i2c_driver.cpp reports
// errors the same way.
static int i2c_result(esp_err_t err, size_t count) {
    if (err == ESP_OK) {
        return int(count);
    }
    return err > 0 ? -err : -1;
}

// cppcheck-suppress unusedFunction
int i2c_write(objnum_t bus_number, uint8_t address, const uint8_t* data, size_t count) {
#    if 0
        esp_err_t        ret = ESP_FAIL;
        i2c_cmd_handle_t cmd = NULL;

        //short implementation does not support zero size writes (example when scanning) PR in IDF?
        //ret =  i2c_master_write_to_device((i2c_port_t)bus_number, address, buff, size, timeOutMillis / portTICK_PERIOD_MS);

        uint8_t cmd_buff[I2C_LINK_RECOMMENDED_SIZE(1)] = { 0 };

        cmd = i2c_cmd_link_create_static(cmd_buff, I2C_LINK_RECOMMENDED_SIZE(1));
        ret = i2c_master_start(cmd);
        if (ret != ESP_OK) {
            goto end;
        }
        ret = i2c_master_write_byte(cmd, (address << 1) | I2C_MASTER_WRITE, true);
        if (ret != ESP_OK) {
            goto end;
        }
        if (count) {
            ret = i2c_master_write(cmd, data, count, true);
            if (ret != ESP_OK) {
                goto end;
            }
        }
        ret = i2c_master_stop(cmd);
        if (ret != ESP_OK) {
            goto end;
        }
        ret = i2c_master_cmd_begin((i2c_port_t)bus_number, cmd, 10 / portTICK_PERIOD_MS);

    end:
        if (cmd != NULL) {
            i2c_cmd_link_delete_static(cmd);
        }
        return ret ? -1 : count;
#    else
    if (count == 0) {
        // A zero length write is how you ask "is anything at this address?", but
        // i2c_master_write_to_device rejects it outright because i2c_master_write insists on at least
        // one byte.  Build the address-only transfer by hand instead.
        uint8_t          buf[I2C_LINK_RECOMMENDED_SIZE(1)] = { 0 };
        i2c_cmd_handle_t cmd                              = i2c_cmd_link_create_static(buf, sizeof(buf));
        if (cmd == NULL) {
            return -1;
        }
        esp_err_t err = i2c_master_start(cmd);
        if (err == ESP_OK) {
            err = i2c_master_write_byte(cmd, (address << 1) | I2C_MASTER_WRITE, true);
        }
        if (err == ESP_OK) {
            err = i2c_master_stop(cmd);
        }
        if (err == ESP_OK) {
            err = i2c_master_cmd_begin((i2c_port_t)bus_number, cmd, pdMS_TO_TICKS(10));
        }
        i2c_cmd_link_delete_static(cmd);
        return i2c_result(err, 0);
    }

    return i2c_result(i2c_master_write_to_device((i2c_port_t)bus_number, address, data, count, pdMS_TO_TICKS(10)), count);
#    endif
}

// cppcheck-suppress unusedFunction
int i2c_read(objnum_t bus_number, uint8_t address, uint8_t* data, size_t count) {
    return i2c_result(i2c_master_read_from_device((i2c_port_t)bus_number, address, data, count, pdMS_TO_TICKS(10)), count);
}
#endif
