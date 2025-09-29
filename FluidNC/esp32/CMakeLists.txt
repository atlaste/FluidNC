idf_component_register(SRCS coredump.c fnc_idf_uart.c i2s_engine.c rmt_engine.c timed_engine.c tmc_spi_support.c delay_usecs.cpp esp_error.cpp  gpio.cpp 
                            GPIOCapabilities.cpp gpio_dump.cpp i2c.cpp littlefs.cpp localfs.cpp PwmPin.cpp restart.cpp sdspi.cpp spi.cpp spiffs.cpp
                            StartupLog.cpp StepTimer.cpp tmc_spi.cpp uart.cpp wdt.cpp
                       INCLUDE_DIRS "." ".." "../include" "../lib/TMCStepper/src"
                       REQUIRES esp_driver_uart driver fatfs vfs spiffs esp_driver_sdspi)
