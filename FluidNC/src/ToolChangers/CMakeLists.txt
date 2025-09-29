idf_component_register(SRCS atc.cpp atc_manual.cpp
                       INCLUDE_DIRS "." "../.." "../../include"
                       REQUIRES esp_driver_uart)
