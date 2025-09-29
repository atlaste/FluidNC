idf_component_register(SRCS ChannelPinDetail.cpp DebugPinDetail.cpp ErrorPinDetail.cpp GPIOPinDetail.cpp I2SOPinDetail.cpp PinAttributes.cpp PinCapabilities.cpp 
                            PinDetail.cpp PinOptionsParser.cpp VoidPinDetail.cpp
                       INCLUDE_DIRS "." "../.." "../../include/"
                       REQUIRES esp_driver_uart 
                       PRIV_REQUIRES Configuration src esp32)
