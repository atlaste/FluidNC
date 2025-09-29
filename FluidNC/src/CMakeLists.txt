idf_component_register(SRCS BTConfig.cpp Channel.cpp Control.cpp ControlPin.cpp CoolantControl.cpp CustomCode.cpp Error.cpp Expression.cpp FileCommands.cpp 
                            FileStream.cpp Flowcontrol.cpp FluidError.cpp FluidPath.cpp GCode.cpp HashFS.cpp InputFile.cpp Job.cpp Jog.cpp JSONEncoder.cpp 
                            AxisLimits.cpp lineedit.cpp Logging.cpp Main.cpp MotionControl.cpp NutsBolts.cpp OLED.cpp Parameters.cpp Parking.cpp Pin.cpp
                            PinMapper.cpp Planner.cpp Probe.cpp ProcessSettings.cpp Protocol.cpp RealtimeCmd.cpp Regex.cpp Report.cpp SDCard.cpp Serial.cpp 
                            Settings.cpp SettingsDefinitions.cpp Status_outputs.cpp Stepper.cpp Stepping.cpp string_util.cpp System.cpp Uart.cpp UartChannel.cpp 
                            UTF8.cpp xmodem.cpp 
                            Print.cpp Stream.cpp
                            StackTrace/AssertionFailed.cpp
                            version.cpp
                       INCLUDE_DIRS "." ".." "../include/"
                       REQUIRES driver fatfs vfs 
                       PRIV_REQUIRES nvs_flash esp_driver_gpio mbedtls Configuration esp32)
