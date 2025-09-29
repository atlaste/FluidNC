idf_component_register(SRCS Dynamixel2.cpp MotorDriver.cpp NullMotor.cpp RcServo.cpp Servo.cpp Solenoid.cpp StandardStepper.cpp StepStick.cpp TMC2130Driver.cpp 
                            TMC2160Driver.cpp TMC2160ProDriver.cpp TMC2208Driver.cpp TMC2209Driver.cpp TMC5160Driver.cpp TMC5160ProDriver.cpp TrinamicBase.cpp 
                            TrinamicSpiDriver.cpp TrinamicUartDriver.cpp
                       INCLUDE_DIRS "." "../.." "../../include" "../../lib/TMCStepper/src"
                       REQUIRES esp_driver_uart 
                       PRIV_REQUIRES Configuration Pins)
