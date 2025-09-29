idf_component_register(SRC_DIRS "."
                       SRCS Axes.cpp Axis.cpp EventPin.cpp Homing.cpp I2CBus.cpp I2SOBus.cpp LimitPin.cpp MachineConfig.cpp Macros.cpp Motor.cpp 
                            ProbeEventPin.cpp SPIBus.cpp UserInputs.cpp UserOutputs.cpp
                       INCLUDE_DIRS "." "../.." "../../include")
