cmake_minimum_required(VERSION 3.17.0)

find_package (Python COMPONENTS Interpreter)
# TODO FIXME: run git-version.py in this working folder.

set(SDKCONFIG "${CMAKE_BINARY_DIR}/sdkconfig")

message(CMAKE_SOURCE_DIR ="${CMAKE_SOURCE_DIR}") 

include($ENV{IDF_PATH}/tools/cmake/project.cmake)

list(APPEND EXTRA_COMPONENT_DIRS "${CMAKE_SOURCE_DIR}/lib/TMCStepper/src" 
								 "${CMAKE_SOURCE_DIR}/src" 
								 "${CMAKE_SOURCE_DIR}/esp32"
								 "${CMAKE_SOURCE_DIR}/src/Configuration" 
								 "${CMAKE_SOURCE_DIR}/src/Kinematics" 
								 "${CMAKE_SOURCE_DIR}/src/Machine" 
								 "${CMAKE_SOURCE_DIR}/src/Motors" 
								 "${CMAKE_SOURCE_DIR}/src/Pins"
								 "${CMAKE_SOURCE_DIR}/src/Spindles" 
								 "${CMAKE_SOURCE_DIR}/src/ToolChangers"
								 "${CMAKE_SOURCE_DIR}/src/Network" )

# Other config:
idf_build_set_property(CXX_COMPILE_OPTIONS "-Wno-deprecated-enum-enum-conversion" APPEND)
idf_build_set_property(CXX_COMPILE_OPTIONS "-fexceptions" APPEND)
idf_build_set_property(C_COMPILE_OPTIONS "-Wno-format" APPEND)

# idf_build_set_property(COMPILE_OPTIONS "${compile_options}" APPEND)
# idf_build_set_property(COMPILE_DEFINITIONS "${compile_definitions}" APPEND)
# idf_build_set_property(LINK_OPTIONS "${link_options}" APPEND)

project(FluidNC)
