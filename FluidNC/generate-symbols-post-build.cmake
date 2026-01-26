# CMake post-build script to generate symbol map
# This is called automatically after the ELF is linked
#
# Required variables (passed via -D arguments from CMakeLists.txt):
#   PROJECT_NAME - name of the project
#   PROJECT_DIR - project directory (CMAKE_SOURCE_DIR)
#   BINARY_DIR - build directory (CMAKE_BINARY_DIR)

if(NOT DEFINED PROJECT_NAME)
    message(FATAL_ERROR "PROJECT_NAME must be defined (-DPROJECT_NAME=...)")
endif()

if(NOT DEFINED PROJECT_DIR)
    message(FATAL_ERROR "PROJECT_DIR must be defined (-DPROJECT_DIR=...)")
endif()

if(NOT DEFINED BINARY_DIR)
    message(FATAL_ERROR "BINARY_DIR must be defined (-DBINARY_DIR=...)")
endif()

find_package(Python3 COMPONENTS Interpreter REQUIRED)

# Get the build directory and ELF path
set(ELF_FILE "${BINARY_DIR}/${PROJECT_NAME}.elf")
set(DATA_SYMBOLS_FILE "${PROJECT_DIR}/data/symbols.txt.gz")
set(SCRIPT_PATH "${PROJECT_DIR}/generate-symbols.py")

message(STATUS "Generating symbol map from ${ELF_FILE} to ${DATA_SYMBOLS_FILE} with ${SCRIPT_PATH}")

# Ensure data directory exists
file(MAKE_DIRECTORY "${PROJECT_DIR}/data")

message(STATUS "=============================================================")
message(STATUS "Performance Profiler - Symbol Map Generation")
message(STATUS "  Project Name: ${PROJECT_NAME}")
message(STATUS "  Project Dir:  ${PROJECT_DIR}")
message(STATUS "  Binary Dir:   ${BINARY_DIR}")
message(STATUS "  ELF File:     ${ELF_FILE}")
message(STATUS "  Output File:  ${DATA_SYMBOLS_FILE}")
message(STATUS "  Script:       ${SCRIPT_PATH}")
message(STATUS "-------------------------------------------------------------")

# Only generate if ELF exists
if(EXISTS "${ELF_FILE}")
    message(STATUS "✓ ELF file found, generating symbols...")
    
    execute_process(
        COMMAND ${Python3_EXECUTABLE} ${SCRIPT_PATH} ${ELF_FILE} -o ${DATA_SYMBOLS_FILE}
        WORKING_DIRECTORY ${PROJECT_DIR}
        RESULT_VARIABLE SYMBOLS_RESULT
        OUTPUT_VARIABLE SYMBOLS_OUTPUT
        ERROR_VARIABLE SYMBOLS_ERROR
        ECHO_OUTPUT_VARIABLE
        ECHO_ERROR_VARIABLE
    )
    
    if(SYMBOLS_RESULT EQUAL 0)
        message(STATUS "- Symbol generation completed successfully!")
        if(EXISTS "${DATA_SYMBOLS_FILE}")
            file(SIZE "${DATA_SYMBOLS_FILE}" SYMBOLS_SIZE)
            math(EXPR SYMBOLS_KB "${SYMBOLS_SIZE} / 1024")
            message(STATUS "- Symbol file created: ${DATA_SYMBOLS_FILE}")
            message(STATUS "- File size: ${SYMBOLS_SIZE} bytes (${SYMBOLS_KB} KB)")
            message(STATUS "- File will be included in LittleFS partition")
        else()
            message(WARNING "- FAILED: Symbol file was NOT created at expected location!")
            message(WARNING "   Expected: ${DATA_SYMBOLS_FILE}")
        endif()
    else()
        message(WARNING "- FAILED: Symbol generation FAILED (exit code: ${SYMBOLS_RESULT})")
        if(SYMBOLS_ERROR)
            message(WARNING "Error output:")
            message(WARNING "${SYMBOLS_ERROR}")
        endif()
        if(SYMBOLS_OUTPUT)
            message(WARNING "Standard output:")
            message(WARNING "${SYMBOLS_OUTPUT}")
        endif()
    endif()
else()
    message(WARNING "- FAILED: ELF file not found!")
    message(WARNING "   Expected location: ${ELF_FILE}")
    message(STATUS "   Skipping symbol map generation")
    message(STATUS "   (This is normal on first build before linking)")
endif()
message(STATUS "=============================================================")
