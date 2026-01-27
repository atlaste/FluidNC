cmake_minimum_required(VERSION 3.17.0)

find_package (Python COMPONENTS Interpreter)
# TODO FIXME: run git-version.py in this working folder.

set(SDKCONFIG "${CMAKE_BINARY_DIR}/sdkconfig")

message(CMAKE_SOURCE_DIR ="${CMAKE_SOURCE_DIR}") 

# Configure component cache to use a persistent location to avoid re-downloading components
# This prevents hitting rate limits and speeds up builds on Windows
if(NOT DEFINED ENV{IDF_COMPONENT_CACHE_PATH})
    # Use a persistent cache directory in the user's home directory
    if(WIN32)
        set(COMPONENT_CACHE_DIR "$ENV{USERPROFILE}\\.espressif\\component_cache")
        # Also create component storage directory for local_storage_url
        set(COMPONENT_STORAGE_DIR "$ENV{USERPROFILE}\\.espressif\\component_storage")
        # Convert to URI format for local_storage_url (forward slashes)
        string(REPLACE "\\" "/" COMPONENT_STORAGE_URI "${COMPONENT_STORAGE_DIR}")
        set(COMPONENT_STORAGE_URI "file:///${COMPONENT_STORAGE_URI}")
    else()
        set(COMPONENT_CACHE_DIR "$ENV{HOME}/.espressif/component_cache")
        set(COMPONENT_STORAGE_DIR "$ENV{HOME}/.espressif/component_storage")
        set(COMPONENT_STORAGE_URI "file://${COMPONENT_STORAGE_DIR}")
    endif()
    set(ENV{IDF_COMPONENT_CACHE_PATH} "${COMPONENT_CACHE_DIR}")
    # Ensure directories exist
    file(MAKE_DIRECTORY "${COMPONENT_CACHE_DIR}")
    file(MAKE_DIRECTORY "${COMPONENT_STORAGE_DIR}")
    
    # Create/update component manager config file with local storage URL
    # This makes the component manager check local storage before making HTTP requests
    set(COMPONENT_MANAGER_CONFIG "${CMAKE_SOURCE_DIR}/idf_component_manager.yml")
    file(WRITE "${COMPONENT_MANAGER_CONFIG}" 
        "# IDF Component Manager Configuration\n"
        "# Auto-generated to minimize HTTP requests and prevent rate limiting\n"
        "# Local storage is checked first, registry is only used as fallback\n"
        "\n"
        "profiles:\n"
        "  default:\n"
        "    local_storage_url:\n"
        "      - ${COMPONENT_STORAGE_URI}\n"
        "    registry_url: https://components.espressif.com\n"
    )
    
    message(STATUS "Setting IDF_COMPONENT_CACHE_PATH to: ${COMPONENT_CACHE_DIR}")
    message(STATUS "Component storage directory: ${COMPONENT_STORAGE_DIR}")
    message(STATUS "Component manager config: ${COMPONENT_MANAGER_CONFIG}")
endif()

# Enable HTTP request caching for component registry (default is True, but explicitly set it)
if(NOT DEFINED ENV{IDF_COMPONENT_CACHE_HTTP_REQUESTS})
    set(ENV{IDF_COMPONENT_CACHE_HTTP_REQUESTS} "True")
endif()

# Disable automatic version checking to reduce network calls
# This prevents the component manager from checking for updates when components already exist
if(NOT DEFINED ENV{IDF_COMPONENT_CHECK_NEW_VERSION})
    set(ENV{IDF_COMPONENT_CHECK_NEW_VERSION} "0")
    message(STATUS "Disabling component version checking to reduce registry requests")
endif()

# Handle machine configuration selection
# Usage: idf.py -DMACHINE=lathe.yaml build
# This will copy machine/lathe.yaml to data/config.yaml before building
if(DEFINED MACHINE)
    message(STATUS "Machine configuration selected: ${MACHINE}")
    set(MACHINE_CONFIG "${MACHINE}.yaml")
else()
    set(MACHINE_CONFIG "default.yaml")
endif()

include($ENV{IDF_PATH}/tools/cmake/project.cmake)

list(APPEND EXTRA_COMPONENT_DIRS "${CMAKE_SOURCE_DIR}/lib" "${CMAKE_SOURCE_DIR}/src" "${CMAKE_SOURCE_DIR}/esp32")

# Other config:
idf_build_set_property(CXX_COMPILE_OPTIONS "-Wno-deprecated-enum-enum-conversion" APPEND)
idf_build_set_property(CXX_COMPILE_OPTIONS "-fexceptions" APPEND)
idf_build_set_property(CXX_COMPILE_OPTIONS "-DIDFBUILD=1" APPEND)
idf_build_set_property(C_COMPILE_OPTIONS "-Wno-format" APPEND)
idf_build_set_property(C_COMPILE_OPTIONS "-DIDFBUILD=1" APPEND)

# idf_build_set_property(COMPILE_OPTIONS "${compile_options}" APPEND)
# idf_build_set_property(COMPILE_DEFINITIONS "${compile_definitions}" APPEND)
# idf_build_set_property(LINK_OPTIONS "${link_options}" APPEND)

project(FluidNC)

# Add custom include directory to Arduino component with highest priority
idf_component_get_property(arduino_lib espressif__arduino-esp32 COMPONENT_LIB)
target_include_directories(${arduino_lib} BEFORE PRIVATE 
    "${CMAKE_SOURCE_DIR}/include/ArduinoIDFHack"
)

# Generate symbol map after ELF is built (before filesystem image)
# Create a custom target that depends on the ELF being built
add_custom_target(generate_symbols
    COMMAND ${CMAKE_COMMAND} 
        -DPROJECT_NAME=${CMAKE_PROJECT_NAME}
        -DPROJECT_DIR=${CMAKE_SOURCE_DIR}
        -DBINARY_DIR=${CMAKE_BINARY_DIR}
        -P ${CMAKE_SOURCE_DIR}/generate-symbols-post-build.cmake
    COMMENT "Generating symbol map for profiling"
    DEPENDS ${CMAKE_PROJECT_NAME}.elf
    VERBATIM
)

# Copy machine configuration before creating filesystem image
# This allows selecting different machine configs via -DMACHINE=<filename>
add_custom_target(copy_machine_config
    COMMAND ${CMAKE_COMMAND}
        -DPROJECT_NAME=${CMAKE_PROJECT_NAME}
        -DPROJECT_DIR=${CMAKE_SOURCE_DIR}
        -DBINARY_DIR=${CMAKE_BINARY_DIR}
        -DMACHINE_CONFIG=${MACHINE_CONFIG}
    -P ${CMAKE_SOURCE_DIR}/copy-machine-config.cmake
    COMMENT "Copying machine configuration"
    VERBATIM
)

# Create LittleFS partition image (includes symbols.txt.gz from data/)
# This must happen AFTER symbol generation AND machine config copy
littlefs_create_partition_image(spiffs data FLASH_IN_PROJECT)

# Make the LittleFS image depend on both symbol generation and machine config
add_dependencies(littlefs_spiffs_bin generate_symbols copy_machine_config)
