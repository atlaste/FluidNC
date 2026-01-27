# CMake script to copy machine-specific configuration
# Called during the build process to copy machine config to data/config.yaml

cmake_minimum_required(VERSION 3.17.0)

if(NOT DEFINED MACHINE_CONFIG)
    # No machine config specified, use default
    set(MACHINE_CONFIG "")
endif()

set(DATA_DIR "${PROJECT_DIR}/data")
set(MACHINE_DIR "${PROJECT_DIR}/machine_configs")
set(TARGET_CONFIG "${DATA_DIR}/config.yaml")

# Ensure data directory exists
file(MAKE_DIRECTORY "${DATA_DIR}")

if(MACHINE_CONFIG AND NOT MACHINE_CONFIG STREQUAL "")
    # Check if it's a relative path or just a filename
 if(EXISTS "${MACHINE_CONFIG}")
   set(SOURCE_CONFIG "${MACHINE_CONFIG}")
    elseif(EXISTS "${MACHINE_DIR}/${MACHINE_CONFIG}")
      set(SOURCE_CONFIG "${MACHINE_DIR}/${MACHINE_CONFIG}")
    elseif(EXISTS "${PROJECT_DIR}/${MACHINE_CONFIG}")
        set(SOURCE_CONFIG "${PROJECT_DIR}/${MACHINE_CONFIG}")
    else()
        message(FATAL_ERROR "Machine config file not found: ${MACHINE_CONFIG}\n"
         "Searched in:\n"
          "  - ${MACHINE_CONFIG} (absolute/relative)\n"
       "  - ${MACHINE_DIR}/${MACHINE_CONFIG}\n"
        "  - ${PROJECT_DIR}/${MACHINE_CONFIG}")
    endif()
    
    message(STATUS "Copying machine config: ${SOURCE_CONFIG} -> ${TARGET_CONFIG}")
    file(COPY "${SOURCE_CONFIG}" DESTINATION "${DATA_DIR}")
    
    # Get the filename from the source
    get_filename_component(SOURCE_FILENAME "${SOURCE_CONFIG}" NAME)
    
    # If the filename is not config.yaml, rename it
    if(NOT SOURCE_FILENAME STREQUAL "config.yaml")
        file(RENAME "${DATA_DIR}/${SOURCE_FILENAME}" "${TARGET_CONFIG}")
    endif()
    
    message(STATUS "Machine configuration '${SOURCE_CONFIG}' copied to data/config.yaml")
else()
  # Check if config.yaml already exists
    if(EXISTS "${TARGET_CONFIG}")
  message(STATUS "Using existing data/config.yaml")
    else()
    message(WARNING "No machine config specified and data/config.yaml does not exist.\n"
   "Build will continue but the device may not have a valid configuration.\n"
    "Use: idf.py -DMACHINE=<filename> build")
    endif()
endif()
