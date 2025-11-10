if(NOT CMAKE_BUILD_EARLY_EXPANSION)
  add_definitions(-DHAVE_CONFIG_H)
  add_definitions(-DNEED_READV -DNEED_WRITEV -DNEED_GETLOGIN_R -DNEED_RANDOM -DNEED_SRANDOM)
endif()    

add_subdirectory(lib)

idf_component_register(
  SRCS ${COMPONENT_SRCS}
  INCLUDE_DIRS "include" "include/esp" "include/smb2""lib"
  REQUIRES lwip
)

# Apply ESP-specific compile definitions to the component
target_compile_definitions(${COMPONENT_LIB} PRIVATE
  HAVE_CONFIG_H
  NEED_READV
  NEED_WRITEV
  NEED_GETLOGIN_R
  NEED_RANDOM
  NEED_SRANDOM
)
