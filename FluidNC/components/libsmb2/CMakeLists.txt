# ESP-IDF v5.x component for libsmb2
# Simplified version that only supports ESP-IDF builds

if(NOT CMAKE_BUILD_EARLY_EXPANSION)
  add_definitions(-DHAVE_CONFIG_H)
  add_definitions(-DNEED_READV -DNEED_WRITEV -DNEED_GETLOGIN_R -DNEED_RANDOM -DNEED_SRANDOM)
endif()    

# Define source files (from lib/CMakeLists.txt ESP_PLATFORM section)
set(COMPONENT_SRCS
  lib/aes.c
  lib/aes_reference.c
  lib/aes128ccm.c
  lib/alloc.c
  lib/asn1-ber.c
  lib/compat.c
  lib/dcerpc.c
  lib/dcerpc-lsa.c
  lib/dcerpc-srvsvc.c
  lib/errors.c
  lib/init.c
  lib/hmac.c
  lib/hmac-md5.c
  lib/libsmb2.c
  lib/md4c.c
  lib/md5.c
  lib/ntlmssp.c
  lib/pdu.c
  lib/sha1.c
  lib/sha224-256.c
  lib/sha384-512.c
  lib/smb2-cmd-close.c
  lib/smb2-cmd-create.c
  lib/smb2-cmd-echo.c
  lib/smb2-cmd-error.c
  lib/smb2-cmd-flush.c
  lib/smb2-cmd-ioctl.c
  lib/smb2-cmd-lock.c
  lib/smb2-cmd-logoff.c
  lib/smb2-cmd-negotiate.c
  lib/smb2-cmd-notify-change.c
  lib/smb2-cmd-oplock-break.c
  lib/smb2-cmd-query-directory.c
  lib/smb2-cmd-query-info.c
  lib/smb2-cmd-read.c
  lib/smb2-cmd-session-setup.c
  lib/smb2-cmd-set-info.c
  lib/smb2-cmd-tree-connect.c
  lib/smb2-cmd-tree-disconnect.c
  lib/smb2-cmd-write.c
  lib/smb2-data-file-info.c
  lib/smb2-data-filesystem-info.c
  lib/smb2-data-security-descriptor.c
  lib/smb2-data-reparse-point.c
  lib/smb2-share-enum.c
  lib/smb3-seal.c
  lib/smb2-signing.c
  lib/socket.c
  lib/spnego-wrapper.c
  lib/sync.c
  lib/timestamps.c
  lib/unicode.c
  lib/usha.c
)

idf_component_register(
  SRCS ${COMPONENT_SRCS}
  INCLUDE_DIRS "include" "include/esp" "include/smb2" "lib"
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

