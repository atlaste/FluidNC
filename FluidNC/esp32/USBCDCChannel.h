#pragma once

#include <sdkconfig.h>

#if defined(CONFIG_IDF_TARGET_ESP32S3)
#    include "esp32s3/USBCDCChannel.h"
#else
#    include "esp32/USBCDCChannel.h"
#endif
