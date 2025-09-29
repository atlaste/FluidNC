#pragma once

#include <sdkconfig.h>

#ifdef CONFIG_LITTLEFS_PAGE_SIZE

extern const char* littlefs_label;

bool littlefs_format(const char* partition_label);
bool littlefs_mount(const char* label = "littlefs", bool format = false);
void littlefs_unmount();

#endif
