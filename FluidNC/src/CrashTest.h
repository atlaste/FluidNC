#pragma once

#include "FluidPath.h"

class CrashTest {
    static void test_detail(void*) {
        while (true) {
            std::error_code ec;
            FluidPath       fpath { "/localfs", "", ec };
            auto            iter = stdfs::directory_iterator { fpath, ec };
            if (!ec) {
                for (auto& dir_entry : iter) {
                    std::string filename = dir_entry.path().filename();

                    char  buf[512];
                    FILE* fp = fopen(filename.c_str(), "rb");
                    if (fp) {
                        while (fread(buf, 1, 512, fp) > 0) {}
                        fclose(fp);
                    }
                }
            }
        }
    }

public:
    static void RunTest() {
        xTaskCreate(test_detail,
                    "IRAMTest",
                    3048,
                    NULL,
                    5,  // priority
                    NULL);
    }
};
