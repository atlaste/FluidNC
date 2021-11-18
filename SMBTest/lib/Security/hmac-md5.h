#pragma once

/* From RFC1204  HMAC-MD5 */

#include <cstdint>

namespace Security {
    class HMAC_MD5 {
    public:
        static void CalculateDigest(unsigned char* text, int text_len, unsigned char* key, int key_len, unsigned char* digest);
    };
}