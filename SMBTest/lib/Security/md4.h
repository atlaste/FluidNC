#pragma once

/* From RFC1320 */

/* MD4.H - header file for MD4C.C
 */

 /* Copyright (C) 1991-2, RSA Data Security, Inc. Created 1991. All
    rights reserved.

    License to copy and use this software is granted provided that it
    is identified as the "RSA Data Security, Inc. MD4 Message-Digest
    Algorithm" in all material mentioning or referencing this software
    or this function.

    License is also granted to make and use derivative works provided
    that such works are identified as "derived from the RSA Data
    Security, Inc. MD4 Message-Digest Algorithm" in all material
    mentioning or referencing the derived work.

    RSA Data Security, Inc. makes no representations concerning either
    the merchantability of this software or the suitability of this
    software for any particular purpose. It is provided "as is"
    without express or implied warranty of any kind.

    These notices must be retained in any copies of any part of this
    documentation and/or software.
  */

#include <cstdint>

namespace Security {
    class MD4 {
        /* MD4 context. */
        uint32_t      state[4];   /* state (ABCD) */
        uint32_t      count[2];   /* number of bits, modulo 2^64 (lsb first) */
        unsigned char buffer[64]; /* input buffer */

        static void MD4Transform(uint32_t[4], unsigned char[64]);
        static void Encode(unsigned char*, uint32_t*, unsigned int);
        static void Decode(uint32_t*, unsigned char*, unsigned int);
        static void MD4_memcpy(unsigned char*, unsigned char*, unsigned int);
        static void MD4_memset(unsigned char*, int, unsigned int);

    public:
        void Init();
        void Update(unsigned char* buf, unsigned int count);
        void Final(unsigned char* dst);
    };
}