/* From RFC2104 */

#include "hmac-md5.h"
#include "md5.h"

#include <cstring>

namespace Security {
    /*
     * unsigned char*  text;                pointer to data stream/
     * int             text_len;            length of data stream
     * unsigned char*  key;                 pointer to authentication key
     * int             key_len;             length of authentication key
     * caddr_t         digest;              caller digest to be filled in
     */
    void HMAC_MD5::CalculateDigest(unsigned char* text, int text_len, unsigned char* key, int key_len, unsigned char* digest)
    {
        MD5 context;
        unsigned char k_ipad[65];    /* inner padding -
                                      * key XORd with ipad
                                      */
        unsigned char k_opad[65];    /* outer padding -
                                      * key XORd with opad
                                      */
        unsigned char tk[16];

        /* if key is longer than 64 bytes reset it to key=MD5(key) */
        if (key_len > 64) {
            MD5 tctx;
            tctx.Init();
            tctx.Update(key, key_len);
            tctx.Final(tk);

            key = tk;
            key_len = 16;
        }

        /*
         * the HMAC_MD5 transform looks like:
         *
         * MD5(K XOR opad, MD5(K XOR ipad, text))
         *
         * where K is an n byte key
         * ipad is the byte 0x36 repeated 64 times
         * and text is the data being protected
         */

         /* start out by storing key in pads */
        memset(k_ipad, 0, sizeof k_ipad);
        memset(k_opad, 0, sizeof k_opad);
        memmove(k_ipad, key, key_len);
        memmove(k_opad, key, key_len);

        /* XOR key with ipad and opad values */
        for (int i = 0; i < 64; i++) {
            k_ipad[i] ^= 0x36;
            k_opad[i] ^= 0x5c;
        }
        /*
         * perform inner MD5
         */
        context.Init();                   /* init context for 1st
                                              * pass */
        context.Update(k_ipad, 64);     /* start with inner pad */
        context.Update(text, text_len); /* then text of datagram */
        context.Final(digest);          /* finish up 1st pass */
        /*
         * perform outer MD5
         */
        context.Init();                   /* init context for 2nd
                                              * pass */
        context.Update(k_opad, 64);     /* start with outer pad */
        context.Update(digest, 16);     /* then results of 1st
                                              * hash */
        context.Final(digest);          /* finish up 2nd pass */
    }
}