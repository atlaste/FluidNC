/**************************** hmac.c ****************************/
/******************** See RFC 4634 for details ******************/
/*
 *  Description:
 *      This file implements the HMAC algorithm (Keyed-Hashing for
 *      Message Authentication, RFC2104), expressed in terms of the
 *      various SHA algorithms.
 */

#include "sha.h"

namespace Security {
    /*
     *  hmac
     *
     *  Description:
     *      This function will compute an HMAC message digest.
     *
     *  Parameters:
     *      whichSha: [in]
     *          One of SHA1, SHA224, SHA256, SHA384, SHA512
     *      key: [in]
     *          The secret shared key.
     *      key_len: [in]
     *          The length of the secret shared key.
     *      message_array: [in]
     *          An array of characters representing the message.
     *      length: [in]
     *          The length of the message in message_array
     *      digest: [out]
     *          Where the digest is returned.
     *          NOTE: The length of the digest is determined by
     *              the value of whichSha.
     *
     *  Returns:
     *      sha Error Code.
     *
     */
    SHAResult HMAC::CalculateDigest(SHAVersion whichSha, const unsigned char* text, int text_len, const unsigned char* key, int key_len, uint8_t* digest)
    {
        HMAC hmac;
        auto result = hmac.Reset(whichSha, key, key_len);
        if (result != SHAResult::Success) { return result; }
        result = hmac.Input(text, text_len);
        if (result != SHAResult::Success) { return result; }
        return hmac.Result(digest);
    }

    /*
     *  hmacReset
     *
     *  Description:
     *      This function will initialize the hmacContext in preparation
     *      for computing a new HMAC message digest.
     *
     *  Parameters:
     *      context: [in/out]
     *          The context to reset.
     *      whichSha: [in]
     *          One of SHA1, SHA224, SHA256, SHA384, SHA512
     *      key: [in]
     *          The secret shared key.
     *      key_len: [in]
     *          The length of the secret shared key.
     *
     *  Returns:
     *      sha Error Code.
     *
     */
    SHAResult HMAC::Reset(SHAVersion whichSha, const unsigned char* key, int key_len)
    {
        /* inner padding - key XORd with ipad */
        unsigned char k_ipad[USHA::USHA_Max_Message_Block_Size];

        /* temporary buffer when keylen > blocksize */
        unsigned char tempkey[USHA::USHAMaxHashSize];

        int blocksize = blockSize = USHA::BlockSize(whichSha);
        int hashsize = hashSize = USHA::HashSize(whichSha);
        this->whichSha = whichSha;

        /*
         * If key is longer than the hash blocksize,
         * reset it to key = HASH(key).
         */
        SHAResult err;
        if (key_len > blocksize)
        {
            USHA tctx;
            err = tctx.Reset(whichSha);
            if (err != SHAResult::Success) { return err; }
            err = tctx.Input(key, key_len);
            if (err != SHAResult::Success) { return err; }
            err = tctx.Result(tempkey);
            if (err != SHAResult::Success) { return err; }

            key = tempkey;
            key_len = hashsize;
        }

        /*
         * The HMAC transform looks like:
         *
         * SHA(K XOR opad, SHA(K XOR ipad, text))
         *
         * where K is an n byte key.
         * ipad is the byte 0x36 repeated blocksize times
         * opad is the byte 0x5c repeated blocksize times
         * and text is the data being protected.
         */

         /* store key into the pads, XOR'd with ipad and opad values */
        int i;
        for (i = 0; i < key_len; i++)
        {
            k_ipad[i] = key[i] ^ 0x36;
            k_opad[i] = key[i] ^ 0x5c;
        }
        /* remaining pad bytes are '\0' XOR'd with ipad and opad values */
        for (; i < blocksize; i++)
        {
            k_ipad[i] = 0x36;
            k_opad[i] = 0x5c;
        }

        /* perform inner hash */
        /* init context for 1st pass */
        err = shaContext.Reset(whichSha);
        if (err != SHAResult::Success) { return err; }
        /* and start with inner pad */
        err = shaContext.Input(k_ipad, blocksize);
        return err;
    }

    /*
     *  hmacInput
     *
     *  Description:
     *      This function accepts an array of octets as the next portion
     *      of the message.
     *
     *  Parameters:
     *      context: [in/out]
     *          The HMAC context to update
     *      message_array: [in]
     *          An array of characters representing the next portion of
     *          the message.
     *      length: [in]
     *          The length of the message in message_array
     *
     *  Returns:
     *      sha Error Code.
     *
     */
    SHAResult HMAC::Input(const unsigned char* text, int text_len)
    {
        /* then text of datagram */
        return shaContext.Input(text, text_len);
    }

    /*
     * HMACFinalBits
     *
     * Description:
     *   This function will add in any final bits of the message.
     *
     * Parameters:
     *   context: [in/out]
     *     The HMAC context to update
     *   message_bits: [in]
     *     The final bits of the message, in the upper portion of the
     *     byte. (Use 0b###00000 instead of 0b00000### to input the
     *     three bits ###.)
     *   length: [in]
     *     The number of bits in message_bits, between 1 and 7.
     *
     * Returns:
     *   sha Error Code.
     */
    SHAResult HMAC::FinalBits(const uint8_t bits, unsigned int bitcount)
    {
        /* then final bits of datagram */
        return shaContext.FinalBits(bits, bitcount);
    }

    /*
     * HMACResult
     *
     * Description:
     *   This function will return the N-byte message digest into the
     *   Message_Digest array provided by the caller.
     *   NOTE: The first octet of hash is stored in the 0th element,
     *      the last octet of hash in the Nth element.
     *
     * Parameters:
     *   context: [in/out]
     *     The context to use to calculate the HMAC hash.
     *   digest: [out]
     *     Where the digest is returned.
     *   NOTE 2: The length of the hash is determined by the value of
     *      whichSha that was passed to hmacReset().
     *
     * Returns:
     *   sha Error Code.
     *
     */
    SHAResult HMAC::Result(uint8_t* digest)
    {
        /* finish up 1st pass */
        /* (Use digest here as a temporary buffer.) */
        SHAResult err;
        err = shaContext.Result(digest);
        if (err != SHAResult::Success) { return err; }

        /* perform outer SHA */
        /* init context for 2nd pass */
        err = shaContext.Reset(whichSha);
        if (err != SHAResult::Success) { return err; }

        /* start with outer pad */
        err = shaContext.Input(k_opad, blockSize);
        if (err != SHAResult::Success) { return err; }

        /* then results of 1st hash */
        err = shaContext.Input(digest, hashSize);
        if (err != SHAResult::Success) { return err; }

        /* finish up 2nd pass */
        err = shaContext.Result(digest);
        return err;
    }
}