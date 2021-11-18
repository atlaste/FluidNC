/********************* See RFC 4634 for details *********************/
/*
 * Description:
 *   This file implements the Secure Hash Signature Standard
 *   algorithms as defined in the National Institute of Standards
 *   and Technology Federal Information Processing Standards
 *   Publication (FIPS PUB) 180-1 published on April 17, 1995, 180-2
 *   published on August 1, 2002, and the FIPS PUB 180-2 Change
 *   Notice published on February 28, 2004.
 *
 *   A combined document showing all algorithms is available at
 *       http://csrc.nist.gov/publications/fips/
 *       fips180-2/fips180-2withchangenotice.pdf
 *
 *   The SHA-384 and SHA-512 algorithms produce 384-bit and 512-bit
 *   message digests for a given data stream. It should take about
 *   2**n steps to find a message with the same digest as a given
 *   message and 2**(n/2) to find any two messages with the same
 *   digest, when n is the digest size in bits. Therefore, this
 *   algorithm can serve as a means of providing a
 *   "fingerprint" for a message.
 *
 * Portability Issues:
 *   SHA-384 and SHA-512 are defined in terms of 64-bit "words",
 *   but if USE_32BIT_ONLY is #defined, this code is implemented in
 *   terms of 32-bit "words". This code uses <stdint.h> (included
 *   via "sha.h") to define the 64, 32 and 8 bit unsigned integer
 *   types. If your C compiler does not support 64 bit unsigned
 *   integers, and you do not #define USE_32BIT_ONLY, this code is
 *   not appropriate.
 *
 * Caveats:
 *   SHA-384 and SHA-512 are designed to work with messages less
 *   than 2^128 bits long. This implementation uses
 *   SHA384/512Input() to hash the bits that are a multiple of the
 *   size of an 8-bit character, and then uses SHA384/256FinalBits()
 *   to hash the final few bits of the input.
 *
 */

#include "sha.h"
#include "sha-private.h"

 /* Define the SHA shift, rotate left and rotate right macro */
#define SHA512_SHR(bits,word)  (((uint64_t)(word)) >> (bits))
#define SHA512_ROTR(bits,word) ((((uint64_t)(word)) >> (bits)) | \
                                (((uint64_t)(word)) << (64-(bits))))

/* Define the SHA SIGMA and sigma macros */
#define SHA512_SIGMA0(word)   \
 (SHA512_ROTR(28,word) ^ SHA512_ROTR(34,word) ^ SHA512_ROTR(39,word))
#define SHA512_SIGMA1(word)   \
 (SHA512_ROTR(14,word) ^ SHA512_ROTR(18,word) ^ SHA512_ROTR(41,word))
#define SHA512_sigma0(word)   \
 (SHA512_ROTR( 1,word) ^ SHA512_ROTR( 8,word) ^ SHA512_SHR( 7,word))
#define SHA512_sigma1(word)   \
 (SHA512_ROTR(19,word) ^ SHA512_ROTR(61,word) ^ SHA512_SHR( 6,word))

namespace Security {
    /*
     * add "length" to the length
     */
    static uint64_t addTemp;

#define SHA384_512AddLength(length)                   \
   (addTemp = Length_Low, Corrupted =        \
    ((Length_Low += length) < addTemp) &&             \
    (++Length_High == 0) ? SHAResult::Null : SHAResult::Success)

    /* Initial Hash Values: FIPS-180-2 sections 5.3.3 and 5.3.4 */
    static uint64_t SHA384_H0[] = {
      0xCBBB9D5DC1059ED8ull, 0x629A292A367CD507ull, 0x9159015A3070DD17ull,
      0x152FECD8F70E5939ull, 0x67332667FFC00B31ull, 0x8EB44A8768581511ull,
      0xDB0C2E0D64F98FA7ull, 0x47B5481DBEFA4FA4ull
    };

    static uint64_t SHA512_H0[] = {
      0x6A09E667F3BCC908ull, 0xBB67AE8584CAA73Bull, 0x3C6EF372FE94F82Bull,
      0xA54FF53A5F1D36F1ull, 0x510E527FADE682D1ull, 0x9B05688C2B3E6C1Full,
      0x1F83D9ABFB41BD6Bull, 0x5BE0CD19137E2179ull
    };


    /*
     * SHA384Reset
     *
     * Description:
     *   This function will initialize the SHA384Context in preparation
     *   for computing a new SHA384 message digest.
     *
     * Parameters:
     *   context: [in/out]
     *     The context to reset.
     *
     * Returns:
     *   sha Error Code.
     *
     */
    SHAResult SHA384::Reset()
    {
        return SHA384_512Reset(SHA384_H0);
    }

    /*
     * SHA384Input
     *
     * Description:
     *   This function accepts an array of octets as the next portion
     *   of the message.
     *
     * Parameters:
     *   context: [in/out]
     *     The SHA context to update
     *   message_array: [in]
     *     An array of characters representing the next portion of
     *     the message.
     *   length: [in]
     *     The length of the message in message_array
     *
     * Returns:
     *   sha Error Code.
     *
     */
    SHAResult SHA384::Input(const uint8_t* message_array, unsigned int length)
    {
        return SHA512::Input(message_array, length);
    }

    /*
     * SHA384FinalBits
     *
     * Description:
     *   This function will add in any final bits of the message.
     *
     * Parameters:
     *   context: [in/out]
     *     The SHA context to update
     *   message_bits: [in]
     *     The final bits of the message, in the upper portion of the
     *     byte. (Use 0b###00000 instead of 0b00000### to input the
     *     three bits ###.)
     *   length: [in]
     *     The number of bits in message_bits, between 1 and 7.
     *
     * Returns:
     *   sha Error Code.
     *
     */
    SHAResult SHA384::FinalBits(const uint8_t message_bits, unsigned int length)
    {
        return SHA512::FinalBits(message_bits, length);
    }

    /*
     * SHA384Result
     *
     * Description:
     *   This function will return the 384-bit message
     *   digest into the Message_Digest array provided by the caller.
     *   NOTE: The first octet of hash is stored in the 0th element,
     *      the last octet of hash in the 48th element.
     *
     * Parameters:
     *   context: [in/out]
     *     The context to use to calculate the SHA hash.
     *   Message_Digest: [out]
     *     Where the digest is returned.
     *
     * Returns:
     *   sha Error Code.
     *
     */
    SHAResult SHA384::Result(uint8_t* Message_Digest)
    {
        return SHA384_512ResultN(Message_Digest, SHA384HashSize);
    }

    /*
     * SHA512Reset
     *
     * Description:
     *   This function will initialize the SHA512Context in preparation
     *   for computing a new SHA512 message digest.
     *
     * Parameters:
     *   context: [in/out]
     *     The context to reset.
     *
     * Returns:
     *   sha Error Code.
     *
     */
    SHAResult SHA512::Reset()
    {
        return SHA384_512Reset(SHA512_H0);
    }

    /*
     * SHA512Input
     *
     * Description:
     *   This function accepts an array of octets as the next portion
     *   of the message.
     *
     * Parameters:
     *   context: [in/out]
     *     The SHA context to update
     *   message_array: [in]
     *     An array of characters representing the next portion of
     *     the message.
     *   length: [in]
     *     The length of the message in message_array
     *
     * Returns:
     *   sha Error Code.
     *
     */
    SHAResult SHA512::Input(const uint8_t* message_array, unsigned int length)
    {
        if (!length)
            return SHAResult::Success;

        if (!message_array)
            return SHAResult::Null;

        if (Computed)
        {
            Corrupted = SHAResult::StateError;
            return SHAResult::StateError;
        }

        if (Corrupted != SHAResult::Success)
            return Corrupted;

        while (length-- && Corrupted == SHAResult::Success)
        {
            Message_Block[Message_Block_Index++] =
                (*message_array & 0xFF);

            if (SHA384_512AddLength(8) == SHAResult::Success &&
                (Message_Block_Index == SHA512_Message_Block_Size))
                SHA384_512ProcessMessageBlock();

            message_array++;
        }

        return SHAResult::Success;
    }

    /*
     * SHA512FinalBits
     *
     * Description:
     *   This function will add in any final bits of the message.
     *
     * Parameters:
     *   context: [in/out]
     *     The SHA context to update
     *   message_bits: [in]
     *     The final bits of the message, in the upper portion of the
     *     byte. (Use 0b###00000 instead of 0b00000### to input the
     *     three bits ###.)
     *   length: [in]
     *     The number of bits in message_bits, between 1 and 7.
     *
     * Returns:
     *   sha Error Code.
     *
     */
    SHAResult SHA512::FinalBits(const uint8_t message_bits, unsigned int length)
    {
        uint8_t masks[8] = {
            /* 0 0b00000000 */ 0x00, /* 1 0b10000000 */ 0x80,
            /* 2 0b11000000 */ 0xC0, /* 3 0b11100000 */ 0xE0,
            /* 4 0b11110000 */ 0xF0, /* 5 0b11111000 */ 0xF8,
            /* 6 0b11111100 */ 0xFC, /* 7 0b11111110 */ 0xFE
        };
        uint8_t markbit[8] = {
            /* 0 0b10000000 */ 0x80, /* 1 0b01000000 */ 0x40,
            /* 2 0b00100000 */ 0x20, /* 3 0b00010000 */ 0x10,
            /* 4 0b00001000 */ 0x08, /* 5 0b00000100 */ 0x04,
            /* 6 0b00000010 */ 0x02, /* 7 0b00000001 */ 0x01
        };

        if (!length)
            return SHAResult::Success;

        if ((Computed) || (length >= 8) || (length == 0))
        {
            Corrupted = SHAResult::StateError;
            return SHAResult::StateError;
        }

        if (Corrupted != SHAResult::Success)
            return Corrupted;

        SHA384_512AddLength(length);
        SHA384_512Finalize((uint8_t)((message_bits & masks[length]) | markbit[length]));

        return SHAResult::Success;
    }

    /*
     * SHA384_512Finalize
     *
     * Description:
     *   This helper function finishes off the digest calculations.
     *
     * Parameters:
     *   context: [in/out]
     *     The SHA context to update
     *   Pad_Byte: [in]
     *     The last byte to add to the digest before the 0-padding
     *     and length. This will contain the last bits of the message
     *     followed by another single bit. If the message was an
     *     exact multiple of 8-bits long, Pad_Byte will be 0x80.
     *
     * Returns:
     *   sha Error Code.
     *
     */
    void SHA512::SHA384_512Finalize(uint8_t Pad_Byte)
    {
        int_least16_t i;
        SHA384_512PadMessage(Pad_Byte);
        /* message may be sensitive, clear it out */
        for (i = 0; i < SHA512_Message_Block_Size; ++i)
            Message_Block[i] = 0;
        Length_Low = 0;
        Length_High = 0;
        Computed = 1;
    }

    /*
     * SHA512Result
     *
     * Description:
     *   This function will return the 512-bit message
     *   digest into the Message_Digest array provided by the caller.
     *   NOTE: The first octet of hash is stored in the 0th element,
     *      the last octet of hash in the 64th element.
     *
     * Parameters:
     *   context: [in/out]
     *     The context to use to calculate the SHA hash.
     *   Message_Digest: [out]
     *     Where the digest is returned.
     *
     * Returns:
     *   sha Error Code.
     *
     */
    SHAResult SHA512::Result(uint8_t* Message_Digest)
    {
        return SHA384_512ResultN(Message_Digest, SHA512HashSize);
    }

    /*
     * SHA384_512PadMessage
     *
     * Description:
     *   According to the standard, the message must be padded to an
     *   even 1024 bits. The first padding bit must be a '1'. The
     *   last 128 bits represent the length of the original message.
     *   All bits in between should be 0. This helper function will
     *   pad the message according to those rules by filling the
     *   Message_Block array accordingly. When it returns, it can be
     *   assumed that the message digest has been computed.
     *
     * Parameters:
     *   context: [in/out]
     *     The context to pad
     *   Pad_Byte: [in]
     *     The last byte to add to the digest before the 0-padding
     *     and length. This will contain the last bits of the message
     *     followed by another single bit. If the message was an
     *     exact multiple of 8-bits long, Pad_Byte will be 0x80.
     *
     * Returns:
     *   Nothing.
     *
     */
    void SHA512::SHA384_512PadMessage(uint8_t Pad_Byte)
    {
        /*
         * Check to see if the current message block is too small to hold
         * the initial padding bits and length. If so, we will pad the
         * block, process it, and then continue padding into a second
         * block.
         */
        if (Message_Block_Index >= (SHA512_Message_Block_Size - 16))
        {
            Message_Block[Message_Block_Index++] = Pad_Byte;
            while (Message_Block_Index < SHA512_Message_Block_Size)
                Message_Block[Message_Block_Index++] = 0;

            SHA384_512ProcessMessageBlock();
        }
        else
            Message_Block[Message_Block_Index++] = Pad_Byte;

        while (Message_Block_Index < (SHA512_Message_Block_Size - 16))
            Message_Block[Message_Block_Index++] = 0;

        /*
         * Store the message length as the last 16 octets
         */
        Message_Block[112] = (uint8_t)(Length_High >> 56);
        Message_Block[113] = (uint8_t)(Length_High >> 48);
        Message_Block[114] = (uint8_t)(Length_High >> 40);
        Message_Block[115] = (uint8_t)(Length_High >> 32);
        Message_Block[116] = (uint8_t)(Length_High >> 24);
        Message_Block[117] = (uint8_t)(Length_High >> 16);
        Message_Block[118] = (uint8_t)(Length_High >> 8);
        Message_Block[119] = (uint8_t)(Length_High);

        Message_Block[120] = (uint8_t)(Length_Low >> 56);
        Message_Block[121] = (uint8_t)(Length_Low >> 48);
        Message_Block[122] = (uint8_t)(Length_Low >> 40);
        Message_Block[123] = (uint8_t)(Length_Low >> 32);
        Message_Block[124] = (uint8_t)(Length_Low >> 24);
        Message_Block[125] = (uint8_t)(Length_Low >> 16);
        Message_Block[126] = (uint8_t)(Length_Low >> 8);
        Message_Block[127] = (uint8_t)(Length_Low);

        SHA384_512ProcessMessageBlock();
    }

    /*
     * SHA384_512ProcessMessageBlock
     *
     * Description:
     *   This helper function will process the next 1024 bits of the
     *   message stored in the Message_Block array.
     *
     * Parameters:
     *   context: [in/out]
     *     The SHA context to update
     *
     * Returns:
     *   Nothing.
     *
     * Comments:
     *   Many of the variable names in this code, especially the
     *   single character names, were used because those were the
     *   names used in the publication.
     *
     *
     */
    void SHA512::SHA384_512ProcessMessageBlock()
    {
        /* Constants defined in FIPS-180-2, section 4.2.3 */
        static const uint64_t K[80] = {
          0x428A2F98D728AE22ull, 0x7137449123EF65CDull, 0xB5C0FBCFEC4D3B2Full,
          0xE9B5DBA58189DBBCull, 0x3956C25BF348B538ull, 0x59F111F1B605D019ull,
          0x923F82A4AF194F9Bull, 0xAB1C5ED5DA6D8118ull, 0xD807AA98A3030242ull,
          0x12835B0145706FBEull, 0x243185BE4EE4B28Cull, 0x550C7DC3D5FFB4E2ull,
          0x72BE5D74F27B896Full, 0x80DEB1FE3B1696B1ull, 0x9BDC06A725C71235ull,
          0xC19BF174CF692694ull, 0xE49B69C19EF14AD2ull, 0xEFBE4786384F25E3ull,
          0x0FC19DC68B8CD5B5ull, 0x240CA1CC77AC9C65ull, 0x2DE92C6F592B0275ull,
          0x4A7484AA6EA6E483ull, 0x5CB0A9DCBD41FBD4ull, 0x76F988DA831153B5ull,
          0x983E5152EE66DFABull, 0xA831C66D2DB43210ull, 0xB00327C898FB213Full,
          0xBF597FC7BEEF0EE4ull, 0xC6E00BF33DA88FC2ull, 0xD5A79147930AA725ull,
          0x06CA6351E003826Full, 0x142929670A0E6E70ull, 0x27B70A8546D22FFCull,
          0x2E1B21385C26C926ull, 0x4D2C6DFC5AC42AEDull, 0x53380D139D95B3DFull,
          0x650A73548BAF63DEull, 0x766A0ABB3C77B2A8ull, 0x81C2C92E47EDAEE6ull,
          0x92722C851482353Bull, 0xA2BFE8A14CF10364ull, 0xA81A664BBC423001ull,
          0xC24B8B70D0F89791ull, 0xC76C51A30654BE30ull, 0xD192E819D6EF5218ull,
          0xD69906245565A910ull, 0xF40E35855771202Aull, 0x106AA07032BBD1B8ull,
          0x19A4C116B8D2D0C8ull, 0x1E376C085141AB53ull, 0x2748774CDF8EEB99ull,
          0x34B0BCB5E19B48A8ull, 0x391C0CB3C5C95A63ull, 0x4ED8AA4AE3418ACBull,
          0x5B9CCA4F7763E373ull, 0x682E6FF3D6B2B8A3ull, 0x748F82EE5DEFB2FCull,
          0x78A5636F43172F60ull, 0x84C87814A1F0AB72ull, 0x8CC702081A6439ECull,
          0x90BEFFFA23631E28ull, 0xA4506CEBDE82BDE9ull, 0xBEF9A3F7B2C67915ull,
          0xC67178F2E372532Bull, 0xCA273ECEEA26619Cull, 0xD186B8C721C0C207ull,
          0xEADA7DD6CDE0EB1Eull, 0xF57D4F7FEE6ED178ull, 0x06F067AA72176FBAull,
          0x0A637DC5A2C898A6ull, 0x113F9804BEF90DAEull, 0x1B710B35131C471Bull,
          0x28DB77F523047D84ull, 0x32CAAB7B40C72493ull, 0x3C9EBE0A15C9BEBCull,
          0x431D67C49C100D4Cull, 0x4CC5D4BECB3E42B6ull, 0x597F299CFC657E2Aull,
          0x5FCB6FAB3AD6FAECull, 0x6C44198C4A475817ull
        };
        int t, t8;			/* Loop counter */
        uint64_t temp1, temp2;	/* Temporary word value */
        uint64_t W[80];		/* Word sequence */
        uint64_t A, B, C, D, E, F, G, H;	/* Word buffers */

        /*
         * Initialize the first 16 words in the array W
         */
        for (t = t8 = 0; t < 16; t++, t8 += 8)
            W[t] = ((uint64_t)(Message_Block[t8]) << 56) |
            ((uint64_t)(Message_Block[t8 + 1]) << 48) |
            ((uint64_t)(Message_Block[t8 + 2]) << 40) |
            ((uint64_t)(Message_Block[t8 + 3]) << 32) |
            ((uint64_t)(Message_Block[t8 + 4]) << 24) |
            ((uint64_t)(Message_Block[t8 + 5]) << 16) |
            ((uint64_t)(Message_Block[t8 + 6]) << 8) |
            ((uint64_t)(Message_Block[t8 + 7]));

        for (t = 16; t < 80; t++)
            W[t] = SHA512_sigma1(W[t - 2]) + W[t - 7] +
            SHA512_sigma0(W[t - 15]) + W[t - 16];

        A = Intermediate_Hash[0];
        B = Intermediate_Hash[1];
        C = Intermediate_Hash[2];
        D = Intermediate_Hash[3];
        E = Intermediate_Hash[4];
        F = Intermediate_Hash[5];
        G = Intermediate_Hash[6];
        H = Intermediate_Hash[7];

        for (t = 0; t < 80; t++)
        {
            temp1 = H + SHA512_SIGMA1(E) + SHA_Ch(E, F, G) + K[t] + W[t];
            temp2 = SHA512_SIGMA0(A) + SHA_Maj(A, B, C);
            H = G;
            G = F;
            F = E;
            E = D + temp1;
            D = C;
            C = B;
            B = A;
            A = temp1 + temp2;
        }

        Intermediate_Hash[0] += A;
        Intermediate_Hash[1] += B;
        Intermediate_Hash[2] += C;
        Intermediate_Hash[3] += D;
        Intermediate_Hash[4] += E;
        Intermediate_Hash[5] += F;
        Intermediate_Hash[6] += G;
        Intermediate_Hash[7] += H;

        Message_Block_Index = 0;
    }

    /*
     * SHA384_512Reset
     *
     * Description:
     *   This helper function will initialize the SHA512Context in
     *   preparation for computing a new SHA384 or SHA512 message
     *   digest.
     *
     * Parameters:
     *   context: [in/out]
     *     The context to reset.
     *   H0
     *     The initial hash value to use.
     *
     * Returns:
     *   sha Error Code.
     *
     */
    SHAResult SHA512::SHA384_512Reset(uint64_t H0[])
    {
        int i;

        Message_Block_Index = 0;
        Length_High = Length_Low = 0;

        for (i = 0; i < SHA512HashSize / 8; i++)
            Intermediate_Hash[i] = H0[i];

        Computed = 0;
        Corrupted = SHAResult::Success;

        return SHAResult::Success;
    }

    /*
     * SHA384_512ResultN
     *
     * Description:
     *   This helper function will return the 384-bit or 512-bit message
     *   digest into the Message_Digest array provided by the caller.
     *   NOTE: The first octet of hash is stored in the 0th element,
     *      the last octet of hash in the 48th/64th element.
     *
     * Parameters:
     *   context: [in/out]
     *     The context to use to calculate the SHA hash.
     *   Message_Digest: [out]
     *     Where the digest is returned.
     *   HashSize: [in]
     *     The size of the hash, either 48 or 64.
     *
     * Returns:
     *   sha Error Code.
     *
     */
    SHAResult SHA512::SHA384_512ResultN(uint8_t Message_Digest[], int HashSize)
    {
        int i;

        if (!Message_Digest)
            return SHAResult::Null;

        if (Corrupted != SHAResult::Success)
            return Corrupted;

        if (!Computed)
            SHA384_512Finalize(0x80);

        for (i = 0; i < HashSize; ++i)
            Message_Digest[i] = (uint8_t)
            (Intermediate_Hash[i >> 3] >> 8 * (7 - (i % 8)));

        return SHAResult::Success;
    }
}