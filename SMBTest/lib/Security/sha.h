#pragma once

#include <cstdint>
#include <utility>

/*
 *  Description:
 *      This file implements the Secure Hash Signature Standard
 *      algorithms as defined in the National Institute of Standards
 *      and Technology Federal Information Processing Standards
 *      Publication (FIPS PUB) 180-1 published on April 17, 1995, 180-2
 *      published on August 1, 2002, and the FIPS PUB 180-2 Change
 *      Notice published on February 28, 2004.
 *
 *      A combined document showing all algorithms is available at
 *              http://csrc.nist.gov/publications/fips/
 *              fips180-2/fips180-2withchangenotice.pdf
 *
 *      The five hashes are defined in these sizes:
 *              SHA-1           20 byte / 160 bit
 *              SHA-224         28 byte / 224 bit
 *              SHA-256         32 byte / 256 bit
 *              SHA-384         48 byte / 384 bit
 *              SHA-512         64 byte / 512 bit
 */

namespace Security {
    enum struct SHAVersion {
        None,
        SHA384,
        SHA512,
        SHA256,

    };

    enum struct SHAResult {
        Success = 0,
        Null,         /* Null pointer parameter */
        InputTooLong, /* input data too long */
        StateError,   /* called Input after FinalBits or Result */
        BadParam      /* passed a bad parameter */
    };

    class SHA {
    public:
        virtual SHAResult Reset() = 0;
        virtual SHAResult Input(const uint8_t* bytes, unsigned int bytecount) = 0;
        virtual SHAResult FinalBits(const uint8_t bits, unsigned int bitcount) = 0;
        virtual SHAResult Result(uint8_t* Message_Digest) = 0;
    };

    /*
     *  Function Prototypes
     */

    class SHA256 : public SHA {
    public:
        static const int SHA256_Message_Block_Size = 64;
        static const int SHA256HashSize = 32;
        static const int SHA256HashSizeBits = 256;

    private:
        /*
         *  This structure will hold context information for the SHA-256
         *  hashing operation.
         */
        uint32_t Intermediate_Hash[SHA256HashSize / 4]; /* Message Digest */

        uint32_t Length_Low;  /* Message length in bits */
        uint32_t Length_High; /* Message length in bits */

        int_least16_t Message_Block_Index; /* Message_Block array index */
        /* 512-bit message blocks */
        uint8_t Message_Block[SHA256_Message_Block_Size];

        int Computed;  /* Is the digest computed? */
        SHAResult Corrupted; /* Is the digest corrupted? */

        void SHA224_256Finalize(uint8_t Pad_Byte);
        void SHA224_256PadMessage(uint8_t Pad_Byte);
        void SHA224_256ProcessMessageBlock();
        SHAResult SHA224_256Reset(uint32_t* H0);
        SHAResult SHA224_256ResultN(uint8_t Message_Digest[], int HashSize);

        static uint32_t SHA256_H0[SHA256::SHA256HashSize / 4];

    public:
        /* SHA-256 */
        SHAResult Reset() override;
        SHAResult Input(const uint8_t* bytes, unsigned int bytecount) override;
        SHAResult FinalBits(const uint8_t bits, unsigned int bitcount) override;
        SHAResult Result(uint8_t* Message_Digest) override;
    };

    class SHA512 : public SHA {
    public:

        static const int SHA512_Message_Block_Size = 128;
        static const int SHA512HashSize = 64;
        static const int SHA512HashSizeBits = 512;

    protected:
        /*
         *  This structure will hold context information for the SHA-512
         *  hashing operation.
         */
        uint64_t Intermediate_Hash[SHA512HashSize / 8]; /* Message Digest */
        uint64_t Length_Low, Length_High;               /* Message length in bits */

        int_least16_t Message_Block_Index;              /* Message_Block array index */
        /* 1024-bit message blocks */
        uint8_t Message_Block[SHA512_Message_Block_Size];

        int Computed;  /* Is the digest computed? */
        SHAResult Corrupted; /* Is the digest corrupted? */

        void SHA384_512Finalize(uint8_t Pad_Byte);
        void SHA384_512PadMessage(uint8_t Pad_Byte);
        void SHA384_512ProcessMessageBlock();
        SHAResult SHA384_512Reset(uint64_t H0[]);
        SHAResult SHA384_512ResultN(uint8_t Message_Digest[], int HashSize);

    public:
        SHAResult Reset() override;
        SHAResult Input(const uint8_t* bytes, unsigned int bytecount) override;
        SHAResult FinalBits(const uint8_t bits, unsigned int bitcount) override;
        SHAResult Result(uint8_t* Message_Digest) override;
    };

    class SHA384 : public SHA512 {
    public:
        static const int SHA384_Message_Block_Size = 128;
        static const int SHA384HashSize = 48;
        static const int SHA384HashSizeBits = 384;

        /*
         *  This structure will hold context information for the SHA-384
         *  hashing operation. It uses the SHA-512 structure for computation.
         */

        SHAResult Reset() override;
        SHAResult Input(const uint8_t* bytes, unsigned int bytecount) override;
        SHAResult FinalBits(const uint8_t bits, unsigned int bitcount) override;
        SHAResult Result(uint8_t* Message_Digest) override;
    };

    class USHA : public SHA {
        SHA* current = nullptr;
        SHAVersion whichVersion = SHAVersion::None;

        inline SHA* Select() {
            return current;
        }

    public:
        USHA() = default;
        USHA(USHA&& o) {
            std::swap(current, o.current);
        }
        USHA(const USHA& o) = delete;

        // Constants from biggest type of SHA (SHA512 at the moment)
        static const int USHA_Max_Message_Block_Size = SHA512::SHA512_Message_Block_Size;
        static const int USHAMaxHashSize = SHA512::SHA512HashSize;
        static const int USHAMaxHashSizeBits = SHA512::SHA512HashSizeBits;

        /* Unified SHA functions, chosen by whichSha */
        SHAResult Reset(SHAVersion whichSha);
        SHAResult Reset() override { return Reset(SHAVersion::SHA512); }
        SHAResult Input(const uint8_t* bytes, unsigned int bytecount) override;
        SHAResult FinalBits(const uint8_t bits, unsigned int bitcount) override;
        SHAResult Result(uint8_t* Message_Digest) override;


        static int BlockSize(SHAVersion whichSha);
        static int HashSize(SHAVersion whichSha);
        static int HashSizeBits(SHAVersion whichSha);

        ~USHA() {
            if (current) { delete current; current = nullptr; }
        }
    };

    class HMAC {
        /*
         *  This structure will hold context information for the HMAC
         *  keyed hashing operation.
         */

        SHAVersion whichSha;   /* which SHA is being used */
        int hashSize;   /* hash size of SHA being used */
        int blockSize;  /* block size of SHA being used */
        USHA shaContext; /* SHA context */
        unsigned char k_opad[USHA::USHA_Max_Message_Block_Size]; /* outer padding - key XORd with opad */

    public:
        /*
         * HMAC Keyed-Hashing for Message Authentication, RFC2104,
         * for all SHAs.
         *
         * This interface allows a fixed-length text input to be used.
         */
        static SHAResult CalculateDigest(
            SHAVersion           whichSha,   /* which SHA algorithm to use */
            const unsigned char* text,        /* pointer to data stream */
            int                  text_len,    /* length of data stream */
            const unsigned char* key,         /* pointer to authentication key */
            int                  key_len,     /* length of authentication key */
            uint8_t* digest);    /* caller digest to fill in */

/*
 * HMAC Keyed-Hashing for Message Authentication, RFC2104,
 * for all SHAs.
 * This interface allows any length of text input to be used.
 */
        SHAResult Reset(SHAVersion whichSha, const unsigned char* key, int key_len);
        SHAResult Input(const unsigned char* text, int text_len);
        SHAResult FinalBits(const uint8_t bits, unsigned int bitcount);
        SHAResult Result(uint8_t* digest);
    };
}