#pragma once

/* Original code from https://github.com/bitdust/tiny-AES128-C, Licenced as Public Domain
 * Heavily modified into a C++ version by Stefan de Bruijn.
 */

#include <cstdint>

namespace Security
{
    class AES {
    private:
        // state - array holding the intermediate results during decryption.
        typedef uint8_t state_t[4][4];

        // The number of columns comprising a state in AES. This is a constant in AES. Value=4
        static const int Nb = 4;
        // The number of 32 bit words in a key.
        static const int Nk = 4;
        // Key length in bytes [128 bit]
        static const int KEYLEN = 16;
        // The number of rounds in AES Cipher.
        static const int Nr = 10;

        static uint8_t getSBoxValue(uint8_t num);
        static uint8_t getSBoxInvert(uint8_t num);
        static void KeyExpansion(const uint8_t* Key, uint8_t* roundKey);
        static void AddRoundKey(uint8_t* roundKey, state_t* state, uint8_t round);
        static void SubBytes(state_t* state);
        static void ShiftRows(state_t* state);
        static uint8_t xtime(uint8_t x);
        static void MixColumns(state_t* state);
        static uint8_t Multiply(uint8_t x, uint8_t y);
        static void InvMixColumns(state_t* state);
        static void InvSubBytes(state_t* state);
        static void InvShiftRows(state_t* state);
        static void Cipher(uint8_t* roundKey, state_t* state);
        static void InvCipher(uint8_t* roundKey, state_t* state);
        static void BlockCopy(uint8_t* output, uint8_t* input);

    public:
        static void AES128_ECB_encrypt(uint8_t* input, const uint8_t* key, uint8_t* output);
        static void AES128_ECB_decrypt(uint8_t* input, const uint8_t* key, uint8_t* output);
    };
}