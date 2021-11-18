#pragma once

/*
   Copyright (C) 2019 by Ronnie Sahlberg <ronniesahlberg@gmail.com>

   This program is free software; you can redistribute it and/or modify
   it under the terms of the GNU Lesser General Public License as published by
   the Free Software Foundation; either version 2.1 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU Lesser General Public License for more details.

   You should have received a copy of the GNU Lesser General Public License
   along with this program; if not, see <http://www.gnu.org/licenses/>.
*/

#include "aes.h"

namespace Security
{
    class AES128CCM : public AES
    {
        static void aes_ccm_generate_b0(unsigned char* nonce, int nlen, int alen, int plen, int mlen, unsigned char* buf);
        static void bxory(unsigned char* b, unsigned char* y, int num);
        static void ccm_generate_T(
            unsigned char* key, unsigned char* nonce, int nlen, unsigned char* aad, int alen, unsigned char* p, int plen, unsigned char* m, int mlen);
        static void ccm_generate_s(unsigned char* key, unsigned char* nonce, int nlen, int plen, int i, unsigned char* s);
        static void aes_ccm_crypt(unsigned char* key, unsigned char* nonce, int nlen, unsigned char* p, int plen);

    public:
        static void aes128ccm_encrypt(unsigned char* key,
            unsigned char* nonce, int nlen,
            unsigned char* aad, int alen,
            unsigned char* p, int plen,
            unsigned char* m, int mlen);

        static int aes128ccm_decrypt(unsigned char* key,
            unsigned char* nonce, int nlen,
            unsigned char* aad, int alen,
            unsigned char* p, int plen,
            unsigned char* m, int mlen);
    };
}