// See RFC 4634 for details
/*
 *  Description:
 *     This file implements a unified interface to the SHA algorithms.
 */

#include "sha.h"

namespace Security {
    /*
     *  USHAReset
     *
     *  Description:
     *      This function will initialize the SHA Context in preparation
     *      for computing a new SHA message digest.
     *
     *  Parameters:
     *      context: [in/out]
     *          The context to reset.
     *      whichSha: [in]
     *          Selects which SHA reset to call
     *
     *  Returns:
     *      sha Error Code.
     *
     */
    SHAResult USHA::Reset(SHAVersion whichSha)
    {
        switch (whichVersion)
        {
        case SHAVersion::SHA256:
        case SHAVersion::SHA384:
        case SHAVersion::SHA512: break;
        default: return SHAResult::BadParam;
        }

        whichVersion = whichSha;
        return Select()->Reset();
    }

    /*
     *  USHAInput
     *
     *  Description:
     *      This function accepts an array of octets as the next portion
     *      of the message.
     *
     *  Parameters:
     *      context: [in/out]
     *          The SHA context to update
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
    SHAResult USHA::Input(const uint8_t* bytes, unsigned int bytecount)
    {
        if (whichVersion == SHAVersion::None) { return SHAResult::Null; }
        return Select()->Input(bytes, bytecount);
    }

    /*
     * USHAFinalBits
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
     */
    SHAResult USHA::FinalBits(const uint8_t bits, unsigned int bitcount)
    {
        if (whichVersion == SHAVersion::None) { return SHAResult::Null; }
        return Select()->FinalBits(bits, bitcount);
    }

    /*
     * USHAResult
     *
     * Description:
     *   This function will return the 160-bit message digest into the
     *   Message_Digest array provided by the caller.
     *   NOTE: The first octet of hash is stored in the 0th element,
     *      the last octet of hash in the 19th element.
     *
     * Parameters:
     *   context: [in/out]
     *     The context to use to calculate the SHA-1 hash.
     *   Message_Digest: [out]
     *     Where the digest is returned.
     *
     * Returns:
     *   sha Error Code.
     *
     */
    SHAResult USHA::Result(uint8_t* Message_Digest)
    {
        if (whichVersion == SHAVersion::None) { return SHAResult::Null; }
        return  Select()->Result(Message_Digest);
    }

    /*
     * USHABlockSize
     *
     * Description:
     *   This function will return the blocksize for the given SHA
     *   algorithm.
     *
     * Parameters:
     *   whichSha:
     *     which SHA algorithm to query
     *
     * Returns:
     *   block size
     *
     */
    int USHA::BlockSize(SHAVersion whichSha)
    {
        switch (whichSha)
        {
        case SHAVersion::SHA256:
            return SHA256::SHA256_Message_Block_Size;
        case SHAVersion::SHA384:
            return SHA384::SHA384_Message_Block_Size;
        case SHAVersion::SHA512:
            return SHA512::SHA512_Message_Block_Size;
        default:
            return SHA512::SHA512_Message_Block_Size;
        }
    }

    /*
     * USHAHashSize
     *
     * Description:
     *   This function will return the hashsize for the given SHA
     *   algorithm.
     *
     * Parameters:
     *   whichSha:
     *     which SHA algorithm to query
     *
     * Returns:
     *   hash size
     *
     */
    int USHA::HashSize(SHAVersion whichSha)
    {
        switch (whichSha)
        {
        case SHAVersion::SHA256:
            return SHA256::SHA256HashSize;
        case SHAVersion::SHA384:
            return SHA384::SHA384HashSize;
        case SHAVersion::SHA512:
            return SHA512::SHA512HashSize;
        default:
            return SHA512::SHA512HashSize;
        }
    }

    /*
     * USHAHashSizeBits
     *
     * Description:
     *   This function will return the hashsize for the given SHA
     *   algorithm, expressed in bits.
     *
     * Parameters:
     *   whichSha:
     *     which SHA algorithm to query
     *
     * Returns:
     *   hash size in bits
     *
     */
    int USHA::HashSizeBits(SHAVersion whichSha)
    {
        switch (whichSha)
        {
        case SHAVersion::SHA256:
            return SHA256::SHA256HashSizeBits;
        case SHAVersion::SHA384:
            return SHA384::SHA384HashSizeBits;
        case SHAVersion::SHA512:
            return SHA512::SHA512HashSizeBits;
        default:
            return SHA512::SHA512HashSizeBits;
        }
    }
}