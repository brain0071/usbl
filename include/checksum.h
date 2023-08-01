#pragma once

#if defined(SEATRAC_USE_CXX_NAMESPACE)
namespace seatrac {
#elif defined(__cplusplus)
extern "C" {
#endif

// Visual Studio versions before 2010 don't have stdint.h, so we just error out.
#if (defined _MSC_VER) && (_MSC_VER < 1600)
#error "The C-SEATRAC implementation requires Visual Studio 2010 or greater"
#endif

#include <stdint.h>

/*!
Function that computes the CRC16 value for an array of sequential bytes
stored in memory.
NB: Types uint8 and uint16 represent unsigned 8 and 16 bit integers
Respectively.
@param buf Pointer to the start of the buffer to compute the CRC16 for.
@param len The number of bytes in the buffer to compute the CRC16 for.
@result The new CRC16 value.
*/
uint16_t CalcCRC16(uint8_t* buf, uint16_t len)
{
    uint16_t poly = 0xA001;
    uint16_t crc = 0;
    for(uint16_t b = 0; b < len; b++) {
        uint8_t v = *buf;
        for(uint8_t i = 0; i < 8; i++) {
            if((v & 0x01) ^ (crc & 0x01)) {
                crc >>= 1;
                crc ^= poly;
            }
            else {
                crc >>= 1;
            }
            v >>= 1;
        }
        buf++;
    }
    return crc;
}

#if defined(SEATRAC_USE_CXX_NAMESPACE) || defined(__cplusplus)
}
#endif
