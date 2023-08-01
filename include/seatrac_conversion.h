#pragma once

/* enable math defines on Windows */
#ifdef _MSC_VER
#ifndef _USE_MATH_DEFINES
#define _USE_MATH_DEFINES
#endif
#endif
#include <math.h>
#include "checksum.h"
#include "seatrac_helpers.h"
#include "seatrac_conversion.h"
#include <stdio.h>

#ifndef M_PI_2
    #define M_PI_2 ((float)asin(1))
#endif


// uint16_t
SEATRAC_HELPER void seatrac_bytes_to_uint16_t(uint8_t* data, uint16_t *tar)
{
    (*tar) = data[0];
    (*tar) = (*tar) + (data[1] << 8);
}

SEATRAC_HELPER void seatrac_uint16_t_to_bytes(uint8_t* buf, uint16_t data)
{
    buf[0] = data & 0xff;
    buf[1] = (data >> 8) & 0xff;
}


// int16_t
SEATRAC_HELPER void seatrac_bytes_to_int16_t(uint8_t* data, int16_t *tar)
{
    (*tar) = data[0];
    (*tar) = (*tar) + (data[1] << 8);
}

SEATRAC_HELPER void seatrac_int16_t_to_bytes(uint8_t* buf, int16_t data)
{
    buf[0] = data & 0xff;
    buf[1] = (data >> 8) & 0xff;
}


// uint32_t
SEATRAC_HELPER void seatrac_bytes_to_uint32_t(uint8_t* data, uint32_t *tar)
{
    (*tar) = data[0];
    (*tar) = (*tar) + (data[1] << 8) + (data[2] << 16) + (data[3] << 24);
}

SEATRAC_HELPER void seatrac_uint32_t_to_bytes(uint8_t* buf, uint32_t data)
{
    buf[0] = data & 0xff;
    buf[1] = (data >> 8) & 0xff;
    buf[2] = (data >> 16) & 0xff;
    buf[3] = (data >> 24) & 0xff;
}


// int32_t
SEATRAC_HELPER void seatrac_bytes_to_int32_t(uint8_t* data, int32_t *tar)
{
    (*tar) = data[0];
    (*tar) = (*tar) + (data[1] << 8) + (data[2] << 16) + (data[3] << 24);
}

SEATRAC_HELPER void seatrac_int32_t_to_bytes(uint8_t* buf, int32_t data)
{
    buf[0] = data & 0xff;
    buf[1] = (data >> 8) & 0xff;
    buf[2] = (data >> 16) & 0xff;
    buf[3] = (data >> 24) & 0xff;
}


// bool
SEATRAC_HELPER void seatrac_bytes_to_bool(uint8_t* data, bool *tar)
{
    if (data[0] == 0) {
        (*tar) = false;
    }
    else {
        (*tar) = true;
    }
}

SEATRAC_HELPER void seatrac_bool_to_bytes(uint8_t* buf, bool data)
{
    if (data) {
        buf[0] = 0xff;
    }
    else {
        buf[0] = 0;
    }
}