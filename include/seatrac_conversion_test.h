#pragma once

#include "seatrac_conversion.h"
#include <assert.h>
#include <stdint.h>

void test_uint16_t_conversion()
{
    uint8_t buf[8];

    uint16_t test_out, test_in;
    test_in = 5;
    seatrac_uint16_t_to_bytes(buf, test_in);
    seatrac_bytes_to_uint16_t(buf, &test_out);

    test_in = 0xffff;
    seatrac_uint16_t_to_bytes(buf, test_in);
    seatrac_bytes_to_uint16_t(buf, &test_out);
}

void test_int16_t_conversion()
{
    uint8_t buf[8];

    int16_t test_out, test_in;
    test_in = 5;
    seatrac_int16_t_to_bytes(buf, test_in);
    seatrac_bytes_to_int16_t(buf, &test_out);

    test_in = -5;
    seatrac_int16_t_to_bytes(buf, test_in);
    seatrac_bytes_to_int16_t(buf, &test_out);

    test_in = 0x7fff;
    seatrac_int16_t_to_bytes(buf, test_in);
    seatrac_bytes_to_int16_t(buf, &test_out);

    test_in = 0x8000;
    seatrac_int16_t_to_bytes(buf, test_in);
    seatrac_bytes_to_int16_t(buf, &test_out);
}

void test_uint32_t_conversion()
{
    uint8_t buf[8];

    uint32_t test_out, test_in;
    test_in = 5;
    seatrac_uint32_t_to_bytes(buf, test_in);
    seatrac_bytes_to_uint32_t(buf, &test_out);

    test_in = 0xffffffff;
    seatrac_uint32_t_to_bytes(buf, test_in);
    seatrac_bytes_to_uint32_t(buf, &test_out);
}

void test_int32_t_conversion()
{
    uint8_t buf[8];

    int32_t test_out, test_in;
    test_in = 5;
    seatrac_int32_t_to_bytes(buf, test_in);
    seatrac_bytes_to_int32_t(buf, &test_out);

    test_in = -5;
    seatrac_int32_t_to_bytes(buf, test_in);
    seatrac_bytes_to_int32_t(buf, &test_out);

    test_in = 0x7fffffff;
    seatrac_int32_t_to_bytes(buf, test_in);
    seatrac_bytes_to_int32_t(buf, &test_out);

    test_in = 0x80000000;
    seatrac_int32_t_to_bytes(buf, test_in);
    seatrac_bytes_to_int32_t(buf, &test_out);
}