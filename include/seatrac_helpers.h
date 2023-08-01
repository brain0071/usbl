#pragma once

#ifndef SEATRAC_HELPER
#define SEATRAC_HELPER static inline
#endif

#include "string.h"
#include "checksum.h"
#include "seatrac_types.h"
#include "seatrac_conversion.h"
#include <stdio.h>

//#define NDEBUG
#include <assert.h>


#ifdef SEATRAC_USE_CXX_NAMESPACE
namespace seatrac {
#endif

/*
 * Internal function to give access to the channel status for each channel
 */
#ifndef SEATRAC_GET_CHANNEL_STATUS
SEATRAC_HELPER seatrac_status_t* seatrac_get_channel_status()
{
    static seatrac_status_t m_seatrac_status;
    return &m_seatrac_status;
}
#endif


/*
 * Internal function to give access to the channel buffer for each channel
 */
#ifndef SEATRAC_GET_CHANNEL_BUFFER
SEATRAC_HELPER seatrac_message_t* seatrac_get_channel_buffer()
{
    static seatrac_message_t m_seatrac_buffer;
    return &m_seatrac_buffer;
}
#endif


/**
 * @brief Reset the status of a channel.
 */
SEATRAC_HELPER void seatrac_reset_channel_status()
{
    seatrac_status_t *status = seatrac_get_channel_status();
    status->parse_state = SEATRAC_PARSE_STATE_IDLE;
}


SEATRAC_HELPER void seatrac_checksum(seatrac_message_t* msg, uint16_t len)
{
    uint16_t checksum = CalcCRC16(msg->payload, len);
    msg->checksum = checksum;
}

SEATRAC_HELPER bool seatrac_checksum_check(seatrac_message_t* msg, uint16_t pos)
{
    uint16_t checksum;
    seatrac_checksum(msg, pos);
    seatrac_bytes_to_uint16_t(&msg->payload[pos], &checksum);

    return checksum == msg->checksum;
}


static inline bool _parse_char(uint8_t *c1)
{
    uint8_t c = (*c1) - 0x30;      //'0'
    if (c > 9) {
        c = (*c1) - 0x41 + 10;     //'A'
        if (c > 15 || c < 10) {
            return false;
        }
    }

    (*c1) = c;
    return true;
}

static inline bool _parse_byte(uint8_t *c1, uint8_t c2)
{
    if (!_parse_char(&c2)) {
        return false;
    }

    (*c1) = ((*c1) << 4) + c2;
    return true;
}



SEATRAC_HELPER uint8_t seatrac_frame_char_buffer(seatrac_message_t* rxmsg,
                                                 seatrac_status_t* status,
                                                 uint8_t c,
                                                 seatrac_message_t* r_message,
                                                 seatrac_status_t* r_seatrac_status)
{
    status->msg_received = SEATRAC_FRAMING_INCOMPLETE;

    switch (status->parse_state)
    {
    case SEATRAC_PARSE_STATE_UNINIT:
    case SEATRAC_PARSE_STATE_IDLE:
        if (c == SEATRAC_STX_Beacon2PC)
        {
            status->parse_state = SEATRAC_PARSE_STATE_GOT_STX;
        }
        break;

    case SEATRAC_PARSE_STATE_GOT_STX:
        if (_parse_char(&c)) {
            rxmsg->cid = c;
            status->parse_state = SEATRAC_PARSE_STATE_GOT_INCOMPAT_CID;
        }
        else {
            status->msg_received = 0;
            status->parse_state = SEATRAC_PARSE_STATE_IDLE;
        }
        break;

    case SEATRAC_PARSE_STATE_GOT_INCOMPAT_CID:
        if (_parse_byte(&rxmsg->cid, c)) {
            status->parse_state = SEATRAC_PARSE_STATE_GOT_COMPAT_CID;

            status->packet_idx = 0;
            rxmsg->payload[status->packet_idx++] = rxmsg->cid;
        }
        else {
            status->msg_received = 0;
            status->parse_state = SEATRAC_PARSE_STATE_IDLE;
        }
        break;

    case SEATRAC_PARSE_STATE_GOT_COMPAT_CID:
        if (_parse_char(&c)) {
            rxmsg->payload[status->packet_idx] = c;
            status->parse_state = SEATRAC_PARSE_STATE_GOT_INCOMPAT_PAYLOAD;
        }
        else if (c == SEATRAC_END_1) {
            // checksum
            if (seatrac_checksum_check(rxmsg, status->packet_idx-2)) {
                status->parse_state = SEATRAC_PARSE_STATE_GOT_INCOMPAT_END;
            }
            else {
                status->msg_received = SEATRAC_FRAMING_BAD_CRC;
                status->parse_state = SEATRAC_PARSE_STATE_IDLE;
            }
        }
        else {
            status->msg_received = 0;
            status->parse_state = SEATRAC_PARSE_STATE_IDLE;
        }
        break;

    case SEATRAC_PARSE_STATE_GOT_INCOMPAT_PAYLOAD:
        if (_parse_byte(&rxmsg->payload[status->packet_idx], c)) {
            status->packet_idx ++;
            status->parse_state = SEATRAC_PARSE_STATE_GOT_COMPAT_CID;
        }
        else {
            status->msg_received = 0;
            status->parse_state = SEATRAC_PARSE_STATE_IDLE;
        }
        break;

    case SEATRAC_PARSE_STATE_GOT_INCOMPAT_END:
        if (c == SEATRAC_END_2) {
            status->msg_received = SEATRAC_FRAMING_OK;
            if (r_message != NULL) {
                memcpy(r_message, rxmsg, sizeof(seatrac_message_t));
                r_message->len = status->packet_idx - 2;
            }
        }
        else {
            status->msg_received = SEATRAC_FRAMING_BAD_END;
        }
        status->parse_state = SEATRAC_PARSE_STATE_IDLE;
        break;
    }


   if (r_seatrac_status != NULL) {
       memcpy(r_seatrac_status, status, sizeof(seatrac_status_t));
   }

    return status->msg_received;
}



SEATRAC_HELPER uint8_t seatrac_parse_char(uint8_t c,
                                          seatrac_message_t* r_message,
                                          seatrac_status_t* r_seatrac_status)
{
    uint8_t msg_received = seatrac_frame_char_buffer(seatrac_get_channel_buffer(),
                                              seatrac_get_channel_status(),
                                              c,
                                              r_message,
                                              r_seatrac_status);
    // printf("%d\n",msg_received);

    if (msg_received == SEATRAC_FRAMING_BAD_CRC || msg_received == SEATRAC_FRAMING_BAD_END) {
        // we got a bad CRC. Treat as a parse failure
        seatrac_message_t* rxmsg = seatrac_get_channel_buffer();
        seatrac_status_t* status = seatrac_get_channel_status();
        if (c == SEATRAC_STX_Beacon2PC)
        {
            status->parse_state = SEATRAC_PARSE_STATE_GOT_STX;
            rxmsg->len = 0;
        }
        return 0;
    }

    return msg_received;
}



static inline void _byte_to_chars(uint8_t *buf, uint8_t data)
{
    buf[0] = (data >> 4);
    if (buf[0] < 10) {
        buf[0] += 0x30;                     //'0'
    }
    else {
        buf[0] = buf[0] - 10 + 0x41;        //'A'
    }

    buf[1] = data & 0xf;
    if (buf[1] < 10) {
        buf[1] += 0x30;                     //'0'
    }
    else {
        buf[1] = buf[1] - 10 + 0x41;        //'A'
    }
}

/**
 * @brief Finalize a SeaTrac message
 */
SEATRAC_HELPER void seatrac_finalize_message(seatrac_message_t* msg)
{
    uint8_t buf[SEATRAC_CORE_HEADER_LEN+SEATRAC_MAX_PAYLOAD_LEN+SEATRAC_NUM_CHECKSUM_BYTES];
    uint16_t num_bytes = msg->len+3;

    assert((num_bytes*2+3) < SEATRAC_MAX_PACKET_LEN);

    buf[0] = msg->cid;

    memcpy(&(buf[1]), msg->payload, msg->len);

    msg->checksum = CalcCRC16(buf, msg->len+1);
    seatrac_uint16_t_to_bytes(&(buf[msg->len+1]), msg->checksum);

    msg->payload[0] = SEATRAC_STX_PC2Beacon;
    for (size_t i=0; i<num_bytes; ++i) {
        _byte_to_chars(&(msg->payload[i*2+1]), buf[i]);
    }
    msg->len = num_bytes * 2 + 1;
    msg->payload[msg->len++] = SEATRAC_END_1;
    msg->payload[msg->len++] = SEATRAC_END_2;
}
