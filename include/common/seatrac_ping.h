#pragma once

// PING PROCOTOL
#include "../seatrac_types.h"
#include "common.h"


// Structs Definitions
SEATRACPACKED(
typedef struct msg_ping_req_t {
            ACOFIX_t acofix;
}) msg_ping_req_t;

SEATRACPACKED(
typedef struct msg_ping_resp_t {
            ACOFIX_t acofix;
}) msg_ping_resp_t;




static inline void seatrac_PING_SEND_pack(seatrac_message_t* msg,
                                          uint8_t dest_id,
                                          uint8_t msg_type)
{
    msg->cid = CID_PING_SEND;
    msg->payload[0] = dest_id; 
    msg->payload[1] = msg_type;
    msg->len = 2;
}

/**
 * @brief Decode a response message into a struct
 */
static inline void seatrac_PING_SEND_response_message_decode(seatrac_message_t* msg,
                                                        msg_response_status_t* resMsg)
{
    seatrac_protocol_response_status_message_decode(msg, resMsg);
}

/**
 * @brief Decode a CID_PING_REQ message into a struct
 */
static inline void seatrac_PING_REQ_message_decode(seatrac_message_t* msg,
                                                   msg_ping_req_t* resMsg)
{
    uint16_t pos = 1;
    seatrac_ACOFIX_T_decode(msg->payload, pos, &(resMsg->acofix));
}

/**
 * @brief Decode a CID_PING_RESP message into a struct
 */
static inline void seatrac_PING_RESP_message_decode(seatrac_message_t* msg,
                                                    msg_ping_resp_t* resMsg)
{
    uint16_t pos = 1;
    seatrac_ACOFIX_T_decode(msg->payload, pos, &(resMsg->acofix));
}

/**
 * @brief Decode a CID_PING_ERROR message into a struct
 */
static inline void seatrac_PING_ERROR_message_decode(seatrac_message_t* msg,
                                                     msg_response_status_t* resMsg)
{
    seatrac_protocol_response_status_message_decode(msg, resMsg);
}



// // Functions to display information of a struct.
// void disp_msg_ping_req(msg_ping_req_t *resMsg)
// {
//     disp_ACOFIX_t(&(resMsg->acofix));
// }

// void disp_msg_ping_resp(msg_ping_resp_t *resMsg)
// {
//     disp_ACOFIX_t(&(resMsg->acofix));
// }