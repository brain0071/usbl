

// PING PROCOTOL
#include "../seatrac_types.h"
#include "common.h"


// Structs Definitions
SEATRACPACKED(
typedef struct msg_dat_receive_t {
            ACOFIX_t acofix;
            bool ack_flag;

            uint8_t packet_len;
            uint8_t packet_data[MAX_PACKET_DATA_LEN];

            bool local_flag;
}) msg_dat_receive_t;

SEATRACPACKED(
typedef struct msg_DAT_QUEUE_SET_response_t {
            uint8_t status;
            uint8_t bid;
            uint8_t packet_len;
}) msg_DAT_QUEUE_SET_response_t;

SEATRACPACKED(
typedef struct msg_DAT_QUEUE_STATUS_response_t {
            uint8_t packet_len[16];
}) msg_DAT_QUEUE_STATUS_response_t;




static inline void seatrac_DAT_SEND_pack(seatrac_message_t* msg,
                                          uint8_t dest_id,
                                          uint8_t msg_type,
                                         uint8_t packet_len,
                                         uint8_t* packet_data)
{
    msg->cid = CID_DAT_SEND;

    msg->payload[0] = dest_id;
    msg->payload[1] = msg_type;
    msg->payload[2] = packet_len;

    msg->len = 3;
    for (uint8_t i=0; i<packet_len; ++i) {
        msg->payload[msg->len++] = packet_data[i];
    }
}

/**
 * @brief Decode a response message into a struct
 */
static inline void seatrac_DAT_SEND_response_message_decode(seatrac_message_t* msg,
                                                        msg_response_status_t* resMsg)
{
    seatrac_protocol_response_status_message_decode(msg, resMsg);
}

/**
 * @brief Decode a CID_DAT_RECEIVE message into a struct
 */
static inline void seatrac_DAT_RECEIVE_message_decode(seatrac_message_t* msg,
                                                        msg_dat_receive_t* resMsg)
{
    uint16_t pos = 1;
    seatrac_ACOFIX_T_decode(msg->payload, pos, &(resMsg->acofix));
    seatrac_bytes_to_bool(&(msg->payload[pos]), &(resMsg->ack_flag));
    pos++;

    resMsg->packet_len = msg->payload[pos++];
    assert(resMsg->packet_len < MAX_PACKET_DATA_LEN);

    for (uint8_t i=0; i<resMsg->packet_len; ++i) {
        resMsg->packet_data[i] = msg->payload[pos++];
    }

    seatrac_bytes_to_bool(&(msg->payload[pos]), &(resMsg->local_flag));
}

/**
 * @brief Decode a CID_DAT_ERROR message into a struct
 */
static inline void seatrac_DAT_ERROR_message_decode(seatrac_message_t* msg,
                                                     msg_response_status_t* resMsg)
{
    seatrac_protocol_response_status_message_decode(msg, resMsg);
}

/**
 * @brief Pack CID_DAT_QUEUE_SET parameters into a CID_DAT_QUEUE_SET message
 */
static inline void seatrac_DAT_QUEUE_SET_pack(seatrac_message_t* msg,
                                              uint8_t dest_id,
                                              uint8_t packet_len,
                                              uint8_t* packet_data)
{
    msg->cid = CID_DAT_QUEUE_SET;

    msg->payload[0] = dest_id;
    msg->payload[1] = packet_len;
    msg->len = 2;

    for (uint8_t i=0; i<packet_len; ++i) {
        msg->payload[msg->len++] = packet_data[i];
    }
}

/**
 * @brief Decode a CID_DAT_QUEUE_SET response message into a struct
 */
static inline void seatrac_DAT_QUEUE_SET_response_message_decode(seatrac_message_t* msg,
                                                                 msg_DAT_QUEUE_SET_response_t* resMsg)
{
    resMsg->status = msg->payload[1];
    resMsg->bid = msg->payload[2];
    resMsg->packet_len = msg->payload[3];
}

/**
 * @brief Pack CID_DAT_QUEUE_CLR parameters into a CID_DAT_QUEUE_CLR message
 */
static inline void seatrac_DAT_QUEUE_CLR_pack(seatrac_message_t* msg,
                                              uint8_t dest_id)
{
    msg->cid = CID_DAT_QUEUE_CLR;

    msg->payload[0] = dest_id;
    msg->len = 1;
}

/**
 * @brief Decode a response message into a struct
 */
static inline void seatrac_DAT_QUEUE_CLR_response_message_decode(seatrac_message_t* msg,
                                                                 msg_response_status_t* resMsg)
{
    seatrac_protocol_response_status_message_decode(msg, resMsg);
}

/**
 * @brief Pack CID_DAT_QUEUE_STATUS parameters into a CID_DAT_QUEUE_STATUS message
 */
static inline void seatrac_DAT_QUEUE_STATUS_pack(seatrac_message_t* msg)
{
    msg->cid = CID_DAT_QUEUE_STATUS;
    msg->len = 0;
}

/**
 * @brief Decode a CID_DAT_QUEUE_STATUS response message into a struct
 */
static inline void seatrac_DAT_QUEUE_STATUS_response_message_decode(seatrac_message_t* msg,
                                                                    msg_DAT_QUEUE_STATUS_response_t* resMsg)
{
    memcpy(resMsg->packet_len, &(msg->payload[1]), 16);
}



// Functions to display information of a struct.
// void disp_msg_dat_receive(msg_dat_receive_t *resMsg)
// {
//     disp_ACOFIX_t(&(resMsg->acofix));

//     // qDebug() << ", ack_flag: " << ((int)resMsg->ack_flag) << ", packet_len: " << ((int)resMsg->packet_len);

//     for (uint8_t i=0; i<resMsg->packet_len; ++i) {
//         // qDebug() << ", packet_data: " << (int)resMsg->packet_data[i];
//     }

//     // qDebug() << ", local_flag: " << (int)resMsg->local_flag << ".\n";
// }

// void disp_msg_dat_queue_set_response(msg_DAT_QUEUE_SET_response_t *resMsg)
// {
//     // qDebug() << "status: " << (int)resMsg->status <<
//                 // ", bid: " << (int)resMsg->bid <<
//                 // ", packet_len: " << (int)resMsg->packet_len;
// }
