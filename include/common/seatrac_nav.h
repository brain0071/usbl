
#include "../seatrac_types.h"
#include "common.h"


// ENUM DEFINITIONS
/** @brief Flag positions of NAV_QUERY_T . */
#ifndef HAVE_ENUM_NAV_QUERY_FLAGS_E
#define HAVE_ENUM_NAV_QUERY_FLAGS_E
typedef enum NAV_QUERY_FLAGS_E
{
    NAV_QUERY_FLAGS_QRY_DATA = 0x80,
    NAV_QUERY_FLAGS_QRY_ATTITUDE = 0x08,
    NAV_QUERY_FLAGS_QRY_TEMP = 0x04,
    NAV_QUERY_FLAGS_QRY_SUPPLY = 0x02,
    NAV_QUERY_FLAGS_QRY_DEPTH = 0x01,
} NAV_QUERY_FLAGS_E;
#endif



// Structs Definitions
SEATRACPACKED(
typedef struct msg_nav_query_req_t {
            ACOFIX_t acofix;
            uint8_t query_flags;

            uint8_t packet_len;
            uint8_t packet_data[MAX_PACKET_DATA_LEN];

            bool local_flag;
}) msg_nav_query_req_t;

SEATRACPACKED(
typedef struct msg_nav_query_resp_t {
            ACOFIX_t acofix;
            uint8_t query_flags;

            // Depth Field
            int32_t remote_depth;

            // Supply Field
            uint16_t remote_supply;

            // Temperature Field
            int16_t remote_temp;

            // Attitude Field
            int16_t remote_yaw;
            int16_t remote_pitch;
            int16_t remote_roll;

            // Data Field
            uint8_t packet_len;
            uint8_t packet_data[MAX_PACKET_DATA_LEN];

            bool local_flag;
}) msg_nav_query_resp_t;

SEATRACPACKED(
typedef struct msg_NAV_QUEUE_SET_response_t {
            uint8_t status;
            uint8_t bid;
            uint8_t packet_len;
}) msg_NAV_QUEUE_SET_response_t;

SEATRACPACKED(
typedef struct msg_NAV_QUEUE_STATUS_response_t {
            uint8_t packet_len[16];
}) msg_NAV_QUEUE_STATUS_response_t;

SEATRACPACKED(
typedef struct msg_nav_status_receive_t {
            ACOFIX_t acofix;
            uint8_t bid;

            uint8_t packet_len;
            uint8_t packet_data[MAX_PACKET_DATA_LEN];

            bool local_flag;
}) msg_nav_status_receive_t;



// Functions to pack or decode a message.
/**
 * @brief Pack CID_NAV_QUERY_SEND parameters into a CID_NAV_QUERY_SEND message
 */
static inline void seatrac_NAV_QUERY_SEND_pack(seatrac_message_t* msg,
                                               uint8_t dest_id,
                                               uint8_t query_flags,
                                               uint8_t packet_len,
                                               uint8_t* packet_data)
{
    msg->cid = CID_NAV_QUERY_SEND;

    msg->payload[0] = dest_id;
    msg->payload[1] = query_flags;
    msg->payload[2] = packet_len;
    msg->len = 3;

    for (uint8_t i=0; i<packet_len; ++i) {
        msg->payload[msg->len++] = packet_data[i];
    }
}

/**
 * @brief Decode a response message into a struct
 */
static inline void seatrac_NAV_QUERY_SEND_response_message_decode(seatrac_message_t* msg,
                                                                  msg_response_status_t* resMsg)
{
    seatrac_protocol_response_status_message_decode(msg, resMsg);
}

/**
 * @brief Decode a CID_NAV_QUERY_REQ message into a struct
 */
static inline void seatrac_NAV_QUERY_REQ_message_decode(seatrac_message_t* msg,
                                                        msg_nav_query_req_t* resMsg)
{
    uint16_t pos = 1;
    seatrac_ACOFIX_T_decode(msg->payload, pos, &(resMsg->acofix));
    resMsg->query_flags = msg->payload[pos++];

    resMsg->packet_len = msg->payload[pos++];
    assert(resMsg->packet_len < MAX_PACKET_DATA_LEN);

    for (uint8_t i=0; i<resMsg->packet_len; ++i) {
        resMsg->packet_data[i] = msg->payload[pos++];
    }

    seatrac_bytes_to_bool(&(msg->payload[pos]), &(resMsg->local_flag));
}

/**
 * @brief Decode a CID_NAV_QUERY_RESP message into a struct
 */
static inline void seatrac_NAV_QUERY_RESP_message_decode(seatrac_message_t* msg,
                                                         msg_nav_query_resp_t* resMsg)
{
    uint16_t pos = 1;
    seatrac_ACOFIX_T_decode(msg->payload, pos, &(resMsg->acofix));
    resMsg->query_flags = msg->payload[pos++];

    if (resMsg->query_flags & NAV_QUERY_FLAGS_QRY_DEPTH) {
        seatrac_bytes_to_int32_t(&(msg->payload[pos]), &(resMsg->remote_depth));
        pos += 4;
    }

    if (resMsg->query_flags & NAV_QUERY_FLAGS_QRY_SUPPLY) {
        seatrac_bytes_to_uint16_t(&(msg->payload[pos]), &(resMsg->remote_supply));
        pos += 2;
    }

    if (resMsg->query_flags & NAV_QUERY_FLAGS_QRY_TEMP) {
        seatrac_bytes_to_int16_t(&(msg->payload[pos]), &(resMsg->remote_temp));
        pos += 2;
    }

    if (resMsg->query_flags & NAV_QUERY_FLAGS_QRY_ATTITUDE) {
        seatrac_bytes_to_int16_t(&(msg->payload[pos]), &(resMsg->remote_yaw));
        pos += 2;
        seatrac_bytes_to_int16_t(&(msg->payload[pos]), &(resMsg->remote_pitch));
        pos += 2;
        seatrac_bytes_to_int16_t(&(msg->payload[pos]), &(resMsg->remote_roll));
        pos += 2;
    }

    if (resMsg->query_flags & NAV_QUERY_FLAGS_QRY_DATA) {
        resMsg->packet_len = msg->payload[pos++];
        assert(resMsg->packet_len < MAX_PACKET_DATA_LEN);

        for (uint8_t i=0; i<resMsg->packet_len; ++i) {
            resMsg->packet_data[i] = msg->payload[pos++];
        }
    }

    seatrac_bytes_to_bool(&(msg->payload[pos]), &(resMsg->local_flag));
}

/**
 * @brief Decode a CID_NAV_ERROR message into a struct
 */
static inline void seatrac_NAV_ERROR_message_decode(seatrac_message_t* msg,
                                                    msg_response_status_t* resMsg)
{
    seatrac_protocol_response_status_message_decode(msg, resMsg);
}

/**
 * @brief Pack CID_NAV_QUEUE_SET parameters into a CID_NAV_QUEUE_SET message
 */
static inline void seatrac_NAV_QUEUE_SET_pack(seatrac_message_t* msg,
                                              uint8_t dest_id,
                                              uint8_t packet_len,
                                              uint8_t* packet_data)
{
    msg->cid = CID_NAV_QUEUE_SET;

    msg->payload[0] = dest_id;
    msg->payload[1] = packet_len;
    msg->len = 2;

    for (uint8_t i=0; i<packet_len; ++i) {
        msg->payload[msg->len++] = packet_data[i];
    }
}

/**
 * @brief Decode a CID_NAV_QUEUE_SET response message into a struct
 */
static inline void seatrac_NAV_QUEUE_SET_response_message_decode(seatrac_message_t* msg,
                                                                 msg_NAV_QUEUE_SET_response_t* resMsg)
{
    resMsg->status = msg->payload[1];
    resMsg->bid = msg->payload[2];
    resMsg->packet_len = msg->payload[3];
}

/**
 * @brief Pack CID_NAV_QUEUE_CLR parameters into a CID_NAV_QUEUE_CLR message
 */
static inline void seatrac_NAV_QUEUE_CLR_pack(seatrac_message_t* msg,
                                              uint8_t dest_id)
{
    msg->cid = CID_NAV_QUEUE_CLR;

    msg->payload[0] = dest_id;
    msg->len = 1;
}

/**
 * @brief Decode a response message into a struct
 */
static inline void seatrac_NAV_QUEUE_CLR_response_message_decode(seatrac_message_t* msg,
                                                                 msg_response_status_t* resMsg)
{
    seatrac_protocol_response_status_message_decode(msg, resMsg);
}

/**
 * @brief Pack CID_NAV_QUEUE_STATUS parameters into a CID_NAV_QUEUE_STATUS message
 */
static inline void seatrac_NAV_QUEUE_STATUS_pack(seatrac_message_t* msg)
{
    msg->cid = CID_NAV_QUEUE_STATUS;
    msg->len = 0;
}

/**
 * @brief Decode a CID_NAV_QUEUE_STATUS response message into a struct
 */
static inline void seatrac_NAV_QUEUE_STATUS_response_message_decode(seatrac_message_t* msg,
                                                                    msg_NAV_QUEUE_STATUS_response_t* resMsg)
{
    memcpy(resMsg->packet_len, &(msg->payload[1]), 16);
}

/**
 * @brief Pack CID_NAV_STATUS_SEND parameters into a CID_NAV_STATUS_SEND message
 */
static inline void seatrac_NAV_STATUS_SEND_pack(seatrac_message_t* msg,
                                                uint8_t dest_id,
                                                uint8_t packet_len,
                                                uint8_t* packet_data)
{
    msg->cid = CID_NAV_STATUS_SEND;

    msg->payload[0] = dest_id;
    msg->payload[1] = packet_len;
    msg->len = 2;

    for (uint8_t i=0; i<packet_len; ++i) {
        msg->payload[msg->len++] = packet_data[i];
    }
}

/**
 * @brief Decode a response message into a struct
 */
static inline void seatrac_NAV_STATUS_SEND_response_message_decode(seatrac_message_t* msg,
                                                                   msg_response_status_t* resMsg)
{
    seatrac_protocol_response_status_message_decode(msg, resMsg);
}

/**
 * @brief Decode a CID_NAV_STATUS_RECEIVE message into a struct
 */
static inline void seatrac_NAV_STATUS_RECEIVE_message_decode(seatrac_message_t* msg,
                                                             msg_nav_status_receive_t* resMsg)
{
    uint16_t pos = 1;
    seatrac_ACOFIX_T_decode(msg->payload, pos, &(resMsg->acofix));

    resMsg->bid = msg->payload[pos++];

    resMsg->packet_len = msg->payload[pos++];
    for (uint8_t i=0; i<resMsg->packet_len; ++i) {
        resMsg->packet_data[i] = msg->payload[pos++];
    }

    seatrac_bytes_to_bool(&(msg->payload[pos]), &(resMsg->local_flag));
}




// Functions to display information of a struct.
// void disp_msg_nav_query_req(msg_nav_query_req_t *resMsg)
// {
//     disp_ACOFIX_t(&(resMsg->acofix));

//     // qDebug() << ", query_flags: " << (int)resMsg->query_flags <<
//                 // ", packet_len: " << (int)resMsg->packet_len;

//     for (uint8_t i=0; i<resMsg->packet_len; ++i) {
//         // qDebug() << ", packet_data: " << (int)resMsg->packet_data[i];
//     }

//     // qDebug() << ", local_flag: " << (int)resMsg->local_flag << ".\n";
// }

// void disp_msg_nav_query_resp(msg_nav_query_resp_t *resMsg)
// {
//     disp_ACOFIX_t(&(resMsg->acofix));

//     // qDebug() << ", query_flags: " << (int)resMsg->query_flags;

//     if (resMsg->query_flags & NAV_QUERY_FLAGS_QRY_DEPTH) {
//         // qDebug() << ", remote_depth: " << (int)resMsg->remote_depth;
//     }

//     if (resMsg->query_flags & NAV_QUERY_FLAGS_QRY_SUPPLY) {
//         // qDebug() << ", remote_supply: " << (int)resMsg->remote_supply;
//     }

//     if (resMsg->query_flags & NAV_QUERY_FLAGS_QRY_TEMP) {
//         // qDebug() << ", remote_temp: " << (int)resMsg->remote_temp;
//     }

//     if (resMsg->query_flags & NAV_QUERY_FLAGS_QRY_ATTITUDE) {
//         // qDebug() << ", remote_yaw: " << (int)resMsg->remote_yaw <<
//                     // ", remote_pitch: " << (int)resMsg->remote_pitch <<
//                     // ", remote_roll: " << (int)resMsg->remote_roll;
//     }

//     if (resMsg->query_flags & NAV_QUERY_FLAGS_QRY_DATA) {
//         // qDebug() << ", packet_len: " << (int)resMsg->packet_len;

//         for (uint8_t i=0; i<resMsg->packet_len; ++i) {
//             // qDebug() << ", packet_data: " << (int)resMsg->packet_data[i];
//         }
//     }

//     // qDebug() << ", local_flag: " << (int)resMsg->local_flag << ".\n";
// }

// void disp_msg_nav_status_receive(msg_nav_status_receive_t *resMsg)
// {
//     disp_ACOFIX_t(&(resMsg->acofix));

//     // qDebug() << ", bid: " << (int)resMsg->bid <<
//                 // ", packet_len: " << (int)resMsg->packet_len;

//     for (uint8_t i=0; i<resMsg->packet_len; ++i) {
//         // qDebug() << ", packet_data: " << (int)resMsg->packet_data[i];
//     }

//     // qDebug() << ", local_flag: " << (int)resMsg->local_flag << ".\n";
// }