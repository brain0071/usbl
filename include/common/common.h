
#pragma once

#ifndef SEATRAC_COMMON_H
#define SEATRAC_COMMON_H

#ifndef SEATRAC_H
    #error Wrong include order: SEATRAC_COMMON.H MUST NOT BE DIRECTLY USED. Include seatrac.h from the same directory instead or set ALL AND EVERY defines from SEATRAC.H manually accordingly, including the #define SEATRAC_H call.
#endif


#ifdef __cplusplus
extern "C" {
#endif


//#define NDEBUG
#include <assert.h>
#include "../seatrac_helpers.h"
#include "ros/ros.h"

// ENUM DEFINITIONS

/** @brief Protocol status. */
#ifndef HAVE_ENUM_SEATRAC_PROTOCOL_STATUS_E
#define HAVE_ENUM_SEATRAC_PROTOCOL_STATUS_E
typedef enum SEATRAC_PROTOCOL_STATUS_E
{
    SEATRAC_PROTOCOL_STATUS_IDLE,
    SEATRAC_PROTOCOL_STATUS_WAITING_FOR_LOCAL_RESP,
    SEATRAC_PROTOCOL_STATUS_WAITING_FOR_REMOTE_RESP,
    SEATRAC_PROTOCOL_STATUS_GOT_REMOTE_RESP,
    SEATRAC_PROTOCOL_STATUS_DAT_SENDING
} SEATRAC_PROTOCOL_STATUS_E;
#endif


/** @brief Acoustic Message Type. */
#ifndef HAVE_ENUM_AMSGTYPE_E
#define HAVE_ENUM_AMSGTYPE_E
typedef enum AMSGTYPE_E
{
    MSG_OWAY = 0x00,
    MSG_OWAYU = 0x01,
    MSG_REQ = 0x02,
    MSG_RESP = 0x03,
    MSG_REQU = 0x04,
    MSG_RESPU = 0x05,
    MSG_REQX = 0x06,
    MSG_RESPX = 0x07,
    MSG_UNKNOWN = 0x08
} AMSGTYPE_E;
#endif


/** @brief Acoustic Payload Identifier. */
#ifndef HAVE_ENUM_APAYLOAD_E
#define HAVE_ENUM_APAYLOAD_E
typedef enum APAYLOAD_E
{
    PLOAD_PING = 0x00,
    PLOAD_ECHO = 0x01,
    PLOAD_NAV = 0x02,
    PLOAD_DAT = 0x03,
    PLOAD_DEX = 0x04
} APAYLOAD_E;
#endif


/** @brief Serial Port Baud Rate. */
//#ifndef HAVE_ENUM_BAUDRATE_E
//#define HAVE_ENUM_BAUDRATE_E
//typedef enum BAUDRATE_E
//{
//    BAUD_4800 = 0x07,
//    BAUD_9600 = 0x08,
//    BAUD_14400 = 0x09,
//    BAUD_19200 = 0x0A,
//    BAUD_38400 = 0x0B,
//    BAUD_57600 = 0x0C,
//    BAUD_115200 = 0x0D
//} BAUDRATE_E;
//#endif


/** @brief Beacon Identification Code. */
#ifndef HAVE_ENUM_BID_E
#define HAVE_ENUM_BID_E
typedef enum BID_E
{
    BEACON_ALL = 0x00,
    BEACON_ID_x = 0x01
} BID_E;
#endif


/** @brief Calibration Actions. */
#ifndef HAVE_ENUM_CAL_ACTION_E
#define HAVE_ENUM_CAL_ACTION_E
typedef enum CAL_ACTION_E
{
    CAL_ACC_DEFAULTS = 0x00,
    CAL_ACC_RESET = 0x01,
    CAL_ACC_CALC = 0x02,
    CAL_MAG_DEFAULTS = 0x03,
    CAL_MAG_RESET = 0x04,
    CAL_MAG_CALC = 0x05,
    CAL_PRES_OFFSET_RESET = 0x06,
    CAL_PRES_OFFSET_CALC = 0x07
} CAL_ACTION_E;
#endif


/** @brief Command Identification Codes. */
#ifndef HAVE_ENUM_CID_E
#define HAVE_ENUM_CID_E
typedef enum CID_E
{
    // System Message
    CID_SYS_ALIVE = 0x01,
    CID_SYS_INFO = 0x02,
    CID_SYS_REBOOT = 0x03,
    CID_SYS_ENGINEERING = 0x04,

    // Firmware Programming Messages
    CID_PROG_INIT = 0x0D,
    CID_PROG_BLOCK = 0x0E,
    CID_PROG_UPDATE = 0x0F,

    // Status Messages
    CID_STATUS = 0x10,
    CID_STATUS_CFG_GET = 0x11,
    CID_STATUS_CFG_SET = 0x12,
    CID_SETTINGS_GET = 0x15,
    CID_SETTINGS_SET = 0x16,
    CID_SETTINGS_LOAD = 0x17,
    CID_SETTINGS_SAVE = 0x18,
    CID_SETTINGS_RESET = 0x19,

    // Calibration Messages
    CID_CAL_ACTION = 0x20,
    CID_AHRS_CAL_GET = 0x21,
    CID_AHRS_CAL_SET = 0x22,

    // Acoustic Transceiver Messages
    CID_XCVR_ANALYSE = 0x30,
    CID_XCVR_TX_MSG = 0x31,
    CID_XCVR_RX_ERR = 0x32,
    CID_XCVR_RX_MSG = 0x33,
    CID_XCVR_RX_REQ = 0x34,
    CID_XCVR_RX_RESP = 0x35,
    CID_XCVR_RX_UNHANDLED = 0x37,
    CID_XCVR_USBL = 0x38,
    CID_XCVR_FIX = 0x39,
    CID_XCVR_STATUS = 0x3A,

    // PING Protocol Messages
    CID_PING_SEND = 0x40,
    CID_PING_REQ = 0x41,
    CID_PING_RESP = 0x42,
    CID_PING_ERROR = 0x43,

    // ECHO Protocol Messages
    CID_ECHO_SEND = 0x48,
    CID_ECHO_REQ = 0x49,
    CID_ECHO_RESP = 0x4A,
    CID_ECHO_ERROR = 0x4B,

    // NAV Protocol Messages
    CID_NAV_QUERY_SEND = 0x50,
    CID_NAV_QUERY_REQ = 0x51,
    CID_NAV_QUERY_RESP = 0x52,
    CID_NAV_ERROR = 0x53,
    CID_NAV_QUEUE_SET = 0x58,
    CID_NAV_QUEUE_CLR = 0x59,
    CID_NAV_QUEUE_STATUS = 0x5A,
    CID_NAV_STATUS_SEND = 0x5B,
    CID_NAV_STATUS_RECEIVE = 0x5C,

    // DAT Protocol Messages
    CID_DAT_SEND = 0x60,
    CID_DAT_RECEIVE = 0x61,
    CID_DAT_ERROR = 0x63,
    CID_DAT_QUEUE_SET = 0x64,
    CID_DAT_QUEUE_CLR = 0x65,
    CID_DAT_QUEUE_STATUS = 0x66
} CID_E;
#endif


/** @brief Command Status Codes. */
#ifndef HAVE_ENUM_CST_E
#define HAVE_ENUM_CST_E
typedef enum CST_E
{
    // General Status Codes
    CST_OK = 0x00,
    CST_FAIL = 0x01,
    CST_EEPROM_ERROR = 0x03,

    // Command Processor Status Codes
    CST_CMD_PARAM_MISSING = 0x04,
    CST_CMD_PARAM_INVALID = 0x05,

    // Firmware Programming Status Codes
    CST_PROG_FLASH_ERROR = 0x0A,
    CST_PROG_FIRMWARE_ERROR = 0x0B,
    CST_PROG_SECTION_ERROR = 0x0C,
    CST_PROG_LENGTH_ERROR = 0x0D,
    CST_PROG_DATA_ERROR = 0x0E,
    CST_PROG_CHECKSUM_ERROR = 0x0F,

    // Acoustic Transceiver Status Codes
    CST_XCVR_BUSY = 0x30,
    CST_XCVR_ID_REJECTED = 0x31,
    CST_XCVR_CSUM_ERROR = 0x32,
    CST_XCVR_LENGTH_ERROR = 0x33,
    CST_XCVR_RESP_TIMEOUT = 0x34,
    CST_XCVR_RESP_ERROR = 0x35,
    CST_XCVR_RESP_WRONG = 0x36,
    CST_XCVR_PLOAD_ERROR = 0x37,
    CST_XCVR_STATE_STOPPED = 0x3A,
    CST_XCVR_STATE_IDLE = 0x3B,
    CST_XCVR_STATE_TX = 0x3C,
    CST_XCVR_STATE_REQ = 0x3D,
    CST_XCVR_STATE_RX = 0x3E,
    CST_XCVR_STATE_RESP = 0x3F,

    // DEX Protocol Status Codes
    CST_DEX_SOCKET_ERROR = 0x70,
    CST_DEX_RX_SYNC = 0x71,
    CST_DEX_RX_DATA = 0x72,
    CST_DEX_RX_SEQ_ERROR = 0x73,
    CST_DEX_RX_MSG_ERROR = 0x74,
    CST_DEX_REQ_ERROR = 0x75,
    CST_DEX_RESP_TMO_ERROR = 0x76,
    CST_DEX_RESP_MSG_ERROR = 0x77,
    CST_DEX_RESP_REMOTE_ERROR = 0x78
} CST_E;
#endif


/** @brief Status Output Mode. */
#ifndef HAVE_ENUM_STATUSMODE_E
#define HAVE_ENUM_STATUSMODE_E
typedef enum STATUSMODE_E
{
    STATUS_MODE_MANUAL = 0x00,
    STATUS_MODE_1HZ = 0x01,
    STATUS_MODE_2HZ5 = 0x02,
    STATUS_MODE_5HZ = 0x03,
    STATUS_MODE_10HZ = 0x04,
    STATUS_MODE_25HZ = 0x05
} STATUSMODE_E;
#endif



// Common Struct Definitions
SEATRACPACKED(
typedef struct msg_response_status_t {
            uint8_t status;
            uint8_t bid;
}) msg_response_status_t;


/** @brief Flag positions of ACOFIX_T.flags . */
#ifndef HAVE_ENUM_ACOFIX_FLAGS_E
#define HAVE_ENUM_ACOFIX_FLAGS_E
typedef enum ACOFIX_FLAGS_E
{
    ACOFIX_FLAGS_HAVE_RANGE_FIELDS = 0x01,
    ACOFIX_FLAGS_HAVE_USBL_FIELDS = 0x02,
    ACOFIX_FLAGS_HAVE_POSITION_FIELDS = 0x04,
    ACOFIX_FLAGS_POSITION_ENHANCED = 0x08,
    ACOFIX_FLAGS_POSITION_FLT_ERROR = 0x10
} ACOFIX_FLAGS_E;
#endif

SEATRACPACKED(
typedef struct ACOFIX_t {
            uint8_t dest_bid;
            uint8_t src_bid;
            uint8_t flags;
            uint8_t msg_type;

            int16_t attitude_yaw;
            int16_t attitude_pitch;
            int16_t attitude_roll;

            uint16_t depth_local;
            uint16_t vos;
            int16_t rssi;

            // Range Fields
            uint32_t range_count;
            int32_t range_time;
            uint16_t range_dist;

            // USBL Fields
            uint8_t usbl_channels;
            int16_t usbl_rssi[4];
            int16_t usbl_azimuth;
            int16_t usbl_elevation;
            int16_t usbl_fit_error;

            // Position Fields
            int16_t position_easting;
            int16_t position_northing;
            int16_t position_depth;
}) ACOFIX_t;



// Functions to decode common struct from a received message.

/**
 * @brief Decode a response_status_message message into a struct
 */
static inline void seatrac_protocol_response_status_message_decode(seatrac_message_t* msg,
                                                                   msg_response_status_t* resMsg)
{
    assert(msg->len == 3);
    resMsg->status = msg->payload[1];
    resMsg->bid = msg->payload[2];
}

/**
 * @brief Decode a ACOFIX_T message into a struct
 */
static inline void seatrac_ACOFIX_T_decode(uint8_t* buf, uint16_t& pos, ACOFIX_t* acofix)
{
    acofix->dest_bid = buf[pos++];
    acofix->src_bid = buf[pos++];
    acofix->flags = buf[pos++];
    acofix->msg_type = buf[pos++];

    seatrac_bytes_to_int16_t(&(buf[pos]), &(acofix->attitude_yaw));
    pos += 2;
    seatrac_bytes_to_int16_t(&(buf[pos]), &(acofix->attitude_pitch));
    pos += 2;
    seatrac_bytes_to_int16_t(&(buf[pos]), &(acofix->attitude_roll));
    pos += 2;

    seatrac_bytes_to_uint16_t(&(buf[pos]), &(acofix->depth_local));
    pos += 2;
    seatrac_bytes_to_uint16_t(&(buf[pos]), &(acofix->vos));
    pos += 2;
    seatrac_bytes_to_int16_t(&(buf[pos]), &(acofix->rssi));
    pos += 2;

    if (acofix->flags & ACOFIX_FLAGS_HAVE_RANGE_FIELDS) {
        seatrac_bytes_to_uint32_t(&(buf[pos]), &(acofix->range_count));
        pos += 4;
        seatrac_bytes_to_int32_t(&(buf[pos]), &(acofix->range_time));
        pos += 4;
        seatrac_bytes_to_uint16_t(&(buf[pos]), &(acofix->range_dist));
        pos += 2;
    }

    if (acofix->flags & ACOFIX_FLAGS_HAVE_USBL_FIELDS) {
        acofix->usbl_channels = buf[pos++];

        for (size_t i=0; i<acofix->usbl_channels; ++i) {
            if (i >= 4) {
                pos += 2;
            }
            else {
                seatrac_bytes_to_int16_t(&(buf[pos]), &(acofix->usbl_rssi[i]));
                pos += 2;
            }
        }

        seatrac_bytes_to_int16_t(&(buf[pos]), &(acofix->usbl_azimuth));
        pos += 2;
        seatrac_bytes_to_int16_t(&(buf[pos]), &(acofix->usbl_elevation));
        pos += 2;
        seatrac_bytes_to_int16_t(&(buf[pos]), &(acofix->usbl_fit_error));
        pos += 2;
    }

    if (acofix->flags & ACOFIX_FLAGS_HAVE_POSITION_FIELDS) {
        seatrac_bytes_to_int16_t(&(buf[pos]), &(acofix->position_easting));
        pos += 2;
        seatrac_bytes_to_int16_t(&(buf[pos]), &(acofix->position_northing));
        pos += 2;
        seatrac_bytes_to_int16_t(&(buf[pos]), &(acofix->position_depth));
        pos += 2;
    }

}



// Functions to display information of a struct.
// void disp_msg_response_status(msg_response_status_t *resMsg)
// {
    // qDebug() << " status: " << (int)resMsg->status <<
                // ", beacon id:" << (int)resMsg->bid << ". \n";
// }

// void disp_ACOFIX_t(ACOFIX_t *resMsg)
// {
//     // qDebug() << " dest_bid: " << (int)resMsg->dest_bid <<
//     //             ", src_bid:" << (int)resMsg->src_bid <<
//     //             ", flags:" << (int)resMsg->flags <<
//     //             ", msg_type:" << (int)resMsg->msg_type <<
//     //             ", attitude_yaw:" << (int)resMsg->attitude_yaw <<
//     //             ", attitude_pitch:" << (int)resMsg->attitude_pitch <<
//     //             ", attitude_roll:" << (int)resMsg->attitude_roll <<
//     //             ", depth_local:" << (int)resMsg->depth_local <<
//     //             ", vos:" << (int)resMsg->vos <<
//     //             ", rssi:" << (int)resMsg->src_bid;

//     if (resMsg->flags & ACOFIX_FLAGS_HAVE_RANGE_FIELDS) {
//         // qDebug() << ", range_count:" << (int)resMsg->range_count <<
//         //             ", range_time:" << (int)resMsg->range_time <<
//         //             ", range_dist:" << (int)resMsg->range_dist;
//     }

//     if (resMsg->flags & ACOFIX_FLAGS_HAVE_USBL_FIELDS) {
//         // qDebug() << ", usbl_channels:" << (int)resMsg->usbl_channels;
//         for (uint8_t i=0; i<resMsg->usbl_channels; ++i) {
//             // qDebug() << ", usbl_rssi:" << (int)resMsg->usbl_rssi[i];
//         }
//         // qDebug() << ", usbl_azimuth:" << (int)resMsg->usbl_azimuth <<
//                     // ", usbl_elevation:" << (int)resMsg->usbl_elevation <<
//                     // ", usbl_fit_error:" << (int)resMsg->usbl_fit_error;
//     }

//     if (resMsg->flags & ACOFIX_FLAGS_HAVE_POSITION_FIELDS) {
//         // qDebug() << ", position_easting:" << (int)resMsg->position_easting <<
//                     // ", position_northing:" << (int)resMsg->position_northing <<
//                     // ", position_depth:" << (int)resMsg->position_depth;
//     }


//     // qDebug() << ". ";
// }


#define MAX_PACKET_DATA_LEN 30

// MESSAGE DEFINITIONS
#include "./seatrac_msg_system.h"
#include "./seatrac_ping.h"
#include "./seatrac_nav.h"
#include "./seatrac_dat.h"

// base include


#ifdef __cplusplus
}
#endif // __cplusplus
#endif // SEATRAC_COMMON_H