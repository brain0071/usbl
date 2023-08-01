#pragma once

// Visual Studio versions before 2010 don't have stdint.h, so we just error out.
#if (defined _MSC_VER) && (_MSC_VER < 1600)
#error "The C-SEATRAC implementation requires Visual Studio 2010 or greater"
#endif

#include <stdbool.h>
#include <stdint.h>

#ifdef SEATRAC_USE_CXX_NAMESPACE
namespace seatrac {
#endif

// Macro to define packed structures
#ifdef __GNUC__
  #define SEATRACPACKED( __Declaration__ ) __Declaration__ __attribute__((packed))
#else
  #define SEATRACPACKED( __Declaration__ ) __pragma( pack(push, 1) ) __Declaration__ __pragma( pack(pop) )
#endif


#ifndef SEATRAC_MAX_PAYLOAD_LEN
// it is possible to override this, but be careful!
#define SEATRAC_MAX_PAYLOAD_LEN 512 ///< Maximum payload length
#endif

#define SEATRAC_CORE_HEADER_LEN 2 ///< Length of core header (of the comm. layer)
#define SEATRAC_NUM_HEADER_BYTES (SEATRAC_CORE_HEADER_LEN + 1) ///< Length of all header bytes, including core and stx
#define SEATRAC_NUM_CHECKSUM_BYTES 4
#define SEATRAC_NUM_NON_PAYLOAD_BYTES (SEATRAC_NUM_HEADER_BYTES + SEATRAC_NUM_CHECKSUM_BYTES)

#define SEATRAC_SIGNATURE_BLOCK_LEN 2

#define SEATRAC_MAX_PACKET_LEN (SEATRAC_MAX_PAYLOAD_LEN + SEATRAC_NUM_NON_PAYLOAD_BYTES + SEATRAC_SIGNATURE_BLOCK_LEN) ///< Maximum packet length



SEATRACPACKED(
typedef struct __seatrac_message {
    uint16_t checksum;      ///< sent at end of packet
    uint8_t cid;            ///< command ID
    uint16_t len;            ///< Length of payload
    uint8_t payload[SEATRAC_MAX_PACKET_LEN];
}) seatrac_message_t;


typedef enum {
    SEATRAC_PARSE_STATE_UNINIT=0,
    SEATRAC_PARSE_STATE_IDLE,
    SEATRAC_PARSE_STATE_GOT_STX,
    SEATRAC_PARSE_STATE_GOT_INCOMPAT_CID,
    SEATRAC_PARSE_STATE_GOT_COMPAT_CID,
    SEATRAC_PARSE_STATE_GOT_INCOMPAT_PAYLOAD,
    SEATRAC_PARSE_STATE_GOT_INCOMPAT_END
} seatrac_parse_state_t; ///< The state machine for the comm parser


typedef enum {
    SEATRAC_FRAMING_INCOMPLETE=0,
    SEATRAC_FRAMING_OK=1,
    SEATRAC_FRAMING_BAD_CRC=2,
    SEATRAC_FRAMING_BAD_END=3
} seatrac_framing_t;


#define SEATRAC_STX_PC2Beacon 0x23          //'#'
#define SEATRAC_STX_Beacon2PC 0x24          //'$'
#define SEATRAC_END_1 0x0D                  //<CR>
#define SEATRAC_END_2 0x0A                  //<LF>

typedef struct __seatrac_status {
    uint8_t msg_received;
    seatrac_parse_state_t parse_state;  ///< Parsing state machine
    uint16_t packet_idx;                 ///< Index in current packet
} seatrac_status_t;



