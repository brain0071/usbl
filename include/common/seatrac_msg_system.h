#include "common.h"


// Structs Definitions
SEATRACPACKED(
typedef struct msg_sys_alive_response_t {
            uint32_t seconds;
}) msg_sys_alive_response_t;


static inline void seatrac_SYS_ALIVE_pack(seatrac_message_t* msg)
{
    msg->cid = CID_SYS_ALIVE;
    msg->len = 0;
}

/**
 * @brief Decode a response message into a struct
 */
static inline void seatrac_SYS_ALIVE_response_message_decode(seatrac_message_t* msg,
                                                             msg_sys_alive_response_t* resMsg)
{
    assert(msg->len == 5);

    seatrac_bytes_to_uint32_t(&(msg->payload[1]), &(resMsg->seconds));
}
