/**
 * naodai:
 * get usbl status: include depth,temp,ahrs
 * **/

#include "../seatrac_types.h"
#include "common.h"

// send status command to beacon
static inline void seatrac_STATUS_pack(seatrac_message_t* msg,
                                        uint8_t dest_id,
                                        uint8_t msg_type)
                                        {
                                            msg->cid = CID_STATUS;
                                            msg->payload[0] = dest_id;
                                            msg->payload[1] = msg_type;
                                            msg->len = 2;
                                        }

