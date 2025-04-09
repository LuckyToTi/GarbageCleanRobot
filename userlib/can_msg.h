#ifndef __CAN_MSG_H_
#define __CAN_MSG_H_

#include <stm32f4xx.h>

struct CanMsg {
    uint32_t id;
    uint32_t dlc;
    uint32_t IDE;
    uint32_t RTR;
    uint8_t data[8];
};

struct CanMsg* can_create_init_new_msg(uint32_t id, uint32_t IDE, uint32_t RTR, uint32_t dlc, const uint8_t* data);
void can_free_unused_msg(struct CanMsg* msg);

#endif  // __CAN_MSG_H_

