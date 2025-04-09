#include "can_msg.h"
#include "rtthread.h"

#define malloc_(size)  rt_malloc(size)
#define free_(ptr)     rt_free(ptr)

struct CanMsg* can_create_new_msg() {
    struct CanMsg* new_msg = NULL;
    new_msg = (struct CanMsg*) malloc_(sizeof(struct CanMsg));
    return new_msg;
}

int32_t can_init_msg(struct CanMsg* msg, uint32_t id, uint32_t IDE, uint32_t RTR, uint32_t dlc, const uint8_t* data) {
    if (msg == NULL) return -1;
    if (dlc == 0 || dlc > 8) return -2;
    if (data == NULL) return -3;
    msg->id = id;
    msg->IDE = IDE;
    msg->RTR = RTR;
    msg->dlc = dlc;
    rt_memcpy(msg->data, data, dlc);
    return 0;
}

struct CanMsg* can_create_init_new_msg(uint32_t id, uint32_t IDE, uint32_t RTR, uint32_t dlc, const uint8_t* data) {
    struct CanMsg* new_msg = NULL;
    new_msg = can_create_new_msg();
    if (new_msg == NULL) return NULL;
    if (can_init_msg(new_msg, id, IDE, RTR, dlc, data) != 0) {
        can_free_unused_msg(new_msg);
        return NULL;
    }
    return new_msg;
}

void can_free_unused_msg(struct CanMsg* msg) {
    if (msg == NULL) return;
    free_(msg);
    msg = NULL;
}
