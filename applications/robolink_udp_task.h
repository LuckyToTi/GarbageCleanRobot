#ifndef __ROBOLINK_UDP_TASK_
#define __ROBOLINK_UDP_TASK_

#include <rtthread.h>
#include "userlib/robolink.h"

rt_thread_t* get_robolink_udp_tid();
void RobolinkUdpThreadEntry(void* parameter);

const struct RobolinkGetMsg* get_robolink_udp_get_msg();
void RobolinkUdpCreateInitSendNewMsgToMb(uint8_t* data_ptr, uint8_t data_len,
        uint8_t sys_id, uint8_t dev_id, uint8_t data_id);
int print_rl_udp(uint8_t l_vl, const char *format, ...);

#endif  // __ROBOLINK_UDP_TASK_
