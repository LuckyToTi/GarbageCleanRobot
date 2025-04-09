#ifndef __ROBOT_MSG_SERVER_THREAD_H_
#define __ROBOT_MSG_SERVER_THREAD_H_

#include "stm32f4xx.h"
#include <rtthread.h>

rt_thread_t* get_robot_msg_server_tid();
void RobotMsgServerThreadEntry(void* parameter);

#endif  // __ROBOT_MSG_SERVER_THREAD_H_
