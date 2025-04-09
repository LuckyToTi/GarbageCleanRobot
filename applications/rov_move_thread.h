#ifndef __ROV_MOVE_THREAD_H__
#define __ROV_MOVE_THREAD_H__

#include "stm32f4xx.h"
#include "rtthread.h"
#include <control_config.h>

rt_thread_t* get_rov_move_tid();

void RovThreadEntry(void* parameter);

#endif
