#ifndef APPLICATIONS_ROBOLINK_USART_THREAD_H_
#define APPLICATIONS_ROBOLINK_USART_THREAD_H_

#include "stm32f4xx.h"
#include <rtthread.h>
#include "userlib/sbus_rc.h"

const struct RobolinkGetMsg* get_robolink_usart_get_msg();
const struct SbusRc* get_sbus_rc_msg();
rt_thread_t* get_robolink_usart_tid();
void ControllerUsartThreadEntry(void* parameter);
void ControllerSbusRcThreadEntry(void* parameter);

#endif /* APPLICATIONS_ROBOLINK_USART_THREAD_H_ */
