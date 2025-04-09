#ifndef APPLICATIONS_CAN1_THREAD_H_
#define APPLICATIONS_CAN1_THREAD_H_

#include "stm32f4xx.h"
#include <rtthread.h>

rt_thread_t* get_can1_tid();
void Can1ThreadEntry(void* parameter);

void Can1TxThreadEntry(void* parameter);
void Can1RxThreadEntry(void* parameter);

void Can1CreateInitSendNewMsgToMb(uint32_t id, uint32_t IDE, uint32_t RTR, uint32_t dlc, const uint8_t* data);

#endif /* APPLICATIONS_CAN1_THREAD_H_ */
