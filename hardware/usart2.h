#ifndef HARDWARE_USART2_H_
#define HARDWARE_USART2_H_

#include "stm32f4xx.h"

void Usart2Init();
int Usart2HalSendDataIt(uint8_t* tx_buffer_address, uint16_t tx_buffer_len);
int Usart2HalReceiveWaitDataItStopIdle(uint8_t* buffer, uint16_t max_len);
int Usart2HalReceiveWaitTimeoutDataItStopIdle(uint8_t* buffer, uint16_t max_len, int timeout);

#endif /* HARDWARE_USART2_H_ */
