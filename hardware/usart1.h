#ifndef HARDWARE_USART1_H_
#define HARDWARE_USART1_H_

#include <stm32f4xx.h>

void Usart1Init();
int Uart1HalReceiveWaitDataItStopIdle(uint8_t* buffer, uint16_t max_len);

#endif /* HARDWARE_USART5_H_ */
