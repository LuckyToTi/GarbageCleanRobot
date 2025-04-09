#ifndef HARDWARE_USART5_H_
#define HARDWARE_USART5_H_

#include <rthw.h>
#include <stm32f4xx.h>

void Uart5HalInterfaceInit();
int Uart5HalReceiveWaitDataItStopIdle(uint8_t* buffer, uint16_t max_len);

#endif /* HARDWARE_USART5_H_ */
