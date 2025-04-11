#ifndef HARDWARE_ADC_H_
#define HARDWARE_ADC_H_

#include <rtthread.h>
#include "stm32f4xx_hal_conf.h"

void MX_ADC1_Init(void);
uint16_t Get_ADC_FilterValue_Safe(uint32_t channel);

#endif /* HARDWARE_ADC_H_ */
