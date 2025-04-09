#ifndef HARDWARE_ADC_H_
#define HARDWARE_ADC_H_

#include "stm32f4xx.h"
#include "stm32f4xx_ll_gpio.h"
#include "stm32f4xx_hal_adc.h"
#include "stm32f4xx_hal_rcc_ex.h"

extern ADC_HandleTypeDef hadc1;

void MX_ADC1_Init(void);

uint16_t get_adc();

#endif /* HARDWARE_ADC_H_ */
