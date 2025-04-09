#ifndef HARDWARE_TIMER_H_
#define HARDWARE_TIMER_H_

#include "stm32f4xx.h"

#define TIM_ARR 20000

#define TIM1_PSC 167
#define TIM3_PSC 83
#define TIM4_PSC 83
#define TIM9_PSC 167

extern TIM_HandleTypeDef htim4;
extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim9;

#define PD12_PWM_OUT_US(t)  __HAL_TIM_SetCompare(&htim4,TIM_CHANNEL_1,t)
#define PD13_PWM_OUT_US(t)  __HAL_TIM_SetCompare(&htim4,TIM_CHANNEL_2,t)
#define PD14_PWM_OUT_US(t)  __HAL_TIM_SetCompare(&htim4,TIM_CHANNEL_3,t)
#define PD15_PWM_OUT_US(t)  __HAL_TIM_SetCompare(&htim4,TIM_CHANNEL_4,t)

#define PE9_PWM_OUT_US(t)   __HAL_TIM_SetCompare(&htim1,TIM_CHANNEL_1,t)
#define PE11_PWM_OUT_US(t)  __HAL_TIM_SetCompare(&htim1,TIM_CHANNEL_2,t)
#define PE13_PWM_OUT_US(t)  __HAL_TIM_SetCompare(&htim1,TIM_CHANNEL_3,t)
#define PE14_PWM_OUT_US(t)  __HAL_TIM_SetCompare(&htim1,TIM_CHANNEL_4,t)

#define PE5_PWM_OUT_US(t)   __HAL_TIM_SetCompare(&htim9,TIM_CHANNEL_1,t)
#define PE6_PWM_OUT_US(t)   __HAL_TIM_SetCompare(&htim9,TIM_CHANNEL_2,t)
#define PB0_PWM_OUT_US(t)   __HAL_TIM_SetCompare(&htim3,TIM_CHANNEL_3,t)
#define PB1_PWM_OUT_US(t)   __HAL_TIM_SetCompare(&htim3,TIM_CHANNEL_4,t)

void TimerInit();

#endif /* HARDWARE_TIMER_H_ */
