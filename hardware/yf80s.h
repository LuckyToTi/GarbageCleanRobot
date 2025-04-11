#ifndef HARDWARE_YF80S_H_
#define HARDWARE_YF80S_H_

#include <rtthread.h>
#include <stm32f4xx_hal.h>

typedef void (*PWM_Set_Func)(uint32_t pulse);
typedef uint16_t (*ADC_Get_Func)(uint32_t channel);

typedef enum {
    YF80S_BRAKE,    // 刹车停止
    YF80S_COAST,    // 滑行停止
    YF80S_CW,       // 正转
    YF80S_CCW       // 反转
} MotorState;

typedef struct {
    GPIO_TypeDef* io0_port;
    uint16_t io0_pin;
    GPIO_TypeDef* io1_port;
    uint16_t io1_pin;

    PWM_Set_Func pwm_set;
    uint32_t pwm_period;

    ADC_Get_Func adc_get;
    uint8_t adc_channel;
} YF80S_Config;

HAL_StatusTypeDef yf80s_init(YF80S_Config *config);
HAL_StatusTypeDef yf80s_set_state(MotorState state);
HAL_StatusTypeDef yf80s_set_speed(uint8_t duty);
float yf80s_read_current(void);

#endif /* HARDWARE_YF80S_H_ */
