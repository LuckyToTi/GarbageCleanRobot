#include "motor_test_thread.h"
#include "hardware/yf80s.h"
#include "hardware/adc.h"
#include "hardware/timer.h"


#define LOG_TAG "motor_t"
//#define LOG_LVL LOG_LVL_INFO
#include <ulog.h>


static void PD12_PWM_Wrapper(uint32_t pulse);

YF80S_Config motor_conf = {
       .io0_port = GPIOE,
       .io0_pin = GPIO_PIN_5,
       .io1_port = GPIOE,
       .io1_pin = GPIO_PIN_6,
       .pwm_set = PD12_PWM_Wrapper,
       .pwm_period = TIM_ARR,
       .adc_get = Get_ADC_FilterValue_Safe,
       .adc_channel = ADC_CHANNEL_6
   };


rt_thread_t motor_test_tid = RT_NULL;
inline rt_thread_t* get_motor_test_tid()
{
    return &motor_test_tid;
}

void MotorTestThreadEntry(void* parameter)
{
    if(yf80s_init(&motor_conf) != HAL_OK) {
        LOG_E("YF80S init failed.");
    }
    float current;
    rt_thread_mdelay(1000);
    for( ; ; ) {
        static int duty_cycle = 0;
        if(duty_cycle > 100)
            duty_cycle = 0;
        yf80s_set_speed(duty_cycle);
        yf80s_set_state(YF80S_CW);
        duty_cycle += 1;
        current = yf80s_read_current();
        LOG_D("current = %.2fA", current);
        rt_thread_mdelay(50);
    }
}

static void PD12_PWM_Wrapper(uint32_t pulse) {
    PD12_PWM_OUT_US(pulse);
}
