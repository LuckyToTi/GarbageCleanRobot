#include "yf80s.h"

static YF80S_Config motor_config;

HAL_StatusTypeDef yf80s_init(YF80S_Config *config)
{
    if (config == NULL || config->pwm_set == NULL || config->adc_get == NULL)
        return HAL_ERROR;

    motor_config = *config;

    __HAL_RCC_GPIOE_CLK_ENABLE();

    GPIO_InitTypeDef GPIO_InitStruct = { 0 };

    GPIO_InitStruct.Pin = motor_config.io0_pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(motor_config.io0_port, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = motor_config.io1_pin;
    HAL_GPIO_Init(motor_config.io1_port, &GPIO_InitStruct);

    HAL_GPIO_WritePin(motor_config.io0_port, motor_config.io0_pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(motor_config.io1_port, motor_config.io1_pin, GPIO_PIN_RESET);

    return HAL_OK;
}

HAL_StatusTypeDef yf80s_set_state(MotorState state)
{
    switch (state)
    {
    case YF80S_BRAKE:
        HAL_GPIO_WritePin(motor_config.io0_port, motor_config.io0_pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(motor_config.io1_port, motor_config.io1_pin, GPIO_PIN_RESET);
        break;
    case YF80S_COAST:
        HAL_GPIO_WritePin(motor_config.io0_port, motor_config.io0_pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(motor_config.io1_port, motor_config.io1_pin, GPIO_PIN_SET);
        break;
    case YF80S_CW:
        HAL_GPIO_WritePin(motor_config.io0_port, motor_config.io0_pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(motor_config.io1_port, motor_config.io1_pin, GPIO_PIN_RESET);
        break;
    case YF80S_CCW:
        HAL_GPIO_WritePin(motor_config.io0_port, motor_config.io0_pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(motor_config.io1_port, motor_config.io1_pin, GPIO_PIN_SET);
        break;
    default:
        return HAL_ERROR;
    }
    return HAL_OK;
}

HAL_StatusTypeDef yf80s_set_speed(uint8_t duty)
{
    if (duty > 100)
        duty = 100;
    else if (duty < 0)
        duty = 0;
    uint32_t pulse = (motor_config.pwm_period * duty) / 100;
    motor_config.pwm_set(pulse);

    return HAL_OK;
}

float yf80s_read_current(void)
{
    uint16_t adc_value = motor_config.adc_get(motor_config.adc_channel);
    float voltage = (adc_value * 3.3f) / 4096.0f;
    float current = (voltage / 3.48f) * 10.0f;

    return (current > 10.0f) ? 10.0f : current;
}
