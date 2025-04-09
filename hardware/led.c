#include "led.h"
#include "stm32f4xx.h"
#include "stm32f4xx_ll_gpio.h"

void LedRgbGpioInit()
{
    __HAL_RCC_GPIOE_CLK_ENABLE();
    GPIO_InitTypeDef GPIO_InitStructure = {};
    GPIO_InitStructure.Pin = GPIO_PIN_2 | GPIO_PIN_3 | GPIO_PIN_4;
    GPIO_InitStructure.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStructure.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStructure.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(GPIOE, &GPIO_InitStructure);
    LedRgbWrite('r', 0u);
    LedRgbWrite('g', 0u);
    LedRgbWrite('b', 0u);
}

/**
 * Control RGB LED
 * @param led_color 'r' is red, 'g' is green, 'b' is blue
 * @param bit 0 is turn off, 1 is turn on
 */
void LedRgbWrite(const char led_color, const unsigned char bit)
{
    switch (led_color)
    {
    case 'r':
        if(bit) LL_GPIO_ResetOutputPin(GPIOE, LL_GPIO_PIN_3);
        else LL_GPIO_SetOutputPin(GPIOE, LL_GPIO_PIN_3);
        return;
    case 'g':
        if(bit) LL_GPIO_ResetOutputPin(GPIOE, LL_GPIO_PIN_2);
        else LL_GPIO_SetOutputPin(GPIOE, LL_GPIO_PIN_2);
        return;
    case 'b':
        if(bit) LL_GPIO_ResetOutputPin(GPIOE, LL_GPIO_PIN_4);
        else LL_GPIO_SetOutputPin(GPIOE, LL_GPIO_PIN_4);
        return;
    }
}

/**
 * Read RGB LED now status
 * @param led_color led_color 'r' is red, 'g' is green, 'b' is blue
 * @return bit 0 is turn off, 1 is turn on
 */
unsigned char LedRgbRead(const char led_color)
{
    switch (led_color)
    {
    case 'r':
        return !LL_GPIO_IsOutputPinSet(GPIOE, LL_GPIO_PIN_3);
    case 'g':
        return !LL_GPIO_IsOutputPinSet(GPIOE, LL_GPIO_PIN_2);
    case 'b':
        return !LL_GPIO_IsOutputPinSet(GPIOE, LL_GPIO_PIN_4);
    default:
        break;
    }
    return 0u;
}
