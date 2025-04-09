#include "led_extend.h"
#include "stm32f4xx.h"
#include "stm32f4xx_ll_gpio.h"

/*/ 外部指示灯
 *	PC6 - Green
 *  PC7 - Red
 **/

void LedExtendRgbGpioInit()
{
    __HAL_RCC_GPIOC_CLK_ENABLE();
    GPIO_InitTypeDef GPIO_InitStructure = {};
    GPIO_InitStructure.Pin = GPIO_PIN_6 | GPIO_PIN_7;
    GPIO_InitStructure.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStructure.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStructure.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStructure);
    LedExtendRgbWrite('r', 0u);
    LedExtendRgbWrite('g', 0u);
}

/**
 * Control RGB LED
 * @param led_color 'r' is red, 'g' is green, 'b' is blue
 * @param bit 0 is turn off, 1 is turn on
 */
void LedExtendRgbWrite(const char led_color, const unsigned char bit)
{
    switch (led_color)
    {
    case 'r':
        if(bit) LL_GPIO_ResetOutputPin(GPIOC, LL_GPIO_PIN_6);
        else LL_GPIO_SetOutputPin(GPIOC, LL_GPIO_PIN_6);
        return;
    case 'g':
        if(bit) LL_GPIO_ResetOutputPin(GPIOC, LL_GPIO_PIN_7);
        else LL_GPIO_SetOutputPin(GPIOC, LL_GPIO_PIN_7);
        return;
    default:
        break;
    }
}

/**
 * Read RGB LED now status
 * @param led_color led_color 'r' is red, 'g' is green, 'b' is blue
 * @return bit 0 is turn off, 1 is turn on
 */
unsigned char LedExtendRgbRead(const char led_color)
{
    switch (led_color)
    {
    case 'r':
        return !LL_GPIO_IsOutputPinSet(GPIOC, LL_GPIO_PIN_6);
    case 'g':
        return !LL_GPIO_IsOutputPinSet(GPIOC, LL_GPIO_PIN_7);
    default:
        break;
    }
    return 0u;
}
