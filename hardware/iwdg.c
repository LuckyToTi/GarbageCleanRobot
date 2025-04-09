#include <hardware/iwdg.h>
#include "config.h"
#include <string.h>
#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_iwdg.h"
#include "rtthread.h"

IWDG_HandleTypeDef hiwdg;

void IWDG_Init(void) {
	hiwdg.Instance = IWDG;
	hiwdg.Init.Prescaler = IWDG_PRESCALER_256;
	hiwdg.Init.Reload = 610;
	if (HAL_IWDG_Init(&hiwdg) != HAL_OK) {

	}
}
