#ifndef HARDWARE_I2C_WD_H_
#define HARDWARE_I2C_WD_H_

#include "stm32f4xx.h"
#include "stm32f4xx_hal.h"
#include "stm32f4xx_ll_gpio.h"
#include "drv_common.h"
#include "userlib/softi2c.h"

void Ms5837_Softi2c_Gpio_Init(void);
void Ms5837_Softi2c_Init(void);
const Softi2c_device_t* Get_Ms5837_I2c_Device(void);

#endif /* HARDWARE_I2C_WD_H_ */
