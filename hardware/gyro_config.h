#ifndef __GYRO_CONFIG_H_
#define __GYRO_CONFIG_H_

#include "stm32f4xx.h"
#include <rtthread.h>


enum Gyro_Config_Setting {
        AXIS,  	// 陀螺仪
		ORI		// 安装方向
};

#define GYRO_DEV_ADDR (0X50 << 1)

void gyroSet(uint8_t MemAddress, uint16_t data, uint8_t len);
int gyroAxis(int axis);
int gyroOrient(int orient);
void gyroSetInit(void);
void gyroResetCalibration(void);
uint16_t gyroGet(enum Gyro_Config_Setting gcs);

#endif // __GYRO_CONFIG_H_
