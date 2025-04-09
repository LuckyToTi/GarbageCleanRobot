#ifndef __SHT30_SENSOR_H_
#define __SHT30_SENSOR_H_

#include "stm32f4xx.h"

int Sht30SoftRest();
int Sht30GetMsg(float* humi, float* temp);
float Sht30CalcTemp(uint16_t raw_data);
float Sht30CalcHumi(uint16_t raw_data);


#endif // _SHT30_SENSOR_H_
