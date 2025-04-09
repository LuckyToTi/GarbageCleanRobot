#ifndef __IIC3_H_
#define __IIC3_H_

#include <stm32f4xx.h>

I2C_HandleTypeDef* get_hi2c3_handle();
void I2c3BspInit();
int I2c3MemWrite(uint16_t DevAddress, uint16_t MemAddress, uint16_t MemAddSize, uint8_t *pData, uint16_t Size);
int I2c3MemRead(uint16_t DevAddress, uint16_t MemAddress, uint16_t MemAddSize, uint8_t *pData, uint16_t Size);
int I2c3MemRead_NoRtos(uint16_t DevAddress, uint16_t MemAddress, uint16_t MemAddSize, uint8_t *pData, uint16_t Size);

#endif  // __IIC2_H_
