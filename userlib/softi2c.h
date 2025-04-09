#ifndef USERLIB_SOFTI2C_H_
#define USERLIB_SOFTI2C_H_

#include "stm32f4xx.h"

typedef struct
{
    void (* Set_Sda_In_Mode)(void); ///< 设置Sda输入模式
    void (* Set_Sda_Out_Mode)(void); ///< 设置Sda输出模式
    void (* I2c_Scl)(unsigned char); ///< 设置Scl输出电平
    void (* I2c_Sda)(unsigned char); ///< 设置Sda输出电平
    unsigned char (* Read_Sda)(void); ///< 读Sda输入电平
    void (* Softi2c_Delay_Us)(unsigned int); ///< 毫秒级延时
    uint32_t delay_time_us;
    uint8_t init_flag;
}Softi2c_device_t;

uint8_t Softi2c_Device_Init_Check(Softi2c_device_t *softi2c_device);
void Softi2c_Device_Deinit(Softi2c_device_t* softi2c_device);

void Softi2c_Start(const Softi2c_device_t* softi2c_device);
void Softi2c_Stop(const Softi2c_device_t* softi2c_device);
unsigned char Softi2c_Wait_Ack(const Softi2c_device_t* softi2c_device);
void Softi2c_Ack(const Softi2c_device_t* softi2c_device);
void Softi2c_NAck(const Softi2c_device_t* softi2c_device);
void Softi2c_Send_Byte(const Softi2c_device_t *softi2c_device, unsigned char tx_data);
unsigned char Softi2c_Read_Byte(const Softi2c_device_t *softi2c_device);

unsigned char Softi2c_Write_Bytes(const Softi2c_device_t *softi2c_device, unsigned char dev_addr, unsigned char *w_data, unsigned char w_len);
unsigned char Softi2c_Write_Then_Read_Bytes(const Softi2c_device_t *softi2c_device, unsigned char dev_addr, unsigned char *w_data, unsigned char w_len, unsigned char *r_data, unsigned char r_len);

#endif /* USERLIB_SOFTI2C_H_ */
