#include "softi2c.h"

uint8_t Softi2c_Device_Init_Check(Softi2c_device_t *softi2c_device)
{
    softi2c_device->init_flag = 0;
    if (softi2c_device->I2c_Scl != NULL && softi2c_device->I2c_Sda != NULL && softi2c_device->Read_Sda != NULL && softi2c_device->Softi2c_Delay_Us != NULL)
    {
        softi2c_device->init_flag = 1;
    }
    return softi2c_device->init_flag;
}

void Softi2c_Device_Deinit(Softi2c_device_t* softi2c_device)
{
    softi2c_device->Set_Sda_In_Mode = NULL;
    softi2c_device->Set_Sda_Out_Mode = NULL;
    softi2c_device->I2c_Scl = NULL;
    softi2c_device->I2c_Sda = NULL;
    softi2c_device->Read_Sda = NULL;
    softi2c_device->Softi2c_Delay_Us = NULL;
    softi2c_device->delay_time_us = 0;
    softi2c_device->init_flag = 0;
}

void Softi2c_Start(const Softi2c_device_t* softi2c_device)
{
    softi2c_device->Set_Sda_Out_Mode();
    softi2c_device->I2c_Sda(1);
    softi2c_device->I2c_Scl(1);
    softi2c_device->Softi2c_Delay_Us(softi2c_device->delay_time_us/2);
    softi2c_device->I2c_Sda(0);
    softi2c_device->Softi2c_Delay_Us(softi2c_device->delay_time_us);
    softi2c_device->I2c_Scl(0);
    softi2c_device->Softi2c_Delay_Us(softi2c_device->delay_time_us/2);
}

//产生IIC停止信号
void Softi2c_Stop(const Softi2c_device_t* softi2c_device)
{
    softi2c_device->Set_Sda_Out_Mode();
    softi2c_device->I2c_Sda(0);
    softi2c_device->I2c_Scl(0);
    softi2c_device->Softi2c_Delay_Us(softi2c_device->delay_time_us/2);
    softi2c_device->I2c_Scl(1);
    softi2c_device->Softi2c_Delay_Us(softi2c_device->delay_time_us);
    softi2c_device->I2c_Sda(1);
    softi2c_device->Softi2c_Delay_Us(softi2c_device->delay_time_us/2);
}

//等待应答信号到来
//返回值：1，接收应答失败
//        0，接收应答成功
unsigned char Softi2c_Wait_Ack(const Softi2c_device_t* softi2c_device)
{
    unsigned char ucErrTime=0;
    softi2c_device->Set_Sda_In_Mode();
    softi2c_device->I2c_Sda(1);
    softi2c_device->Softi2c_Delay_Us(1);
    softi2c_device->I2c_Scl(1);
    softi2c_device->Softi2c_Delay_Us(1);
    while(softi2c_device->Read_Sda())
    {
        ucErrTime++;
        if(ucErrTime>250)
        {
            Softi2c_Stop(softi2c_device);
            return 1;
        }
    }
    softi2c_device->I2c_Scl(0);//时钟输出0
    return 0;
}
//产生ACK应答
void Softi2c_Ack(const Softi2c_device_t* softi2c_device)
{
    softi2c_device->I2c_Scl(0);
    softi2c_device->Set_Sda_Out_Mode();;
    softi2c_device->I2c_Sda(0);
    softi2c_device->Softi2c_Delay_Us(softi2c_device->delay_time_us/2);
    softi2c_device->I2c_Scl(1);
    softi2c_device->Softi2c_Delay_Us(softi2c_device->delay_time_us);
    softi2c_device->I2c_Scl(0);
    softi2c_device->Softi2c_Delay_Us(softi2c_device->delay_time_us/2);
}
//不产生ACK应答
void Softi2c_NAck(const Softi2c_device_t* softi2c_device)
{
    softi2c_device->I2c_Scl(0);
    softi2c_device->Set_Sda_Out_Mode();
    softi2c_device->I2c_Sda(1);
    softi2c_device->Softi2c_Delay_Us(softi2c_device->delay_time_us/2);
    softi2c_device->I2c_Scl(1);
    softi2c_device->Softi2c_Delay_Us(softi2c_device->delay_time_us);
    softi2c_device->I2c_Scl(0);
    softi2c_device->Softi2c_Delay_Us(softi2c_device->delay_time_us/2);
}

//IIC发送一个字节
//返回从机有无应答
//1，有应答
//0，无应答
void Softi2c_Send_Byte(const Softi2c_device_t *softi2c_device, unsigned char tx_data)
{
    unsigned char _t;
    softi2c_device->Set_Sda_Out_Mode();
    softi2c_device->I2c_Scl(0);
    for (_t = 0; _t < 8; _t++)
    {
        softi2c_device->I2c_Sda(((tx_data & 0x80) >> 7));
        tx_data <<= 1;
        softi2c_device->Softi2c_Delay_Us(softi2c_device->delay_time_us/2);
        softi2c_device->I2c_Scl(1);
        softi2c_device->Softi2c_Delay_Us(softi2c_device->delay_time_us);
        softi2c_device->I2c_Scl(0);
        softi2c_device->Softi2c_Delay_Us(softi2c_device->delay_time_us/2);
    }
}

//读1个字节，ack=1时，发送ACK，ack=0，发送nACK
unsigned char Softi2c_Read_Byte(const Softi2c_device_t *softi2c_device)
{
    unsigned char _t = 0;
    unsigned char receive = 0;
    softi2c_device->Set_Sda_In_Mode();
    softi2c_device->I2c_Scl(0);
    for (_t = 0; _t < 8; _t++)
    {
        softi2c_device->Softi2c_Delay_Us(softi2c_device->delay_time_us/2);
        softi2c_device->I2c_Scl(1);
        softi2c_device->Softi2c_Delay_Us(softi2c_device->delay_time_us);
        receive <<= 1;
        if (softi2c_device->Read_Sda())
        {
            receive++;
        }
        softi2c_device->I2c_Scl(0);
        softi2c_device->Softi2c_Delay_Us(softi2c_device->delay_time_us/2);
    }
    return receive;
}

unsigned char Softi2c_Write_Bytes(const Softi2c_device_t *softi2c_device, unsigned char dev_addr, unsigned char *w_data, unsigned char w_len)
{
    unsigned char w_cnt = 0;
    Softi2c_Start(softi2c_device);
    Softi2c_Send_Byte(softi2c_device, dev_addr << 1); //发送写命令
    Softi2c_Wait_Ack(softi2c_device);

    for (w_cnt = 0; w_cnt < w_len; w_cnt++)
    {
        Softi2c_Send_Byte(softi2c_device, w_data[w_cnt]); //发送地址
        Softi2c_Wait_Ack(softi2c_device);
    }
    Softi2c_Stop(softi2c_device); //产生一个停止条件
    return 1;
}

unsigned char Softi2c_Write_Then_Read_Bytes(const Softi2c_device_t *softi2c_device, unsigned char dev_addr, unsigned char *w_data, unsigned char w_len, unsigned char *r_data, unsigned char r_len)
{
    unsigned char w_cnt = 0;
    unsigned char r_cnt = 0;
    Softi2c_Start(softi2c_device);
    Softi2c_Send_Byte(softi2c_device, dev_addr << 1); //发送写命令
    Softi2c_Wait_Ack(softi2c_device);

    for (w_cnt = 0; w_cnt < w_len; w_cnt++)
    {
        Softi2c_Send_Byte(softi2c_device, w_data[w_cnt]); //发送地址
        Softi2c_Wait_Ack(softi2c_device);
    }

    Softi2c_Start(softi2c_device);
    Softi2c_Send_Byte(softi2c_device, (dev_addr << 1) + 1); //进入接收模式
    Softi2c_Wait_Ack(softi2c_device);

    for (r_cnt = 0; r_cnt < r_len; r_cnt++)
    {
        r_data[r_cnt] = Softi2c_Read_Byte(softi2c_device);
        if (r_cnt != r_len - 1)
            Softi2c_Ack(softi2c_device); //带ACK的读数据
        else
            Softi2c_NAck(softi2c_device); //最后一个字节NACK
    }
    Softi2c_Stop(softi2c_device); //产生一个停止条件
    return r_cnt;
}
