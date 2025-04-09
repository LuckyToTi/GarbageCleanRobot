#include "hardware/sht30_sensor.h"
#include "iic3.h"
#include <string.h>

#define SHT30_DEVADD               (0x44 << 1) //设备地址
#define CMD_CLOCKSTRENABLE_H    (0x2C06)
#define CMD_SOFT_RESET          (0X30A2)

int Sht30GetMsg(float* humi, float* temp)
{
    uint8_t buf[6];
    uint16_t temp_ = 0x0000u;
    uint16_t humi_ = 0x0000u;
    if(I2c3MemRead(SHT30_DEVADD, CMD_CLOCKSTRENABLE_H, I2C_MEMADD_SIZE_16BIT, buf, 6) != 0)
    {
        return 1;  // 错误处理
    }
    temp_ =  buf[0] << 8;
    temp_ += buf[1];
    humi_ =  buf[3] << 8;
    humi_ += buf[4];
    *humi = Sht30CalcHumi(humi_);
    *temp = Sht30CalcTemp(temp_);
    return 0;
}

int Sht30SoftRest()
{
//    return I2c3MemWrite(SHT30_DEVADD, CMD_SOFT_RESET, I2C_MEMADD_SIZE_16BIT, 0x00, 0);
    return 0;
}

float Sht30CalcTemp(uint16_t raw_data)
{
    return (float)175 * raw_data / 65535 - 45;
}

float Sht30CalcHumi(uint16_t raw_data)
{
    return (float)raw_data * 100 / 65535;
}
