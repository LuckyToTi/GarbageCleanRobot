#include "ms5837.h"
#include "i2c_wd.h"
#include "math.h"
#include <rthw.h>
#include "drv_common.h"

#define Ms5837_I2c_Start() Softi2c_Start(Get_Ms5837_I2c_Device())
#define Ms5837_I2c_Stop() Softi2c_Stop(Get_Ms5837_I2c_Device())
#define Ms5837_I2c_Send_Byte(data) Softi2c_Send_Byte(Get_Ms5837_I2c_Device(), data)
#define Ms5837_I2c_Read_Byte(ack) Ms5837_I2c_Read_Byte_And_Ack(Get_Ms5837_I2c_Device(), ack)
#define Ms5837_I2c_Wait_Ack() Softi2c_Wait_Ack(Get_Ms5837_I2c_Device())
#define Ms5837_Delay_Us(nus) rt_hw_us_delay(nus)
#define Ms5837_Delay_Ms(nms) rt_thread_mdelay(nms)

uint8_t Ms5837_I2c_Read_Byte_And_Ack(const Softi2c_device_t *softi2c_device, uint8_t ack)
{
    uint8_t read_data = Softi2c_Read_Byte(softi2c_device);
    if (!ack) Softi2c_NAck(softi2c_device);
    else Softi2c_Ack(softi2c_device);
    return read_data;
}

/*
C1  压力灵敏度 SENS|T1
C2  压力补偿  OFF|T1
C3  温度压力灵敏度系数 TCS
C4  温度系数的压力补偿 TCO
C5  参考温度 T|REF
C6  温度系数的温度 TEMPSENS
*/
uint32_t Cal_C[7]; //用于存放PROM中的6组数据1-6

float Aux;

/*
dT 实际和参考温度之间的差异
TEMP 实际温度
*/
signed int dT, TEMP;

/*
OFF 实际温度补偿
SENS 实际温度灵敏度
*/
int64_t OFF_, SENS;

uint32_t D1_Pres, D2_Temp; // 数字压力值,数字温度值
double Temperature;
int32_t Pressure, Pressure_old; //大气压
int32_t OFFi = 0, SENSi = 0, Ti = 0;
int64_t OFF2 = 0;
int64_t SENS2 = 0;
uint32_t TEMP2; //温度校验值
double T2;
int32_t ms5837_error_cnt = 0; //ms5837错误计数
int8_t ms5837_state = 0;      //ms5837状态

/*******************************************************************************
  * @函数名称     MS5837BA_RESET
  * @函数说明   复位MS5837
  * @输入参数   无
  * @输出参数   无
  * @返回参数   无
*******************************************************************************/
void MS5837_30BA_ReSet(void)
{
    // 数据重置
    Pressure = 0;
    Temperature = 0;
    ms5837_error_cnt = 0;
    ms5837_state = 0;
    Ms5837_I2c_Start();
    Ms5837_I2c_Send_Byte(MS5837_30BA_WriteCommand); //发送写命令
    Ms5837_I2c_Wait_Ack();
    Ms5837_I2c_Send_Byte(MS5837_30BA_ResetCommand); //发送复位命令
    Ms5837_I2c_Wait_Ack();
    Ms5837_I2c_Stop();
}

/*******************************************************************************
  * @函数名称     MS5837_PROM
  * @函数说明   初始化5837_30BA
  * @输入参数   无
  * @输出参数   无
  * @返回参数   无
  * @全局变量   Cal_C数组,用于CRC校验以及计算压力值和温度值
*******************************************************************************/
uint8_t MS5837_30BA_PROM(void)
{
    uint8_t inth, intl;
    uint8_t i;

    MS5837_30BA_ReSet(); //复位MS5837
    Ms5837_Delay_Ms(20);
    for (i = 0; i < 7; i++)
    {

        Ms5837_I2c_Start();
        Ms5837_I2c_Send_Byte(MS5837_30BA_WriteCommand); //发送写命令
        Ms5837_I2c_Wait_Ack();
        Ms5837_I2c_Send_Byte(MS5837_30BA_PROM_RD + (i * 2)); //发送PROM获取命令
        Ms5837_I2c_Wait_Ack();
        Ms5837_I2c_Stop();
        Ms5837_Delay_Us(5);
        Ms5837_I2c_Start();
        Ms5837_I2c_Send_Byte(MS5837_30BA_ReadCommand); //进入接收模式
        Ms5837_Delay_Us(5);
        Ms5837_I2c_Wait_Ack();
        inth = Ms5837_I2c_Read_Byte(1); //带ACK的读数据
        Ms5837_Delay_Us(5);
        intl = Ms5837_I2c_Read_Byte(0); //最后一个字节NACK
        Ms5837_I2c_Stop();
        Cal_C[i] = (((uint16_t)inth << 8) | intl);
    }
    return !Cal_C[0];
}

/**************************实现函数********************************************
*函数原型:    unsigned long MS5837BA_getConversion(void)
*功　　能:    读取 MS5837 的转换结果
*******************************************************************************/
unsigned long MS5837_30BA_GetConversion(uint8_t command)
{

    unsigned long conversion = 0;
    uint8_t temp[3];

    Ms5837_I2c_Start();
    Ms5837_I2c_Send_Byte(MS5837_30BA_WriteCommand); //写地址
    ms5837_error_cnt += Ms5837_I2c_Wait_Ack();
    Ms5837_I2c_Send_Byte(command); //写转换命令
    ms5837_error_cnt += Ms5837_I2c_Wait_Ack();
    Ms5837_I2c_Stop();

    Ms5837_Delay_Ms(10);
    Ms5837_I2c_Start();
    Ms5837_I2c_Send_Byte(MS5837_30BA_WriteCommand); //写地址
    ms5837_error_cnt += Ms5837_I2c_Wait_Ack();
    Ms5837_I2c_Send_Byte(0); // start read sequence
    ms5837_error_cnt += Ms5837_I2c_Wait_Ack();
    Ms5837_I2c_Stop();

    Ms5837_I2c_Start();
    Ms5837_I2c_Send_Byte(MS5837_30BA_ReadCommand); //进入接收模式
    ms5837_error_cnt += Ms5837_I2c_Wait_Ack();
    temp[0] = Ms5837_I2c_Read_Byte(1); //带ACK的读数据  bit 23-16
    temp[1] = Ms5837_I2c_Read_Byte(1); //带ACK的读数据  bit 8-15
    temp[2] = Ms5837_I2c_Read_Byte(0); //带NACK的读数据 bit 0-7
    Ms5837_I2c_Stop();

    conversion = (unsigned long)temp[0] * 65536 + (unsigned long)temp[1] * 256 + (unsigned long)temp[2];
    return conversion;
}

/**************************实现函数********************************************
*函数原型:    void MS5837_30BA_GetData(void)
*功　　能:    获取温度压力数据并进行二阶温度补偿
*******************************************************************************/

void MS5837_30BA_GetData(void)
{

    D2_Temp = MS5837_30BA_GetConversion(MS5837_30BA_D2_OSR_8192);
    Ms5837_Delay_Ms(20);
    D1_Pres = MS5837_30BA_GetConversion(MS5837_30BA_D1_OSR_8192);
    Ms5837_Delay_Ms(20);
    dT = D2_Temp - (((uint32_t)Cal_C[5]) * 256l);
    SENS = (int64_t)Cal_C[1] * 32768l + ((int64_t)Cal_C[3] * dT) / 256l;
    OFF_ = (int64_t)Cal_C[2] * 65536l + ((int64_t)Cal_C[4] * dT) / 128l;
    TEMP = 2000l + (int64_t)(dT)*Cal_C[6] / 8388608LL;

    //二阶温度补偿
    if (TEMP < 2000) // low temp
    {
        Ti = (3 * (int64_t)(dT) * (int64_t)(dT) / (8589934592LL));
        OFFi = (3 * (TEMP - 2000) * (TEMP - 2000)) / 2;
        SENSi = (5 * (TEMP - 2000) * (TEMP - 2000)) / 8;
    }
    else
    { // high temp
        Ti = 2 * (dT * dT) / (137438953472LL);
        OFFi = (1 * (TEMP - 2000) * (TEMP - 2000)) / 16;
        SENSi = 0;
    }
    OFF2 = OFF_ - OFFi;
    SENS2 = SENS - SENSi;
    Pressure = ((D1_Pres * SENS2) / 2097152l - OFF2) / 8192l;      //校准后压力数据 单位：0.1mBar
    Temperature = (TEMP - Ti) / 100.0;                             //校准后温度数据
}

#define DBG_TAG "ms5837"
#define DBG_LVL DBG_ERROR
#include <rtdbg.h>

/**************************实现函数********************************************
*函数原型:    unsigned char MS5837_30BA_Crc4()
*功　　能:    对获取的Cal_C数组进行CRC校验
*******************************************************************************/

unsigned char MS5837_30BA_Crc4()
{
    int cnt;
    int t;
    unsigned int n_rem = 0;
    unsigned char n_bit;
    unsigned char a = 0;
    unsigned char b = 0;
    unsigned short int n_prom[8];

    for (t = 0; t < 7; t++)
    {
        n_prom[t] = Cal_C[t];
    }
    n_prom[0] = ((n_prom[0]) & 0x0FFF);
    n_prom[7] = 0;
    for (cnt = 0; cnt < 16; cnt++)
    {
        if (cnt % 2 == 1)
            n_rem ^= (unsigned short)((n_prom[cnt >> 1]) & 0x00FF);
        else
            n_rem ^= (unsigned short)(n_prom[cnt >> 1] >> 8);
        for (n_bit = 8; n_bit > 0; n_bit--)
        {
            if (n_rem & (0x8000))
                n_rem = (n_rem << 1) ^ 0x3000;
            else
                n_rem = (n_rem << 1);
        }
    }
    n_rem = ((n_rem >> 12) & 0x000F);
    a = (n_rem ^ 0x00);
    b = Cal_C[0] >> 12;

    if (a == b)
    {
        return 1;
    }
    else
    {
        LOG_D("crc16 %d %d", a, b);
        return 0;
    }
}
