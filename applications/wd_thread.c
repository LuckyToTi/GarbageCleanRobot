#include "wd_thread.h"
#include <stm32f4xx_hal.h>
#include <rthw.h>
#include "hardware/i2c_wd.h"
#include "hardware/ms5837.h"
#include "robolink_udp_task.h"
#include "config.h"

#define NOW_AIR_PRESS sensor_now_air_pressure
#define ERROR_CNT_THRESHOLD 30

static int32_t last_ms5837_error_cnt; //上次的传感器错误计数
static uint32_t sensor_now_air_pressure = 0xFFFFFFFF;  //当前大气压强
static uint32_t sensor_pressure = 0xFFFFFFFF; // 原始压力数据 0.1 mBar
static int32_t sensor_waterdepth = (int32_t)0xFFFFFFFF; // 单位：0.1 cm
static int16_t sensor_temperature = (int16_t)0xFFFF; // temperature/10 = 实际温度值 单位：摄氏度

rt_thread_t wd_tid = RT_NULL;
inline rt_thread_t* get_wd_tid()
{
    return &wd_tid;
}

void WaterdepthThreadEntry(void* parameter)
{
    rt_thread_mdelay(750);
    for(;;)
    {
        rt_thread_mdelay(100);
         Ms5837_Softi2c_Init();
 MS5837_RESET:
         rt_thread_mdelay(10);
         MS5837_30BA_ReSet(); //复位MS5837
         rt_thread_mdelay(10);
         MS5837_30BA_PROM(); //初始化MS5837
         rt_thread_mdelay(10);
         if (MS5837_30BA_Crc4()) // CRC4校验
         {
             ms5837_error_cnt = 0;
             ms5837_state = 1u;
         }
         else //校验失败重新初始化
         {
             goto MS5837_RESET;
         }
         rt_thread_mdelay(100);

         MS5837_30BA_GetData();
         sensor_now_air_pressure = get_config()->water_depth_sensor_config.air_press_custom_switch ? get_config()->water_depth_sensor_config.air_press : Pressure;
         rt_thread_mdelay(100);
         // 开始读取传感器数据
         for (;;)
         {
             last_ms5837_error_cnt = ms5837_error_cnt;
             MS5837_30BA_GetData();
             if(ms5837_error_cnt > ERROR_CNT_THRESHOLD)
             {
                 sensor_now_air_pressure = 0xFFFFFFFF;
                 sensor_pressure = 0xFFFFFFFF;
                 sensor_waterdepth = 0xFFFFFFFF;
                 sensor_temperature = 0xFFFF;
                 goto MS5837_RESET;
             }
             else if(ms5837_error_cnt == last_ms5837_error_cnt)
             {
                 if (ms5837_error_cnt > 0) //缓慢消除错误
                 {
                     ms5837_error_cnt--;
                 }
                 sensor_pressure = Pressure;
                 sensor_waterdepth = Pressure > NOW_AIR_PRESS ? (Pressure-NOW_AIR_PRESS) : 0;
                 sensor_temperature = Temperature * 100.0f;
                 float wd = sensor_waterdepth / 1000.0f;
                 RobolinkUdpCreateInitSendNewMsgToMb((uint8_t*)&wd, 4,
                                         get_config()->robolink_id_config.local_id, 0x07, 0x01);
                 float tp = Temperature;
                 RobolinkUdpCreateInitSendNewMsgToMb((uint8_t*)&tp, 4,
                                                          get_config()->robolink_id_config.local_id, 0x07, 0x02);
                 //LOG_D("waterdepth = %f\t temperature = %f\r\n",wd,tp);
             }
            rt_thread_mdelay(20);
         }
    }
}

/**
 * @brief 获取原始压力数据
 *
 * @return uint32_t
 */
uint32_t Get_Sensor_Pressure(void)
{
    return sensor_pressure;
}

/**
 * @brief 获取当前水深
 *
 * @return int32_t 单位：mm
 */
int32_t Get_Water_Depth(void)
{
    return sensor_waterdepth;
}

/**
 * @brief 获取当前温度
 *
 * @return int16_t (return_value / 100) = 温度，单位：摄氏度
 */
int16_t Get_Temperature(void)
{
    return sensor_temperature;
}
