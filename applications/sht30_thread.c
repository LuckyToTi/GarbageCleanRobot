#include "sht30_thread.h"
#include <stm32f4xx_hal.h>
#include "hardware/sht30_sensor.h"
#include "config.h"
#include "robolink_udp_task.h"
#include "userlib/module_offline_detection.h"

#define LOG_TAG     "sht30_t"
#define LOG_LVL     LOG_LVL_INFO
#include <ulog.h>

uint8_t get_temp_and_humi_sensor_dev_id();
inline uint8_t get_temp_and_humi_sensor_dev_id() {return 0x03;}

// sht30线程控制块
rt_thread_t sht30_tid = RT_NULL;
inline rt_thread_t* get_sht30_tid()
{
    return &sht30_tid;
}

void Sht30ThreadEntry(void* parameter)
{
    rt_thread_mdelay(250);
    struct MonitoredModule* sht30_m = RegisterNewMonitoredModule("sht30", 0, 750, 2, NULL);
    Sht30SoftRest();
    for( ; ; ) {
        rt_thread_mdelay(500);
        float temp = 0.0f, humi = 0.0f;
        if(0 == Sht30GetMsg(&humi, &temp))
            MonitoredModuleReload(sht30_m);

        // 湿度数据
        RobolinkUdpCreateInitSendNewMsgToMb((uint8_t*)&humi, sizeof(float), get_config()->robolink_id_config.local_id, get_temp_and_humi_sensor_dev_id(), 0x03);
        LOG_D("SHT30_Humi: %.2f", humi);

        // 温度数据
        RobolinkUdpCreateInitSendNewMsgToMb((uint8_t*)&temp, sizeof(float), get_config()->robolink_id_config.local_id, get_temp_and_humi_sensor_dev_id(), 0x04);
        LOG_D("SHT30_Temp: %.2f", temp);
    }
    UnregisterMonitoredModuleByPtr(sht30_m);
}

