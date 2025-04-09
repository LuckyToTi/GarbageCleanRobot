/*
 * Copyright (c) 2006-2021, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2024-12-19     24144       the first version
 */

#include "yesens_thread.h"
#include "hardware/usart1.h"
#include "string.h"
#include "robolink_udp_task.h"
#include "config.h"

#define LOG_TAG     "yesens_t"         // 该模块对应的标签。不定义时，默认：NO_TAG
#define LOG_LVL     LOG_LVL_INFO  // 该模块对应的日志输出级别。不定义时，默认：调试级别
#include <ulog.h>                 // 必须在 LOG_TAG 与 LOG_LVL 下面

const uint16_t yesens_recv_max_data = 512;

protocol_info_t yesens_analyze_data = {};

inline const protocol_info_t* get_yesens_data()
{
    return &yesens_analyze_data;
}

rt_thread_t yesens_tid = RT_NULL;
inline rt_thread_t* get_yesens_tid()
{
    return &yesens_tid;
}

void YsensThreadEntry(void* parameter)
{
    unsigned char* yesens_recv_data = rt_malloc(yesens_recv_max_data);
    rt_thread_mdelay(750);
    for( ; ; )
    {
        rt_memset(yesens_recv_data, 0, yesens_recv_max_data);
        int get_len = Uart1HalReceiveWaitDataItStopIdle((uint8_t*)yesens_recv_data, yesens_recv_max_data);
//        LOG_D("%d",get_len);
        if(get_len != 67) continue;
        analysis_data(yesens_recv_data, get_len, &yesens_analyze_data);  // 解析yesens数据
//        LOG_D("err = %d",errcode);
        LOG_D("roll = %f ,pitch = %f ,yaw = %f",yesens_analyze_data.attitude.roll, yesens_analyze_data.attitude.pitch, yesens_analyze_data.attitude.yaw);
//        LOG_D("q0 = %f,q1 = %f,q2 = %f,q3 = %f,",yesens_analyze_data.attitude.quaternion_data0, yesens_analyze_data.attitude.quaternion_data1, yesens_analyze_data.attitude.quaternion_data2, yesens_analyze_data.attitude.quaternion_data3);
//        LOG_D("angle_x = %f, angle_y = %f, angle_z = %f",yesens_analyze_data.angle_rate.x, yesens_analyze_data.angle_rate.y, yesens_analyze_data.angle_rate.z);
//        LOG_D("accel_x = %f, accel_y = %f accel_z = %f",yesens_analyze_data.accel.x, yesens_analyze_data.accel.y, yesens_analyze_data.accel.z);
//        LOG_D("norm_mag.x = %f, norm_mag.y = %f, norm_mag.z = %f",yesens_analyze_data.norm_mag.x, yesens_analyze_data.norm_mag.y, yesens_analyze_data.norm_mag.z);
//        LOG_D("raw_mag.x = %f, raw_mag.y = %f, raw_mag.z = %f",yesens_analyze_data.raw_mag.x, yesens_analyze_data.raw_mag.y, yesens_analyze_data.raw_mag.z);
        uint8_t upload_yesens_recv_data[12];
        rt_memcpy(upload_yesens_recv_data + 0,&yesens_analyze_data.attitude.roll, sizeof(float));
        rt_memcpy(upload_yesens_recv_data + 4,&yesens_analyze_data.attitude.pitch, sizeof(float));
        rt_memcpy(upload_yesens_recv_data + 8,&yesens_analyze_data.attitude.yaw, sizeof(float));
        RobolinkUdpCreateInitSendNewMsgToMb(upload_yesens_recv_data,12,get_config()->robolink_id_config.local_id, 0x02, 0x04);
        rt_thread_mdelay(30);
    }
    rt_free(yesens_recv_data);
}



