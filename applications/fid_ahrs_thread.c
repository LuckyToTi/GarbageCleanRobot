/*
 * Copyright (c) 2006-2021, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2025-01-07     24144       the first version
 */


#include "fid_ahrs_thread.h"
#include "hardware/usart1.h"
#include "userlib/bytetransformers.h"
#include "math.h"
#include "config.h"
#include "robolink_udp_task.h"

#define LOG_TAG     "fid_ahrs"
//#define LOG_LVL     LOG_LVL_INFO
#include <ulog.h>

static void FidAnalyzeData(struct fid_data* fid_analyze_data,const uint8_t* Primitive_data,int get_len);

rt_thread_t fid_ahrs_tid = RT_NULL;
inline rt_thread_t* get_fid_ahrs_tid()
{
    return &fid_ahrs_tid;
}

const uint16_t fid_recv_max_data = 512;

struct fid_data fid_analyze_data = {};

inline const struct fid_data* get_fid()
{
    return &fid_analyze_data;
}



void FidThreadEntry(void* parameter)
{
    uint8_t* fid_recv_data = rt_malloc(fid_recv_max_data);
    rt_thread_mdelay(750);
    for (;;)
    {
        rt_memset(fid_recv_data, 0 ,fid_recv_max_data);
        int get_len = Uart1HalReceiveWaitDataItStopIdle(fid_recv_data,fid_recv_max_data);
        if(!(get_len>0)) continue;
        FidAnalyzeData(&fid_analyze_data,fid_recv_data,get_len);  // 解析数据
        LOG_D(" %f %f %f ",fid_analyze_data.euler_angle.roll_x,fid_analyze_data.euler_angle.pitch_y,fid_analyze_data.euler_angle.yaw_z);
        uint8_t upload_fid_recv_data[12];
        rt_memcpy(upload_fid_recv_data + 0,&fid_analyze_data.euler_angle.roll_x, sizeof(float));
        rt_memcpy(upload_fid_recv_data + 4,&fid_analyze_data.euler_angle.pitch_y, sizeof(float));
        rt_memcpy(upload_fid_recv_data + 8,&fid_analyze_data.euler_angle.yaw_z, sizeof(float));
        RobolinkUdpCreateInitSendNewMsgToMb(upload_fid_recv_data,12,get_config()->robolink_id_config.local_id, 0x02, 0x04);
    }
}



static void FidAnalyzeData(struct fid_data* fid_analyze_data,const uint8_t* Primitive_data,int get_len)
{
    if(Primitive_data[0]==0xfc && Primitive_data[1]==0x41 && Primitive_data[2]==0x30 && get_len == 56)
    {
        fid_analyze_data->angular_velocity.x = get_f(Primitive_data+7,LSB) * (180 / M_PI);
        fid_analyze_data->angular_velocity.y = get_f(Primitive_data+11,LSB) * (180 / M_PI);
        fid_analyze_data->angular_velocity.z = get_f(Primitive_data+15,LSB) * (180 / M_PI);
        fid_analyze_data->euler_angle.roll_x = get_f(Primitive_data+19,LSB) * (180 / M_PI);
        fid_analyze_data->euler_angle.pitch_y = get_f(Primitive_data+23,LSB) * (180 / M_PI);
        fid_analyze_data->euler_angle.yaw_z = get_f(Primitive_data+27,LSB)* (180 / M_PI) - 180;
        fid_analyze_data->quaternion.q1 = get_f(Primitive_data+31,LSB);
        fid_analyze_data->quaternion.q2 = get_f(Primitive_data+35,LSB);
        fid_analyze_data->quaternion.q3 = get_f(Primitive_data+39,LSB);
        fid_analyze_data->quaternion.q4 = get_f(Primitive_data+43,LSB);
    }
}






