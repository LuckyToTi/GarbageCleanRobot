/*
 * Copyright (c) 2006-2021, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2025-01-07     24144       the first version
 */
#ifndef APPLICATIONS_FID_AHRS_THREAD_H_
#define APPLICATIONS_FID_AHRS_THREAD_H_


#include "rtthread.h"
#include "stm32f4xx.h"


struct fid_data
{
     struct
     {
         float x;
         float y;
         float z;
     }angular_velocity;      //角速度

     struct
     {
         float roll_x;
         float pitch_y;
         float yaw_z;
     }euler_angle;           //欧拉角

     struct
     {
         float q1;
         float q2;
         float q3;
         float q4;
     }quaternion;            //四元数
};

rt_thread_t* get_fid_ahrs_tid();
void FidThreadEntry(void* parameter);


#endif /* APPLICATIONS_FID_AHRS_THREAD_H_ */
