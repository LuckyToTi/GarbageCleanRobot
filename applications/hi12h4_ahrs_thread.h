#ifndef APPLICATIONS_HI12H4_AHRS_THREAD_H_
#define APPLICATIONS_HI12H4_AHRS_THREAD_H_

#include "stm32f4xx.h"
#include <rtthread.h>

struct hi12h4_imu_data_t
{
    uint8_t tag;
    struct
    {
        float x;
        float y;
        float z;
    } acc;          //加速度
    struct
    {
        float x;
        float y;
        float z;
    } gyr;      //角速度
    struct
    {
        float hx;
        float hy;
        float hz;
    } mag;        //磁场
    struct
    {
        float roll_x;
        float pitch_y;
        float yaw_z;
    } euler;           //欧拉
    struct
    {
        float q[4];
    } quat;           //四元数
    float pressure;

    uint32_t timestamp;
};

const struct hi12h4_imu_data_t* get_hi12h4_imu_data();
rt_thread_t* get_hi12h4_ahrs_tid();
void HI12H4AhrsThreadEntry(void* parameter);

#endif /* APPLICATIONS_HI12H4_AHRS_THREAD_H_ */
