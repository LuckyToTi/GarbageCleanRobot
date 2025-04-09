#ifndef APPLICATIONS_GYRO_THREAD_H_
#define APPLICATIONS_GYRO_THREAD_H_

#include <rtthread.h>

struct BspWitMotionGyroData
{
    struct
    {
        float x;
        float y;
        float z;
    }acceleration;          //加速度
    struct
    {
        float x;
        float y;
        float z;
    }angular_velocity;      //角速度
    struct
    {
        int hx;
        int hy;
        int hz;
    }magnetic_field;        //磁场
    struct
    {
        float roll_x;
        float pitch_y;
        float yaw_z;
    }euler_angle;           //欧拉
    struct
    {
        float q[4];
    }quaternions;           //四元数
    //float temperature;      //温度
};

const struct BspWitMotionGyroData* get_bsp_gyro_data();
rt_thread_t* get_gyro_tid();
void GyroThreadEntry(void* parameter);

#endif /* APPLICATIONS_GYRO_THREAD_H_ */
