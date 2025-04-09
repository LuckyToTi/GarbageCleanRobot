#ifndef APPLICATIONS_GPS_THREAD_H_
#define APPLICATIONS_GPS_THREAD_H_

#include "stm32f4xx.h"
#include <rtthread.h>

rt_thread_t* get_wit_gnss_data_tid();
void WitGnssThreadEntry(void* parameter);


struct WitGnssData
{
    struct
    {
        float x;
        float y;
        float z;
    }acceleration; //加速度

    struct
    {
        float roll_x;
        float pitch_y;
        float yaw_z;
    }angle;  //角度

    struct
    {
        float x;
        float y;
        float z;
    }Angular_velocity;  //角速度

    struct
    {
        float x;
        float y;
        float z;
    }Magnetic;  //磁场

    struct
    {
        float longitude;
        float latitude;
    }latitude_and_longitude;  //经纬度

    struct
    {
        float height;
        float Heading_angle;
        float grouand_speed;
    }gpsdata;  //地速

    struct
    {
        float Quaternion0;
        float Quaternion1;
        float Quaternion2;
        float Quaternion3;
    }Quaternion;  //四元数

};

#endif /* APPLICATIONS_GPSS_THREAD_H_ */
