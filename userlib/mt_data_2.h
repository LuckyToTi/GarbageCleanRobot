#ifndef __MT_DATA_2_H_
#define __MT_DATA_2_H_

#include "stm32f4xx.h"

#define XBUS_DATA_SEG_OFFSET (4u)

struct MTData2 {
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
    }euler_angle;           //欧拉角
};

int XBusMTData2HeaderCheck(const uint8_t* data);
int XBusCheckGetLen(const uint8_t* data, int data_len);
int XBusCheckChecksum(const uint8_t* data, int data_len);
int OrientationEulerAnglesDataParse(const uint8_t* data, struct MTData2* mtdata2);

#endif  // __MT_DATA_2_H_
