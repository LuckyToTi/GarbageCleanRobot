#ifndef __DVL_JSON_TCP_THREAD_H_
#define __DVL_JSON_TCP_THREAD_H_


#include "stm32f4xx.h"
#include <rtthread.h>

struct DvlPositionLocal {
    float ts;
    float x;
    float y;
    float z;
    float pos_std;
    float roll;
    float pitch;
    float yaw;
    int8_t status;
};

float get_absolute_yaw_z_value();
float get_gloal_yaw();
rt_thread_t* get_dvl_json_tcp_tid();
void DvlJsonTcpThreadEntry(void* parameter);
struct DvlPositionLocal* get_dvl_position_local_data();
struct DvlPositionLocal* get_dvl_position_local_data_fixed();
float get_dvl_yaw();
int StartDvlReset();
void GetX1YRealCoordinate(float x2y_y, float x2y_x, float x1y_y, float x1y_x, float x1y_x2y_include_angle, float *x2y_to_x1y_y, float *x2y_to_x1y_x);

#endif  // __DVL_JSON_TCP_THREAD_H_
