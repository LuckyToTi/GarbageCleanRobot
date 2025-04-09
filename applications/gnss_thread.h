#ifndef APPLICATIONS_GNSS_THREAD_H_
#define APPLICATIONS_GNSS_THREAD_H_

#include "stm32f4xx.h"
#include <rtthread.h>

struct DvlPositionLocal_C75 {
    float ts;
    float x;
    float y;
    float z;
    float pos_std;
    float roll;
    float pitch;
    float yaw;
    int8_t status;
    uint8_t c;
};

rt_thread_t* get_gnss_tid();
void GnssThreadEntry(void* parameter);

#endif /* APPLICATIONS_GNSS_THREAD_H_ */
