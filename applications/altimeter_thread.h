#ifndef APPLICATIONS_ALTIMETER_THREAD_H_
#define APPLICATIONS_ALTIMETER_THREAD_H_

#include <rtthread.h>

struct AltimeterData {
    uint32_t distance;
    uint16_t DataConfidence;
    uint16_t SendTime;
    uint32_t ping_number;
    uint32_t scan_start;
    uint32_t scan_length;
    uint32_t gain_seting;
    uint16_t profile_data_length;
    uint8_t profile_data[200];
};

const struct AltimeterData* get_altimeter_msg();

rt_thread_t* get_altimeter_tid();
void AltimeterThreadEntry(void* parameter);

#endif /* APPLICATIONS_ALTIMETER_THREAD_H_ */
