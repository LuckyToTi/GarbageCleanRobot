#ifndef APPLICATIONS_XSENS_AHRS_THREAD_H_
#define APPLICATIONS_XSENS_AHRS_THREAD_H_

#include "stm32f4xx.h"
#include <rtthread.h>
#include "userlib/mt_data_2.h"

const struct MTData2* get_mt_data_2();
rt_thread_t* get_xsens_ahrs_tid();
void XsensAhrsThreadEntry(void* parameter);

#endif /* APPLICATIONS_XSENS_AHRS_THREAD_H_ */
