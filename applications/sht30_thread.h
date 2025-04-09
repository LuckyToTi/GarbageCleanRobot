#ifndef APPLICATIONS_SHT30_THREAD_H_
#define APPLICATIONS_SHT30_THREAD_H_


#include "stm32f4xx.h"
#include <rtthread.h>

rt_thread_t* get_sht30_tid();
void Sht30ThreadEntry(void* parameter);

#endif /* APPLICATIONS_SHT30_THREAD_H_ */
