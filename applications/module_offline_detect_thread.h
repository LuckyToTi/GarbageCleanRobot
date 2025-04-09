#ifndef __MODULE_OFFLINE_DETECT_THREAD_H_
#define __MODULE_OFFLINE_DETECT_THREAD_H_

#include "stm32f4xx.h"
#include <rtthread.h>

rt_thread_t* get_module_offline_detect_tid();
void ModuleOfflineDetectThreadEntry(void* parameter);

#endif  // __MODULE_OFFLINE_DETECT_THREAD_H_
