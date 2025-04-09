#ifndef __EQUIPMENT_THREAD_H__
#define __EQUIPMENT_THREAD_H__

#include "stm32f4xx.h"
#include "rthw.h"
#include "rtthread.h"

rt_thread_t* get_equipment_tid();
void EquipmentThreadEntry(void* parameter);

#endif /* __EQUIPMENT_THREAD_H__ */
