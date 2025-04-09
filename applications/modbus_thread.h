#ifndef APPLICATIONS_MODBUS_THREAD_H_
#define APPLICATIONS_GYRO_MB_THREAD_H_

#include "stm32f4xx.h"
#include <rtthread.h>

struct PowerManageData{
    uint16_t cell1;
    uint16_t cell2;
    uint16_t cell3;
    uint16_t cell4;

    uint16_t Totalvoltage;
    uint16_t extertemp1;
    uint16_t extertemp2;
    uint16_t Dieremp;

    uint16_t cadccurrent;
    uint16_t Fu11chgCaP;
    uint16_t Remaincap;
    uint16_t RSOC;

    uint16_t CycleCount;
    uint16_t Packstatus;
    uint16_t Batterystatus;
};

rt_thread_t* get_modbus_tid();
void modbus_thread(void* parameter);
const struct PowerManageData* get_PowerManage_msg();

#endif /* APPLICATIONS_MODBUS_THREAD_H_ */
