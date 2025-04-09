#include "module_offline_detect_thread.h"
#include <stm32f4xx_hal.h>
#include "robolink_udp_task.h"
#include "userlib/module_offline_detection.h"
#include "config.h"

#define LOG_TAG     "mDtc_t"
#define LOG_LVL     LOG_LVL_INFO
#include <ulog.h>

// module_offline_detect线程控制块
rt_thread_t module_offline_detect_tid = RT_NULL;
inline rt_thread_t* get_module_offline_detect_tid()
{
    return &module_offline_detect_tid;
}

//static void ModuleOfflineDetectReportThreadEntry(void* parameter);

void ModuleOfflineDetectThreadEntry(void* parameter)
{
//    rt_thread_t report_tid = rt_thread_create("mdStMR_t", ModuleOfflineDetectReportThreadEntry,
//            (void*)0, 1024+1024, 19, 1);
    rt_thread_mdelay(750);
//    if(RT_NULL != report_tid) {
//        rt_thread_startup(report_tid);
//    }

    for( ; ; ) {
        rt_thread_mdelay(10);
        MonitorAllModules();
    }
}

//static void ModuleOfflineDetectReportThreadEntry(void* parameter)
//{
//    for( ; ; ) {
//        rt_thread_mdelay(1000);
//        char** data = NULL; int len = 0;
//        GetRobolinkModuleOnlineStatusDataSegmentList(&data, &len);
//        for(int i=0; i<len; i++) {
//            RobolinkUdpCreateInitSendNewMsgToMb((uint8_t *)(data[i] + 1), data[i][0] + 1,
//                    get_config()->robolink_id_config.local_id, 0xFE, 0x01);
//            rt_thread_mdelay(3);
//        }
//        DeleteRobolinkModuleOnlineStatusDataSegmentList(data, len);
//    }
//}


