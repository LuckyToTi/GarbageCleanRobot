#ifndef __MODULE_OFFLINE_DETECTION_H_
#define __MODULE_OFFLINE_DETECTION_H_

#include "rtthread.h"

#define MAX_CHECK_INTERVAL_10MS (1000)  // 对应10s

struct MonitoredModule {
    char* name;

    int32_t check_interval;  // *10ms (1-100)
    int32_t max_timeout_times;  // 最大超时次数

    uint8_t is_reload;  // 0 1
    int32_t current_timeout_count;

    uint8_t mode;  // 超时模式 0（超时检测） 1（直接设置模块状态）
    uint8_t state;  // 0:离线 1:在线

    void (*state_changed_func)(struct MonitoredModule*);
};

void MonitoredModuleInit();
struct MonitoredModule* RegisterNewMonitoredModule(char* name, uint8_t mode,
        int32_t check_interval_ms, int32_t max_timeout_times, void (*state_changed_func)(struct MonitoredModule*));
int UnregisterMonitoredModuleByPtr(struct MonitoredModule* module);
int UnregisterMonitoredModuleByName(char* name);
void MonitorAllModules();
struct MonitoredModule* FindMonitoredModuleByName(char* name);
void MonitoredModuleReload(struct MonitoredModule* module);
int32_t MonitoredModuleReloadByName(char* name);
void SetMonitoredModuleState(struct MonitoredModule* module, uint8_t state);
int32_t SetMonitoredModuleStateByName(char* name, uint8_t state);

void GetRobolinkModuleOnlineStatusDataSegmentList(char*** data, int* num);
void DeleteRobolinkModuleOnlineStatusDataSegmentList(char** data, int num);

char* ModuleStateCreateNewJsonStr();

void debug_print();

#endif  // __MODULE_OFFLINE_DETECTION_H_
