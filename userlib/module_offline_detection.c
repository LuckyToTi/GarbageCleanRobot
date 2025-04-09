#include "module_offline_detection.h"
#include <string.h>
#include "cJSON.h"

struct MonitoredModuleNode {
    struct MonitoredModule* node;
    struct MonitoredModuleNode* next;
};

static struct MonitoredModuleNode root_ = {0, NULL};

static rt_mutex_t monitored_module_root_mutex_ = RT_NULL;

#define _MM_LL_LOCK(dosth) \
    do { \
        if (RT_EOK == rt_mutex_take(monitored_module_root_mutex_, RT_WAITING_FOREVER)) { \
            dosth; \
            rt_mutex_release(monitored_module_root_mutex_); \
        } \
    } while (0)

/* --- 定义 _SCOPED_LOCK()保护作用域内的代码o --- */

// 定义一个结构用于在作用域结束时自动释放锁
typedef struct {
    rt_mutex_t mutex;
} __scoped_mutex_t;

// 定义一个清理函数用于在作用域结束时释放锁
void __mutex_cleanup(__scoped_mutex_t *scoped_mutex) {
    rt_mutex_release(scoped_mutex->mutex);
}

// 定义一个宏来初始化 scoped_mutex_t 并锁定互斥锁
#define _SCOPED_LOCK(mutex) __scoped_mutex_t _scoped_lock __attribute__((cleanup(__mutex_cleanup))) = { mutex }; rt_mutex_take(mutex, RT_WAITING_FOREVER)

#define _MM_LL_LOCK_BELOW() _SCOPED_LOCK(monitored_module_root_mutex_)

void MonitoredModuleInit()
{
    if(monitored_module_root_mutex_ == NULL)
        monitored_module_root_mutex_ = rt_mutex_create("mMdN_m", RT_IPC_FLAG_PRIO);
    if(monitored_module_root_mutex_ == NULL)
        rt_kprintf("memory poor!\r\n");
}

static struct MonitoredModule* CreateNewMonitoredModule(char* name, uint8_t mode,
        int32_t check_interval_ms, int32_t max_timeout_times, void (*state_changed_func)(struct MonitoredModule*))
{
    struct MonitoredModule* new_module = rt_malloc(sizeof(struct MonitoredModule));
    if(new_module == RT_NULL) return RT_NULL;
    new_module->name = rt_malloc(rt_strlen(name)+1);
    if(new_module->name == RT_NULL) {
        rt_free(new_module);
        return RT_NULL;
    }
    new_module->name[rt_strlen(name)] = '\0';
    rt_memcpy(new_module->name, name, rt_strlen(name));
    new_module->mode = (mode>0 ? 1:0);
    new_module->check_interval = check_interval_ms / 10;
    if(new_module->check_interval < 1) new_module->check_interval = 1;
    if(new_module->check_interval > MAX_CHECK_INTERVAL_10MS) new_module->check_interval = MAX_CHECK_INTERVAL_10MS;
    new_module->max_timeout_times = max_timeout_times;
    new_module->state_changed_func = state_changed_func;
    new_module->is_reload = 0;
    new_module->state = 0;
    new_module->current_timeout_count = 0;
    return new_module;
}

static void DeleteMonitoredModule(struct MonitoredModule* module)
{
    if(module == NULL) return;
    if(module->name != NULL)
        rt_free(module->name);
    rt_free(module);
    module = NULL;
}

struct MonitoredModule* RegisterNewMonitoredModule(char* name, uint8_t mode,
        int32_t check_interval_ms, int32_t max_timeout_times, void (*state_changed_func)(struct MonitoredModule*))
{
    struct MonitoredModule* new_module = CreateNewMonitoredModule(name, mode, check_interval_ms, max_timeout_times, state_changed_func);
    if(new_module == RT_NULL) return RT_NULL;
    struct MonitoredModuleNode* new_node = rt_malloc(sizeof(struct MonitoredModuleNode));
    if(new_node == RT_NULL) {
        rt_free(new_module);
        return RT_NULL;
    }
    new_node->node = new_module;
    new_node->next = RT_NULL;

    _MM_LL_LOCK_BELOW();
    struct MonitoredModuleNode* node = &root_;
    while (node->next != NULL) {
        node = node->next;
    }
    node->next = new_node;
    (*((int*)(&root_.node))) ++;
    return new_module;
}

static void DeleteNextMonitoredModuleNode(struct MonitoredModuleNode* node)
{
    if(node == NULL) return;
    struct MonitoredModuleNode* next_node = node->next;
    if(next_node == NULL) return;
    node->next = next_node->next;
    DeleteMonitoredModule(next_node->node);
    rt_free(next_node);
}

int UnregisterMonitoredModuleByPtr(struct MonitoredModule* module)
{
    struct MonitoredModuleNode* node = &root_;
    int ret = -1;
    _MM_LL_LOCK_BELOW();
    while (node->next != NULL) {
        struct MonitoredModuleNode* previous_node = node;
        node = node->next;
        if(module == node->node) {
            DeleteNextMonitoredModuleNode(previous_node);
            ret = 0;
            (*((int*)(&root_.node))) --;
        }
    }
    return ret;
}

int UnregisterMonitoredModuleByName(char* name)
{
    struct MonitoredModuleNode* node = &root_;
    int ret = -1;
    _MM_LL_LOCK_BELOW();
    while (node->next != NULL) {
        struct MonitoredModuleNode* previous_node = node;
        node = node->next;
        if(rt_strcmp(name, node->node->name)) {
            DeleteNextMonitoredModuleNode(previous_node);
            ret = 0;
            (*((int*)(&root_.node))) --;
        }
    }
    return ret;
}

static uint8_t MonitorSingleModule(struct MonitoredModule* module)
{
    if(module->is_reload > 0) {
        module->current_timeout_count = 0;
        module->is_reload = 0;
        if(module->state != 1) {
            module->state = 1;
            if(module->state_changed_func != NULL)
                module->state_changed_func(module);
        }
    } else {
        if(module->current_timeout_count < module->max_timeout_times) {
            module->current_timeout_count ++;
        }
        if(module->current_timeout_count >= module->max_timeout_times) {
            if(module->state != 0) {
                module->state = 0;
                if(module->state_changed_func != NULL)
                    module->state_changed_func(module);
            }
        }
    }
    return module->state;
}

// 10ms 执行一次
void MonitorAllModules()
{
    static int32_t time = 0;
    time = (time < MAX_CHECK_INTERVAL_10MS-1 ? (time+1) : (0));
    struct MonitoredModuleNode* node = &root_;
    _MM_LL_LOCK_BELOW();
    while (node->next != NULL) {
        node = node->next;
        if(time % node->node->check_interval == 0 && node->node->mode == 0) {
            MonitorSingleModule(node->node);
        }
    }
}

// protect by lock
struct MonitoredModule* FindMonitoredModuleByName(char* name)
{
    struct MonitoredModuleNode* node = &root_;
    _MM_LL_LOCK_BELOW();
    while (node->next != NULL) {
        node = node->next;
        if(rt_strcmp(name, node->node->name)) {
            return node->node;
        }
    }
    return RT_NULL;
}

void MonitoredModuleReload(struct MonitoredModule* module)
{
    if(module != NULL)
        module->is_reload = 1;
}

int32_t MonitoredModuleReloadByName(char* name)
{
    struct MonitoredModule* module = FindMonitoredModuleByName(name);
    if(module == NULL) return -1;
    MonitoredModuleReload(module);
    return 0;
}

void SetMonitoredModuleState(struct MonitoredModule* module, uint8_t state)
{
    if(module == RT_NULL) return;
    if(module->mode == 0) return;
    state = state > 0 ? 1:0;
    if(state != module->state) {
        module->state = state;
        if(module->state_changed_func != NULL)
            module->state_changed_func(module);
    }
}

int32_t SetMonitoredModuleStateByName(char* name, uint8_t state)
{
    struct MonitoredModule* module = FindMonitoredModuleByName(name);
    if(module == NULL) return -1;
    SetMonitoredModuleState(module, state);
    return 0;
}

void GetRobolinkModuleOnlineStatusDataSegmentList(char*** data, int* num)
{
    if((*data) != NULL) return;
    int num_current = 0;
    struct MonitoredModuleNode* node = &root_;
    _MM_LL_LOCK_BELOW();
    (*data) = rt_malloc(((int)node->node) * sizeof(char*));
    if((*data) == NULL) return;
    while (node->next != NULL) {
        node = node->next;
        (*data)[num_current] = NULL;
        int str_len = rt_strlen(node->node->name);
#define MAX_NAME_STR_LEN (100)
        str_len = str_len > MAX_NAME_STR_LEN ? MAX_NAME_STR_LEN : str_len;
        (*data)[num_current] = rt_malloc(sizeof(char) * (str_len + 2));
        if(*data != NULL) {
            ((uint8_t*)((*data)[num_current]))[0] = str_len;
            rt_memcpy((*data)[num_current] + 1, node->node->name, str_len);
            (*data)[num_current][str_len+1] = node->node->state ? (1):(0);
            num_current ++;
        }
    }
    if(((int)root_.node) == num_current)
        *num = num_current;
}

void DeleteRobolinkModuleOnlineStatusDataSegmentList(char** data, int num)
{
    if(data == NULL) return;
    for(int i=0; i<num; i++) {
        if(data[i] != NULL) {
            rt_free(data[i]);
        }
    }
    rt_free(data);
}

static void ModuleStateCJsonInit(cJSON* cjson)
{
    if(cjson == NULL) return;
    cJSON_AddStringToObject(cjson, "type", "module_state");
    cJSON_AddStringToObject(cjson, "format", "json_v1");
    cJSON_AddObjectToObject(cjson, "data");
}

static int ModuleStateCJsonAddFromStructure(cJSON* cjson, const struct MonitoredModule* data)
{
    if(cjson == NULL) return -1;
    if(cJSON_GetObjectItem(cjson, data->name) != NULL) return -2;
    cJSON_AddBoolToObject(cjson, data->name, data->state>0 ? ((cJSON_bool)1):((cJSON_bool)0));
    return 0;
}

static void ModuleStateCJsonUpdate(cJSON* cjson, struct MonitoredModuleNode* root)
{
    if(cjson == NULL || root == NULL) return;
    struct MonitoredModuleNode *node = root;
    while (node->next != NULL) {
        node = node->next;
        ModuleStateCJsonAddFromStructure(cjson, node->node);
    }
}

cJSON* ModuleStateCreateNewCJson()
{
    cJSON* cjson = cJSON_CreateObject();
    if(cjson == NULL) return cjson;
    ModuleStateCJsonInit(cjson);
    _MM_LL_LOCK_BELOW();
    ModuleStateCJsonUpdate(cJSON_GetObjectItem(cjson, "data"), &root_);
    return cjson;
}

char* ModuleStateCreateNewJsonStr()
{
    cJSON* cjson = ModuleStateCreateNewCJson();
    if(cjson == NULL) return NULL;
    char* j_str = cJSON_PrintUnformatted(cjson);
    cJSON_Delete(cjson);
    if(j_str == NULL) return NULL;
    return j_str;
}

void moitored_modules_list()
{
    rt_kprintf("---moitored_modules---start\r\n");
    int len = 0;
    struct MonitoredModuleNode* node = &root_;
    _MM_LL_LOCK(
    while (node->next != NULL) {
        len ++;
        node = node->next;
        rt_kprintf(" %s: \t\t %s\r\n", node->node->name, node->node->state ? ("o"):("x"));
    }
    );
    rt_kprintf("---%d module%s---\r\n", len, (len>1 ? "s":""));
    rt_kprintf("---moitored_modules---end\r\n\r\n");
}

void debug_print()
{
    struct MonitoredModuleNode* node = &root_;
    _MM_LL_LOCK(
    while (node->next != NULL) {
        node = node->next;
        rt_kprintf("name:%s: state:%d mode:%d is_reload:%d check_interval:%d max_timeout_times:%d current_timeout_count:%d state_changed_func:%d\r\n",
                node->node->name, node->node->state, node->node->mode, node->node->is_reload, node->node->check_interval,
                node->node->max_timeout_times, node->node->current_timeout_count, node->node->state_changed_func);
    }
    );
}

MSH_CMD_EXPORT(moitored_modules_list, List the online status of all modules);
