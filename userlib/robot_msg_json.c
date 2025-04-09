#include "robot_msg_json.h"
#include <rtthread.h>
#include <string.h>
#include "finsh.h"

#define LOG_TAG     "r_m_j"
#define LOG_LVL     LOG_LVL_INFO
#include <ulog.h>

static rt_mutex_t robot_msg_json_root_mutex_ = RT_NULL;

// 头节点的type, 记录了有多少节点
struct RobotMsgJsonLinkNode robot_msg_json_root
    = {{NULL, 0, NULL, J_P_RO}, NULL};

void RobotMsgJsonInit()
{
    robot_msg_json_root_mutex_ = rt_mutex_create("rMSGj_l", RT_IPC_FLAG_PRIO);
    if(robot_msg_json_root_mutex_ == NULL)
        LOG_E("Memory poor!!!");
}

static void RobotMsgJsonInsert(struct RobotMsgJsonLinkNode * root, struct RobotMsgJsonLinkNode * new_node)
{
	struct RobotMsgJsonLinkNode* node = root;
	while (node->next != NULL) {
		node = node->next;
	}
	new_node->next = node->next;
	node->next = new_node;
	root->data.type ++;
}

void RobotMsgJsonRegister(const char* name, int type, const void* data, int permission)
{
    struct RobotMsgJsonLinkNode* new_link_node = rt_malloc(sizeof(struct RobotMsgJsonLinkNode));
	new_link_node->data.string = name;
	new_link_node->data.type = type;
	new_link_node->data.data = (void*)data;
	new_link_node->data.permission = permission;
    if(RT_EOK == rt_mutex_take(robot_msg_json_root_mutex_, RT_WAITING_FOREVER)){
        RobotMsgJsonInsert(&robot_msg_json_root, new_link_node);  // 插入至命令链表
        rt_mutex_release(robot_msg_json_root_mutex_);
    }
}

// boolean | 0.0：True; else: False
// int     | data=(int)data
// float   | data=data
static int RobotMsgJsonModifyData(struct RobotMsgJsonLinkNode * root, const char* name, float data)
{
    struct RobotMsgJsonLinkNode* node = root;
    while (node->next != NULL) {
        node = node->next;
        if(rt_strcmp(node->data.string, name) == 0) {
            if(node->data.permission == J_P_RW) {
                switch(node->data.type) {
                case J_T_Bool:
                    (*((int*)node->data.data)) = (data == 0.0 ? 0:1);
                    return node->data.type;
                    break;
                case J_T_NumberInt:
                    (*((int*)node->data.data)) = data;
                    return node->data.type;
                    break;
                case J_T_NumberFloat:
                    (*((float*)node->data.data)) = data;
                    return node->data.type;
                    break;
                }
            }
        }
    }
    return -1;
}

static void RobotMsgJsonPrintSingle(struct RobotMsgJson* r)
{
    rt_kprintf("name:%s \t type: %d \t ptr: %d \t permit: %d\r\n", r->string, r->type, r->data, r->permission);
}

static void RobotMsgJsonLinkListPrint(struct RobotMsgJsonLinkNode* root)
{
	struct RobotMsgJsonLinkNode* node = root;
	rt_kprintf("RobotMsgJson\tnum: %d\r\n", node->data.type);
	while (node->next != NULL) {
		node = node->next;
		RobotMsgJsonPrintSingle(&node->data);
	}
}

void r_msg_print()
{
    if(RT_EOK == rt_mutex_take(robot_msg_json_root_mutex_, RT_WAITING_FOREVER)){
        RobotMsgJsonLinkListPrint(&robot_msg_json_root);
        rt_mutex_release(robot_msg_json_root_mutex_);
    }
}

static int RobotMsgCJsonAddFromStructure(cJSON* cjson, const struct RobotMsgJson* s)
{
    if(cjson == NULL) return -1;
    if(cJSON_GetObjectItem(cjson, s->string) != NULL) return -2;

    switch(s->type) {
    case J_T_Bool:
        cJSON_AddBoolToObject(cjson, s->string, *((int*)s->data)>0 ? ((cJSON_bool)1):((cJSON_bool)0));
        break;
    case J_T_NumberInt:
        cJSON_AddNumberToObject(cjson, s->string, (*((int*)s->data)));
        break;
    case J_T_NumberFloat:
        cJSON_AddNumberToObject(cjson, s->string, (*((float*)s->data)));
        break;
    case J_T_String:
        cJSON_AddStringToObject(cjson, s->string, s->data);
        break;
    case J_T_Invalid:
    case J_T_NULL:
    default:
        cJSON_AddNullToObject(cjson, s->string);
        break;
    }
    return 0;
}

static void RobotMsgCJsonInit(cJSON* cjson)
{
    if(cjson == NULL) return;
    cJSON_AddStringToObject(cjson, "type", "robot_data");
    cJSON_AddStringToObject(cjson, "format", "json_v1");
    cJSON_AddObjectToObject(cjson, "data");
}

static void RobotMsgCJsonUpdate(cJSON* cjson, struct RobotMsgJsonLinkNode* root)
{
    if(cjson == NULL || root == NULL) return;
    struct RobotMsgJsonLinkNode *node = root;
    while (node->next != NULL) {
        node = node->next;
        RobotMsgCJsonAddFromStructure(cjson, &node->data);
    }
}

cJSON* RobotMsgCreateNewCJson()
{
    cJSON* cjson = cJSON_CreateObject();
    if(cjson == NULL) return cjson;
    RobotMsgCJsonInit(cjson);
    if(RT_EOK == rt_mutex_take(robot_msg_json_root_mutex_, RT_WAITING_FOREVER)){
        RobotMsgCJsonUpdate(cJSON_GetObjectItem(cjson, "data"), &robot_msg_json_root);
        rt_mutex_release(robot_msg_json_root_mutex_);
    }
    return cjson;
}

// back str need be free
char* RobotMsgCreateNewJsonStr()
{
    cJSON* cjson = RobotMsgCreateNewCJson();
    if(cjson == NULL) return NULL;
    char* j_str = cJSON_PrintUnformatted(cjson);
    cJSON_Delete(cjson);
    if(j_str == NULL) return NULL;
    return j_str;
}

//{
//    "type": "modify_robot_data",
//    "format": "json_v1",
//    "data": {
//        "xxx": 0
//    }
//}

int RobotMsgModifyDataByJsonStr(const char* json_str)
{
    if(json_str == NULL) return -1;
    int err = 0;
    cJSON* cjson = cJSON_Parse(json_str);
    cJSON* type = cJSON_GetObjectItem(cjson,"type");
    if(type->type != cJSON_String || rt_strcmp(type->valuestring, "modify_robot_data")) {err = -2; goto END;}
    cJSON* data = cJSON_GetObjectItem(cjson,"data");
    if(data->type != cJSON_Object) {err = -3; goto END;}

    cJSON* data_item = NULL;
    cJSON_ArrayForEach(data_item, data) {
        if (cJSON_IsNumber(data_item)) {
            int r = RobotMsgJsonModifyData(&robot_msg_json_root, data_item->string, cJSON_GetNumberValue(data_item));
            if(r < 0) err = -4;
            LOG_D("key:%s Number: %f\n", data_item->string, cJSON_GetNumberValue(data_item));
        } else if (cJSON_IsBool(data_item)) {
            int r = RobotMsgJsonModifyData(&robot_msg_json_root, data_item->string, cJSON_IsTrue(data_item) ? 1.0f : 0.0f);
            if(r < 0) err = -5;
            LOG_D("key:%s Bool: %s\n", data_item->string, cJSON_IsTrue(data_item) ? "true" : "false");
        }
    }

END:
    cJSON_Delete(cjson);
    return err;
}

//{
//    "type": "response",
//    "format": "json_v1",
//    "response_to": "modify_robot_data",
//    "success": true,
//    "error_message": ""
//}

char* GetRobotMsgModifyDataResponseStr(int success, const char* err_msg)
{
    cJSON* cjson = cJSON_CreateObject();
    cJSON_AddStringToObject(cjson, "type", "response");
    cJSON_AddStringToObject(cjson, "format", "json_v1");
    cJSON_AddStringToObject(cjson, "response_to", "modify_robot_data");
    cJSON_AddBoolToObject(cjson, "success", success>0 ? ((cJSON_bool)1):((cJSON_bool)0));
    cJSON_AddStringToObject(cjson, "error_message", err_msg != NULL ? err_msg : "");
    char* ret_str = cJSON_PrintUnformatted(cjson);
    cJSON_Delete(cjson);
    return ret_str;
}

void r_msg_json_print()
{
    cJSON* cjson = RobotMsgCreateNewCJson();
    if(cjson == NULL) return;
    rt_kprintf("Robot Msg Json:\r\n");
    rt_kprintf("--start--\r\n");
    // rt_kprintf("%s\r\n", cJSON_PrintUnformatted(cjson));
    rt_kprintf("%s\r\n", cJSON_Print(cjson));
    rt_kprintf("--end--\r\n");
    cJSON_Delete(cjson);
}

MSH_CMD_EXPORT(r_msg_print, robot msg ll print);
MSH_CMD_EXPORT(r_msg_json_print, robot msg json print);
