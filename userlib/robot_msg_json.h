#ifndef __ROBOT_MSG_JSON_H_
#define __ROBOT_MSG_JSON_H_

#include "stdio.h"
#include "cJSON.h"

/* Robot Msg Data Types: */
#define J_T_Invalid     (0)
#define J_T_Bool        (1 << 0)
#define J_T_NULL        (1 << 1)
#define J_T_NumberInt   (1 << 2)
#define J_T_NumberFloat (1 << 3)
#define J_T_String      (1 << 4)

/* Robot Msg Data Permission: */
#define J_P_RO     (0)
#define J_P_RW     (1 << 0)

struct RobotMsgJson {
    /* The item's name string */
    const char *string;

    /* The type of the item, as above. */
    int type;

    /* The item's data point */
    void* data;

    /* The item's data permission */
    int permission;
};

struct RobotMsgJsonLinkNode {
    struct RobotMsgJson data;
    struct RobotMsgJsonLinkNode* next;
};

void RobotMsgJsonInit();
void RobotMsgJsonRegister(const char* name, int type, const void* data, int permission);
cJSON* RobotMsgCreateNewCJson();
char* RobotMsgCreateNewJsonStr();
int RobotMsgModifyDataByJsonStr(const char* json_str);
char* GetRobotMsgModifyDataResponseStr(int success, const char* err_msg);

#endif  // __ROBOT_MSG_JSON_H_
