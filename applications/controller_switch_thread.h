#ifndef __CONTROLLER_SWITCH_THREAD_H_
#define __CONTROLLER_SWITCH_THREAD_H_

#include "stm32f4xx.h"
#include <rtthread.h>
#include "controller_switch_def.h"

struct ControlMsg {
    struct RobotCommMotion robot_common_motion;
    struct RovControl rov_control;
    struct ShipControl ship_control;
    struct EquipSwitch equip_switch;
//    struct MecanumControl macanum_control;
};

void reset_control_msg(struct ControlMsg* msg);
const struct ControlMsg* get_control_msg();
struct ControlMsg* get_control_msg_to_change();
void ControllerNewMsgUpdate(const uint32_t id);
rt_thread_t* get_controller_switch_tid();
void ControllerRecvCntProcessThreadEntry(void* parameter);

#endif  // __CONTROLLER_SWITCH_THREAD_H_
