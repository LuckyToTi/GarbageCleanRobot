#ifndef __CONTROLLER_SWITCH_DEF_H_
#define __CONTROLLER_SWITCH_DEF_H_

#include <string.h>
#include "userlib/math2.h"
#include "userlib/robolink.h"
#include "userlib/sbus_rc.h"
#include "robolink_udp_task.h"
#include <stdbool.h>

#define CONTROLLER_TYPE_NUM (6u)

enum CONTROLLER_ID_
{
    C_UART_RC_FORCE = 0,
    C_UDP_ROBOLINK_GAMEPAD,
    C_UDP_ROBOLINK_CONTROLLER_BOX2023,
    C_UDP_ROBOLINK_GAMEPAD_WEB,
    C_UART_ROBOLINK_GAMEPAD,
    C_UART_ROBOLINK_CONTROLLER_BOX2023
};

struct RobotCommMotion {
    // 右手坐标系
    // -1000-1000
    float x;
    float y;
    float z;
    float revolve_x;
    float revolve_y;
    float revolve_z;
};

static inline void reset_robot_comm_motion(struct RobotCommMotion* robot_motion)
{
    robot_motion->x = 0.0; robot_motion->y = 0.0; robot_motion->z = 0.0;
    robot_motion->revolve_x = 0.0; robot_motion->revolve_y = 0.0; robot_motion->revolve_z = 0.0;
}

struct RovControl {
    float z_force;  // -1000-1000
    int roll_hold;  // 0 1
    int pitch_hold;  // 0 1
    int yaw_hold;  // 0 1
    int depth_hold;  // 0 1
    int position_hold;  // 0 1
    int line_hold;  // 0 1
    int position_head_move; // 0 1
    int gear_control_mode; // 0.摄像头 1.云台
    int attitude_type; // 清洗机器人翻转姿态
    int reset_knob;
    float led_brightness;  // 0-100
    float camera_pitch;  // -50 - 50
    float cloud_platform_revolve;  // -50 - 50
    float cloud_platform_pitch;  // -50 - 50
    float gripper_angle;  // 0 - 100 机械手
    float gripper_speed;  // -50 - 50 机械手

};

struct EquipSwitch {
    int sonar;
};

static inline void reset_rov_control(struct RovControl* rov_device)
{
    rov_device->z_force = 0.0; rov_device->roll_hold = 0; rov_device->pitch_hold = 0; rov_device->yaw_hold = 0; rov_device->depth_hold = 0;
    rov_device->position_hold = 0; rov_device->line_hold = 0; rov_device->gear_control_mode = 0;
    rov_device->cloud_platform_revolve = 0.0; rov_device->cloud_platform_pitch = 0.0;
    rov_device->led_brightness = 0.0; rov_device->camera_pitch = 0.0; rov_device->gripper_angle = 0.0; rov_device->gripper_speed = 0.0;
}

struct ShipControl {
    int roll_hold;  // 0 1
    float led_brightness;  // 0-100
    float camera_pitch;  // -50 - 50

};

static inline void reset_ship_control(struct ShipControl* ship_device)
{
    ship_device->roll_hold = 0;
    ship_device->led_brightness = 0;
    ship_device->camera_pitch = 0;
}

static inline void sbus_to_robot_comm_motion(const struct SbusRc* sbus, struct RobotCommMotion* robot_motion)
{
    robot_motion->y = -RockerRawDataToRockerData_Float(sbus->ch[2], 200, 1000, 1800, 1000, 50);
    robot_motion->x = -RockerRawDataToRockerData_Float(sbus->ch[3], 200, 1000, 1800, 1000, 50);
    robot_motion->revolve_z = -RockerRawDataToRockerData_Float(sbus->ch[0], 200, 1000, 1800, 1000, 50);
}


static inline void gamepad_to_robot_comm_motion(const struct ControllerGamepad* gamepad, struct RobotCommMotion* robot_motion)
{
    robot_motion->x = -RockerRawDataToRockerData_Float(gamepad->dwYpos, 0, 32767.5, 65535, 1000, 3000);
    robot_motion->y = -RockerRawDataToRockerData_Float(gamepad->dwXpos, 0, 32767.5, 65535, 1000, 3000);
    robot_motion->z = -RockerRawDataToRockerData_Float(gamepad->dwZpos, 128, 32767.5, 65408, 1000, 3000*2);  // XBOX手柄
    robot_motion->revolve_z = -RockerRawDataToRockerData_Float(gamepad->dwUpos, 128, 32767.5, 65535, 1000, 3000);
}

static inline void box_2023_to_robot_comm_motion(const struct ControllerBox2023* controllerBox, struct RobotCommMotion* robot_motion)
{
    robot_motion->y = -RockerRawDataToRockerData_Float(controllerBox->x_pos, 0, 32767.5, 65535, 1000, 3000);
    robot_motion->x = RockerRawDataToRockerData_Float(controllerBox->y_pos, 0, 32767.5, 65535, 1000, 3000);
    robot_motion->z = RockerRawDataToRockerData_Float(controllerBox->knob1[0], 0, 32767.5, 65535, 1000, 3000);
    robot_motion->revolve_z = -RockerRawDataToRockerData_Float(controllerBox->r_pos, 0, 32767.5, 65535, 1000, 3000);
}

static inline void gamepad_web_to_robot_comm_motion(const struct ControllerGamepadWeb* gamepad, struct RobotCommMotion* robot_motion)
{
    robot_motion->y = -RockerRawDataToRockerData_Float(gamepad->axes[1], 0, 32767.5, 65535, 1000, 3000);
    robot_motion->x = -RockerRawDataToRockerData_Float(gamepad->axes[0], 0, 32767.5, 65535, 1000, 3000);
    robot_motion->z =  RockerRawDataToRockerData_Float(-gamepad->button6 + gamepad->button7 + 65535, 0, 65535, 65535*2, 1000, 3000*2);
    robot_motion->revolve_z = RockerRawDataToRockerData_Float(gamepad->axes[2], 0, 32767.5, 65535, 1000, 3000);
}

static inline void gamepad_to_rov_control(const struct ControllerGamepad* gamepad, const struct ControllerGamepad* last_gamepad, struct RovControl* rov_device)
{
    static int auxiliary_level = 0;  // 辅助驾驶等级
    rov_device->z_force = -RockerRawDataToRockerData_Float(gamepad->dwRpos, 0, 32767.5, 65535, 1000, 3000);

    // 辅助航行控制
    int auxiliary_level_changed = 0;
    if(auxiliary_level > 0 && XBOX_IS_PRESSED_BTN(last_gamepad, gamepad, XBOX_CTRL_LB)) {
        auxiliary_level--;
        auxiliary_level_changed = 1;
    }
    if(auxiliary_level < 3 && XBOX_IS_PRESSED_BTN(last_gamepad, gamepad, XBOX_CTRL_RB)) {
        auxiliary_level++;
        auxiliary_level_changed = 1;
    }
    if(auxiliary_level_changed) {
        switch(auxiliary_level) {
            case 0: rov_device->roll_hold = 0; rov_device->pitch_hold = 0; rov_device->yaw_hold = 0; rov_device->line_hold = 0; break;
            case 1: rov_device->roll_hold = 1; rov_device->pitch_hold = 1; rov_device->yaw_hold = 0; rov_device->line_hold = 0; break;
            case 2: rov_device->roll_hold = 1; rov_device->pitch_hold = 1; rov_device->yaw_hold = 1; rov_device->line_hold = 0; break;
            case 3: rov_device->roll_hold = 1; rov_device->pitch_hold = 1; rov_device->yaw_hold = 1; rov_device->line_hold = 1; break;
            default: break;
        }
        print_rl_udp(0x02, "ROV auxiliary level change to %d.", auxiliary_level);
    }

    if(rov_device->gear_control_mode == 0){
        // 摄像头俯仰
        if(rov_device->camera_pitch < 50.0) {
            if(XBOX_IS_PRESS_POV(gamepad, XBOX_CTRL_UP)) {
                rov_device->camera_pitch += 1.0;
            }
        }
        if(rov_device->camera_pitch > -50.0) {
            if(XBOX_IS_PRESS_POV(gamepad, XBOX_CTRL_DOWN)) {
                rov_device->camera_pitch -= 1.0;
            }
        }
    }else {
        // 云台俯仰
//        if(rov_device->cloud_platform_pitch < 50.0) {
        if(rov_device->cloud_platform_pitch < 25.0) { // 大黄特供
            if(XBOX_IS_PRESS_POV(gamepad, XBOX_CTRL_UP) && fabs(rov_device->cloud_platform_revolve) < 5) {
                rov_device->cloud_platform_pitch += 1.0;
            }
        }
        if(rov_device->cloud_platform_pitch > -25.0) {
            if(XBOX_IS_PRESS_POV(gamepad, XBOX_CTRL_DOWN) && fabs(rov_device->cloud_platform_revolve) < 5) {
                rov_device->cloud_platform_pitch -= 1.0;
            }
        }

        // 云台左右
        if(rov_device->cloud_platform_revolve < 25.0) {
            if(XBOX_IS_PRESS_POV(gamepad, XBOX_CTRL_LEFT) && fabs(rov_device->cloud_platform_pitch) < 5) {
                rov_device->cloud_platform_revolve += 1.0;
            }
        }
        if(rov_device->cloud_platform_revolve > -25.0) {
            if(XBOX_IS_PRESS_POV(gamepad, XBOX_CTRL_RIGHT) && fabs(rov_device->cloud_platform_pitch) < 5) {
                rov_device->cloud_platform_revolve -= 1.0;
            }
        }
//        if(fabs(rov_device->cloud_platform_pitch) > 5){
//            rov_device->cloud_platform_revolve = 0;
//        }
//        if(fabs(rov_device->cloud_platform_revolve) > 5){
//            rov_device->cloud_platform_pitch = 0;
//        }

    }

    // 定深开关
    if(XBOX_IS_PRESSED_BTN(last_gamepad, gamepad, XBOX_CTRL_A)) {
        if(rov_device->depth_hold == 0) {
            rov_device->depth_hold = 1;
        } else {
            rov_device->depth_hold = 0;
        }
        print_rl_udp(0x02, "ROV depth_hold change to %d.", rov_device->depth_hold);
    }

    // 动力定位开关
    if(XBOX_IS_PRESSED_BTN(last_gamepad, gamepad, XBOX_CTRL_B)) {
        if(rov_device->position_hold == 0) {
            rov_device->position_hold = 1;
        } else {
            rov_device->position_hold = 0;
        }
        print_rl_udp(0x02, "ROV position_hold change to %d.", rov_device->position_hold);
    }

    // 定深开关
    if(XBOX_IS_PRESSED_BTN(last_gamepad, gamepad, XBOX_CTRL_X)) {
        if(rov_device->gear_control_mode == 0) {
            rov_device->gear_control_mode = 1;
        } else {
            rov_device->gear_control_mode = 0;
        }
        print_rl_udp(0x02, "ROV gear_control_mode change to %d.", rov_device->gear_control_mode);
    }

    // 水下灯开关
    if(XBOX_IS_PRESSED_BTN(last_gamepad, gamepad, XBOX_CTRL_Y)) {
        if(rov_device->led_brightness == 0) {
            rov_device->led_brightness = 100;
        } else {
            rov_device->led_brightness = 0;
        }
        print_rl_udp(0x02, "ROV LED brightness change to %.1f.", rov_device->led_brightness);
    }

    // 机械手开合
//    if(XBOX_IS_PRESS_POV(gamepad, XBOX_CTRL_RIGHT)) {
//        if(rov_device->gripper_angle < 100.0) {
//            rov_device->gripper_angle += 1.0;
//        }
//        rov_device->gripper_speed = 50.0;
//    } else if(XBOX_IS_PRESS_POV(gamepad, XBOX_CTRL_LEFT)) {
//        if(rov_device->gripper_angle > 0.0) {
//            rov_device->gripper_angle -= 1.0;
//        }
//        rov_device->gripper_speed = -50.0;
//    } else {
//        rov_device->gripper_speed = 0.0;
//    }
}

static inline void sbus_to_ship_control(const struct SbusRc* sbus, const struct SbusRc* last_sbus, struct RovControl* rov_device)
{
    // TODO: ...
}

static inline void box_2023_to_rov_control(const struct ControllerBox2023* controller_box, struct RovControl* rov_device)
{
//   static int knob2_flag_z_force_pitch = 0;
//   static int knob2_flag_camera_pitch = 0;
//   static int knob2_flag_3 = 0;
//   static int knob2_flag_4 = 0;
//   static int knob2_flag_5 = 0;

    switch(controller_box->on_off[0]) {
    case 0:
        rov_device->yaw_hold = 0;
        rov_device->line_hold = 0;
        break;
    case 65534:
        rov_device->yaw_hold = 1;
        rov_device->line_hold = 1;
        break;
    default:
        rov_device->yaw_hold = 1;
        rov_device->line_hold = 0;
        break;
    }

    if(controller_box->on_off[1] == 0) {
        rov_device->pitch_hold = 0;
        rov_device->roll_hold = 0;
        rov_device->depth_hold = 0;
    } else {
        rov_device->pitch_hold = 1;
        rov_device->roll_hold = 1;
        rov_device->depth_hold = 1;
    }

    if(controller_box->on_off[2] == 0) {
        rov_device->position_hold = 0;
    } else {
        rov_device->position_hold = 1;
    }

//    if(controller_box->knob1[3] == 65535){
//        rov_device->gripper_speed = 1;
//    }else{
//        rov_device->gripper_speed = -1;
//    }

//    rov_device->z_force = RockerRawDataToRockerData_Float(controller_box->u_pos, 0, 32767.5, 65535, 1000, 3000);
//    rov_device->cloud_platform_revolve = RockerRawDataToRockerData_Float(controller_box->knob1[2], 0, 32767.5, 65535, 50, 1000);
//    rov_device->cloud_platform_pitch = RockerRawDataToRockerData_Float(controller_box->knob1[3], 0, 32767.5, 65535, 50, 1000);
    rov_device->reset_knob = RockerRawDataToRockerData_Float(controller_box->knob1[1], 0, 32767.5, 65535, 50, 1000) > 30 ? 1 : 0;

    rov_device->cloud_platform_revolve = RockerRawDataToRockerData_Float(controller_box->knob1[2], 0, 32767.5, 65535, 50, 1000);
    rov_device->cloud_platform_pitch = RockerRawDataToRockerData_Float(controller_box->knob1[3], 0, 32767.5, 65535, 50, 1000);
    rov_device->cloud_platform_pitch = rov_device->cloud_platform_pitch > 20 ? 20 : rov_device->cloud_platform_pitch;
    if(fabs(rov_device->cloud_platform_pitch) > 5){
        rov_device->cloud_platform_revolve = 0;
    }
    if(fabs(rov_device->cloud_platform_revolve) > 5){
        rov_device->cloud_platform_pitch = 0;
    }

    rov_device->led_brightness = controller_box->knob2[0] * 100.0 / 65535; Float_Constrain(&rov_device->led_brightness, 0, 100);
    rov_device->camera_pitch = RockerRawDataToRockerData_Float(controller_box->knob1[4], 0, 32767.5, 65535, 50, 1000);

//    MedianConfirmation(&rov_device->gripper_angle, &knob2_flag_z_force_pitch, 0, 2);
//    MedianConfirmation(&rov_device->camera_pitch, &knob2_flag_camera_pitch, 0, 2);
//    MedianConfirmation(&rov_device->z_force, &knob2_flag_2);
//    MedianConfirmation(&rov_device->gripper_angle, &knob2_flag_3);
//    MedianConfirmation(&rov_device->reset_knob, &knob2_flag_4);
}
static inline void gamepad_web_to_ship_control(const struct ControllerGamepadWeb* gamepad, const struct ControllerGamepadWeb* last_gamepad, struct ShipControl* ship_device)
{
}

#endif  // __CONTROLLER_SWITCH_DEF_H_
