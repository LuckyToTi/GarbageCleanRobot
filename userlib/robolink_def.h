#ifndef __ROBOLINK_DEF_H_
#define __ROBOLINK_DEF_H_

#include "bytetransformers.h"

#define XBOX_CTRL_A  (1u)
#define XBOX_CTRL_B  (2u)
#define XBOX_CTRL_X  (4u)
#define XBOX_CTRL_Y  (8u)
#define XBOX_CTRL_LB (16u)
#define XBOX_CTRL_RB (32u)
#define XBOX_VIEW    (64u)
#define XBOX_MENU    (128u)
#define XBOX_LS_CLICK (256u)
#define XBOX_RS_CLICK (512u)
#define XBOX_CTRL_UP     (0u)
#define XBOX_CTRL_DOWN   (18000u)
#define XBOX_CTRL_LEFT   (27000u)
#define XBOX_CTRL_RIGHT  (9000u)
#define XBOX_IS_PRESSED_BTN(last, now, btn) ((!(((last)->dwButtons & btn) == btn)) && (((now)->dwButtons & btn) == btn))
#define XBOX_IS_PRESS_POV(now, direction)   ((now)->dwPOV == direction)

struct ControllerGamepad
{
    uint32_t dwXpos;// 左右平移
    uint32_t dwYpos;// 前后平移
    uint32_t dwZpos;
    uint32_t dwRpos;
    uint32_t dwUpos;
    uint32_t dwVpos;
    uint32_t dwButtons;
    uint32_t dwButtonNumber;
    uint32_t dwPOV;
};

struct ControllerBox2023 {
    uint16_t x_pos;
    uint16_t y_pos;
    uint16_t z_pos;
    uint16_t r_pos;
    uint16_t u_pos;
    uint16_t v_pos;
    uint16_t knob1[5];
    uint16_t knob2[3];
    uint16_t on_off[3];
};

struct ControllerGamepadWeb {
    uint16_t axes[4];
    uint16_t buttons;
    uint16_t button6;
    uint16_t button7;
};

static inline void robolink_process_raw_controller_gamepad(uint8_t* raw_data, struct ControllerGamepad* data)
{
    if(raw_data == NULL || data == NULL) return;
    data->dwXpos = get_u32(raw_data + 0, LSB);
    data->dwYpos = get_u32(raw_data + 4, LSB);
    data->dwZpos = get_u32(raw_data + 8, LSB);
    data->dwRpos = get_u32(raw_data + 12, LSB);
    data->dwUpos = get_u32(raw_data + 16, LSB);
    data->dwVpos = get_u32(raw_data + 20, LSB);
    data->dwButtons = get_u32(raw_data + 24, LSB);
    data->dwButtonNumber = get_u32(raw_data + 28, LSB);
    data->dwPOV = get_u32(raw_data + 32, LSB);
}

static inline void robolink_process_raw_controller_box_2023(uint8_t* raw_data, struct ControllerBox2023* data)
{
    if(raw_data == NULL || data == NULL) return;
    data->x_pos = get_u16(raw_data + 0, LSB);
    data->y_pos = get_u16(raw_data + 2, LSB);
    data->z_pos = get_u16(raw_data + 8, LSB);
    data->r_pos = get_u16(raw_data + 6, LSB);
    data->u_pos = get_u16(raw_data + 4, LSB);
    data->v_pos = get_u16(raw_data + 10, LSB);
    data->knob1[0] = get_u16(raw_data + 12, LSB);
    data->knob1[1] = get_u16(raw_data + 14, LSB);
    data->knob1[2] = get_u16(raw_data + 16, LSB);
    data->knob1[3] = get_u16(raw_data + 18, LSB);
    data->knob1[4] = get_u16(raw_data + 20, LSB);
    data->knob2[0] = get_u16(raw_data + 22, LSB);
    data->knob2[1] = get_u16(raw_data + 24, LSB);
    data->knob2[2] = get_u16(raw_data + 26, LSB);
    data->on_off[0] = get_u16(raw_data + 28, LSB);
    data->on_off[1] = get_u16(raw_data + 30, LSB);
    data->on_off[2] = get_u16(raw_data + 32, LSB);
}

static inline void robolink_process_raw_controller_gamepad_web(uint8_t* raw_data, struct ControllerGamepadWeb* data)
{
    if(raw_data == NULL || data == NULL) return;
    data->axes[0] = get_u16(raw_data + 0, LSB);
    data->axes[1] = get_u16(raw_data + 2, LSB);
    data->axes[2] = get_u16(raw_data + 4, LSB);
    data->axes[3] = get_u16(raw_data + 6, LSB);
    data->buttons = get_u16(raw_data + 8, LSB);
    data->button6 = get_u16(raw_data + 10, LSB);
    data->button7 = get_u16(raw_data + 12, LSB);
}

static inline void reset_controller_gamepad(struct ControllerGamepad* data)
{
    data->dwXpos = 32768;
    data->dwYpos = 32768;
    data->dwZpos = 32768;
    data->dwRpos = 32768;
    data->dwUpos = 0;
    data->dwVpos = 0;
    data->dwButtons = 0;
    data->dwButtonNumber = 0;
    data->dwPOV = 65535;
}

static inline void reset_controller_box_2023(struct ControllerBox2023* data)
{
    // TODO: 完善控制器数重置函数
}

static inline void reset_controller_gamepad_web(struct ControllerGamepadWeb* data)
{
    data->axes[0] = 32768;
    data->axes[1] = 32768;
    data->axes[2] = 32768;
    data->axes[3] = 32768;
    data->buttons = 0;
    data->button6 = 0;
    data->button7 = 0;
}

struct RobolinkGetMsg
{
    struct ControllerGamepad controller_gamepad;
    struct ControllerBox2023 controller_box_2023;
    struct ControllerGamepadWeb controller_gamepad_web;
};

#endif  // __ROBOLINK_DEF_H_
