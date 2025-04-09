#ifndef __ROV_PUSHES_H__
#define __ROV_PUSHES_H__

#include "stm32f4xx.h"
#include "userlib/math2.h"
#include "userlib/pid.h"
#include "config.h"
#include "controller_switch_thread.h"
#include "motion_algorithm.h"
#include <control_config.h>

struct Targets {
    float roll_tar;
    float pitch_tar;
    float line_tar;
    float Dvl_x_tar;
    float Dvl_y_tar;
    float depth_fixed_tar;
    float yaw_hold_tar;
    float line_yaw_tar;
    float head_move_x_tar;
    float head_move_y_tar;
    float head_move_yaw;
};

struct Targets* get_move_tar();

// from rov_move.c
void MoveAutoAndManual(struct MotionSpeed* motion_speed, const struct ControlMsg* ctrl_msg, float T);

// from rov_pushes.c
void MotorsSpeedReset(float * motors, int num);
void MotorsSpeedInvertAndLimit(float * motors, float power_limit);
void MotorsSpeedConvertToTargetArray(int16_t* motor_array, const float * motors, int num, float mid_value, float min, float max);
void CalcHorizontalDirectionMotorsSpeed_4pushes(float * motors, float x, float y, float yaw_z, float rate);
void CalcVerticalDirectionMotorsSpeed_2pushes(float * motors, float z, float roll_x, float rate);
void CalcVerticalDirectionMotorsSpeed_4pushes(float * motors, float z, float roll_x, float pitch, float rate);
void MotorPwmOut_6pushes(const int16_t* output);

#endif
