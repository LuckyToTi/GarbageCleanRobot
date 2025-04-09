#ifndef __MECANUM_PUSHES_H__
#define __MECANUM_PUSHES_H__

#include "stm32f4xx.h"
#include "userlib/math2.h"
#include "userlib/pid.h"
#include "config.h"
#include "controller_switch_thread.h"
#include "motion_algorithm.h"

#define WHEELS_OUT_SWITCH (1)  // 0 pwm, 1 can(c610电调)

//struct Targets {
//    float x_tar;
//    float y_tar;
//    float yaw_hold_tar;
//    float line_yaw_tar;
//    float head_move_x_tar;
//    float head_move_y_tar;
//    float head_move_yaw;
//};

//struct Targets* get_move_tar();

// from mecanum_move.c
void MoveAutoAndManualForMecanum(struct MotionSpeed* motion_speed, const struct ControlMsg* ctrl_msg, float T);

// from mecanum_move.c
void WheelsSpeedReset(float * motors, int num);
void WheelsSpeedInvert(float * motors);
void WheelsSpeedConvertToTargetArray(int16_t* motor_array, const float * motors, int num, float mid_value, float min, float max);
void CalcMecanumWheelWheelsSpeed(float * motors, float x, float y, float yaw_z, float rate);
void MotorPwmOut_4wheels(const int16_t* output);

#endif
