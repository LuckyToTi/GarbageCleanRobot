#include <movelib/mecanum_move.h>
#include "config.h"
#include "hardware/timer.h"
#include "math.h"
#include "motion_algorithm.h"

#define LOG_TAG     "mecanum_pushes.c"
#define LOG_LVL     LOG_LVL_INFO
#include <ulog.h>

// 推进器全归零
void WheelsSpeedReset(float * motors, int num) {
    for(int i=0; i<num; i++) {
        motors[i] = 0.0;
    }
}

// 部分取反
void WheelsSpeedInvert(float * motors) {
	for(int _i = 0; _i < 4; _i ++){
		motors[_i] = get_config()->push_invert[_i] == -1 ? (-motors[_i]) : (motors[_i]);
	}
}

void WheelsSpeedConvertToTargetArray(int16_t* motor_array, const float * motors, int num, float mid_value, float min, float max)
{
    for(int i=0; i<num; i++) {
        float motor_value = motors[i] + mid_value;
        Float_Constrain(&motor_value, min, max);
        motor_array[i] = motor_value;
    }
}

void WheelsSpeedToPercent(int16_t* pwms, const float * motors, int num, float mid_value, float min, float max)
{
    for(int i=0; i<num; i++) {
        float motor_pwm = motors[i] + mid_value;
        Float_Constrain(&motor_pwm, min, max);
        pwms[i] = motor_pwm;
    }
}

// 水平方向4个推进器
// +x 前进 ；+y 左平移；+yaw_z 左自旋
void CalcMecanumWheelWheelsSpeed(float * motors, float x, float y, float yaw_z, float rate)
{
    motors[0] += (-x + y + yaw_z) * rate;
    motors[1] += (+x + y + yaw_z) * rate;
    motors[2] += (-x - y + yaw_z) * rate;
    motors[3] += (+x - y + yaw_z) * rate;
}

//  推进器输出
void MotorPwmOut_4wheels(const int16_t* output) {
	PD15_PWM_OUT_US(output[0]);
	PD14_PWM_OUT_US(output[1]);
	PD13_PWM_OUT_US(output[2]);
	PD12_PWM_OUT_US(output[2]);
	PE9_PWM_OUT_US(output[3]);
}
