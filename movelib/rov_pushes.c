#include "rov_move.h"
#include "config.h"
#include "hardware/timer.h"
#include "math.h"

#define LOG_TAG     "r_s_p"
#define LOG_LVL     LOG_LVL_INFO
#include <ulog.h>

// 推进器全归零
void MotorsSpeedReset(float * motors, int num)
{
    for(int i=0; i<num; i++) {
        motors[i] = 0.0;
    }
}

// 部分取反及功率限制
void MotorsSpeedInvertAndLimit(float * motors, float power_limit) {
    float power_limit_final = 0;
    power_limit_final = power_limit > 1 ? 1: power_limit;
    power_limit_final = power_limit == 0 ? 0.5: power_limit;

	for(int _i = 0; _i < (4 + PUSHES_SWITCH * 2); _i ++){
		motors[_i] = get_config_future()->push_invert[_i] == -1 ? (-motors[_i] * power_limit_final) : (motors[_i] * power_limit_final);
	}
}

void MotorsSpeedConvertToTargetArray(int16_t* motor_array, const float * motors, int num, float mid_value, float min, float max)
{
    for(int i=0; i<num; i++) {
        float motor_value = motors[i] + mid_value;
        Float_Constrain(&motor_value, min, max);
        motor_array[i] = motor_value;
    }
}

void MotorsSpeedToPercent(int16_t* pwms, const float * motors, int num, float mid_value, float min, float max)
{
    for(int i=0; i<num; i++) {
        float motor_pwm = motors[i] + mid_value;
        Float_Constrain(&motor_pwm, min, max);
        pwms[i] = motor_pwm;
    }
}

// 水平方向4个推进器
// +x 前进 ；+y 做平移；+yaw_z 左自旋
void CalcHorizontalDirectionMotorsSpeed_4pushes(float * motors, float x, float y, float yaw_z, float rate)
{
    motors[0] += (-x + y + yaw_z) * rate;
    motors[1] += (+x + y + yaw_z) * rate;
    motors[2] += (-x - y + yaw_z) * rate;
    motors[3] += (+x - y + yaw_z) * rate;
}

// 垂直方向2个推进器
// +z 上浮；+roll_x 向右横滚
void CalcVerticalDirectionMotorsSpeed_2pushes(float * motors, float z, float roll_x, float rate) {
    motors[4] += (+z + roll_x) * rate;
    motors[5] += (-z + roll_x) * rate;
}

// 垂直方向4个推进器
// +z 上浮；+roll_x 向右横滚  +pitch
void CalcVerticalDirectionMotorsSpeed_4pushes(float * motors, float z, float roll_x, float pitch, float rate) {
    motors[4] += (+z + roll_x - pitch) * rate;
    motors[5] += (-z + roll_x + pitch) * rate;
    motors[6] += (-z - roll_x - pitch) * rate;
    motors[7] += (+z - roll_x + pitch) * rate;
}

//  推进器输出
void MotorPwmOut_6pushes(const int16_t* output) {
	PD15_PWM_OUT_US(output[0]);
	PD14_PWM_OUT_US(output[1]);
//	PD13_PWM_OUT_US(output[2]);
	PD12_PWM_OUT_US(output[2]);
	PE9_PWM_OUT_US(output[3]);
	PE11_PWM_OUT_US(output[4]);
	PE13_PWM_OUT_US(output[5]);
}
