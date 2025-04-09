#include "rov_move_thread.h"
#include "userlib/math2.h"
#include "movelib/rov_move.h"
#include "controller_switch_thread.h"
#include "hardware/motor_control_center.h"


#define LOG_TAG     "r_m_t"
//#define LOG_LVL     LOG_LVL_INFO
#include <ulog.h>

#define POWER_LIMIT 1 // 功率限制
void MotorCanControlSendMotors(int16_t * output);
void MotorVescDutyCanControlSendEightMotors(int16_t* output);
void MotorVescDutyCanControlSendEightMotorsF(float* output);
int16_t map_value(int16_t x, int16_t x_min, int16_t x_max, int16_t x0, int16_t dead_zone);

rt_thread_t rov_move_tid = RT_NULL; //水下机器人线程控制块
inline rt_thread_t* get_rov_move_tid() {
    return &rov_move_tid;
}

void RovThreadEntry(void* parameter) {

	rt_thread_delay(1100);
	for (;;) {
	    struct MotionSpeed motion_speed = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
		#if defined(PUSHES_SWITCH) && PUSHES_SWITCH==1
			float motor_speed[6] = { 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f };
        #elif defined(PUSHES_SWITCH) && PUSHES_SWITCH==2
            float motor_speed[8] = { 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
		#endif

		#if defined(PUSHES_OUT_SWITCH) && PUSHES_SWITCH==1
			int16_t motor_speed_pwm_output[6] = { 0, 0, 0, 0, 0, 0};
		#elif defined(PUSHES_OUT_SWITCH) && PUSHES_SWITCH==2
			int16_t motor_speed_pwm_output[8] = { 0, 0, 0, 0, 0, 0, 0, 0};
		#endif

		MoveAutoAndManual(&motion_speed, get_control_msg(), 20.0f/1000);

		// x y z ... -> 电机速度
		CalcHorizontalDirectionMotorsSpeed_4pushes(motor_speed, motion_speed.x, motion_speed.y, motion_speed.yaw_z, 1.0);

		// 垂推转换
		#if defined(PUSHES_SWITCH) && PUSHES_SWITCH==1
			CalcVerticalDirectionMotorsSpeed_2pushes(motor_speed, motion_speed.z, motion_speed.roll_x, 1.0);
		#elif defined(PUSHES_SWITCH) && PUSHES_SWITCH==2
			CalcVerticalDirectionMotorsSpeed_4pushes(motor_speed, motion_speed.z, motion_speed.roll_x,motion_speed.pitch_y, 1.0);
		#endif

		ScaleDownDataF(motor_speed+0, 1000, 4);
		ScaleDownDataF(motor_speed+4, 1000, PUSHES_SWITCH * 2);
		MotorsSpeedInvertAndLimit(motor_speed, (float)get_config_future()->push2_invert[2]/10);

		#if PUSHES_OUT_SWITCH == 1
			MotorsSpeedConvertToTargetArray(motor_speed_pwm_output, motor_speed, 4 + PUSHES_SWITCH * 2, 1500, 1000, 2000);
			for(int i = 0; i < PUSHES_SWITCH * 2; i++){
			    motor_speed_pwm_output[i] = map_value(motor_speed_pwm_output[i], 1000, 2000, 1500, get_config_future()->push2_invert[0]);
			}
			MotorCanControlSendMotors(motor_speed_pwm_output);
		#elif PUSHES_OUT_SWITCH == 2
			for(int i=0; i<8; i++) {
			    motor_speed[i] *= (100000 / 1000.0);
			}
			MotorsSpeedConvertToTargetArray(motor_speed_pwm_output, motor_speed, 4 + PUSHES_SWITCH * 2, 0, -100000, 100000);
			MotorVescDutyCanControlSendEightMotorsF(motor_speed);
		#else
			MotorsSpeedConvertToTargetArray(motor_speed_pwm_output, motor_speed, 4 + PUSHES_SWITCH * 2, 1500, 1000, 2000);
        	MotorPwmOut_6pushes(motor_speed_pwm_output);
		#endif
		rt_thread_delay(20);
	}
}


void MotorCanControlSendMotors(int16_t* output){
	MotorTxDataConvertAndSend(0,output[0],output[1],output[2],3);
	rt_thread_delay(3);
	MotorTxDataConvertAndSend(3,output[3],output[4],output[5],3);
	rt_thread_delay(3);
	#if PUSHES_SWITCH==2
		MotorTxDataConvertAndSend(6,output[6],output[7],0,3);
		rt_thread_delay(3);
	#endif
}

void MotorVescDutyCanControlSendEightMotors(int16_t* output){
	for(int _i = 0; _i < 4 + PUSHES_SWITCH * 2; _i ++){
		MotorVescDutyControlTxDataConvertAndSend(_i, output[_i]);
		rt_thread_delay(3);
	}
}


void MotorVescDutyCanControlSendEightMotorsF(float* output) {
    for(int _i = 0; _i < 4 + PUSHES_SWITCH * 2; _i ++) {
        float f_n = output[_i];
        Float_Constrain(&f_n, -100000, 100000);
        int32_t o = f_n;
        MotorVescDutyControlTxDataConvertAndSend(_i+1, o);
        rt_thread_delay(3);
    }
}

int16_t map_value(int16_t x, int16_t x_min, int16_t x_max, int16_t x0, int16_t dead_zone) {
    // 输入值检查
    if (x < x_min) x = x_min;
    if (x > x_max) x = x_max;

    // 计算死区边界
    float x_dead_zone_min = x0 - dead_zone;
    float x_dead_zone_max = x0 + dead_zone;

    // 计算增益，避免除以零
    float gain_upper = (x_max - x_dead_zone_max) / (x_max - x0 + 1e-6);
    float gain_lower = (x_dead_zone_min - x_min) / (x0 - x_min + 1e-6);

    // 确保增益为非负数
    if (gain_upper < 0) gain_upper = 0;
    if (gain_lower < 0) gain_lower = 0;

    if (x == x0) {
        // 在死区内，输出等于输入
        return x;
    }
    else if (x > x0) {
        // 高于中值，使用上侧增益
        return x0 + dead_zone + (x - x0) * gain_upper;
    }
    else {
        // 低于中值，使用下侧增益
        return x0 - dead_zone - (x0 - x) * gain_lower;
    }
}
