#ifndef CONTROL_H__
#define CONTROL_H__

/* 	control.h
	该文件包含各种机器人配置选项的宏定义，
	用于根据不同的硬件和功能需求进行配置。
 * */

/* PUSHES_OUT_SWITCH: 0.PWM, 1.CAN, 2.VESC_CAN_DUTY
 * PUSHES_SWITCH: 1.6个推进器(4平推 2垂推), 2.8个推进器(4平推 4垂推)
 * USE_IMU_DATA: 1.核心板板载陀螺仪, 2.MTI60
 * EQUIPMENT_CLOUD_PLATFORM: 云台
 * EQUIPMENT_CAMERA: 摄像头舵机
 * EQUIPMENT_LED: 水下灯
 * EQUIPMENT_IRON_HAND: 机械手
 */

#define ROBOT_CONFIG (2) //0.自定义, 1.小蓝, 2.大黄
#if ROBOT_CONFIG == 0
	#define PUSHES_OUT_SWITCH (0)
	#define PUSHES_SWITCH (1)
	#define USE_IMU_DATA (0)
	#define EQUIPMENT_CLOUD_PLATFORM (0)
	#define EQUIPMENT_CAMERA (0)
	#define EQUIPMENT_LED (0)
	#define EQUIPMENT_IRON_HAND (0)
#elif ROBOT_CONFIG == 1
	#define PUSHES_OUT_SWITCH (0)
	#define PUSHES_SWITCH (1)
	#define USE_IMU_DATA (1)
	#define EQUIPMENT_CLOUD_PLATFORM (0)
	#define EQUIPMENT_CAMERA (1)
	#define EQUIPMENT_LED (1)
	#define EQUIPMENT_IRON_HAND (0)
#elif ROBOT_CONFIG == 2
	#define PUSHES_OUT_SWITCH (2)
	#define PUSHES_SWITCH (2)
	#define USE_IMU_DATA (2)
	#define EQUIPMENT_CLOUD_PLATFORM (1)
	#define EQUIPMENT_CAMERA (1)
	#define EQUIPMENT_LED (1)
	#define EQUIPMENT_IRON_HAND (0)
#endif

#endif
