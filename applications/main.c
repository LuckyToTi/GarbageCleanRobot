/*
 * Copyright (c) 2006-2023, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2023-08-17     RT-Thread    first version
 */

#include <rtthread.h>
#include <rthw.h>

// devices / hardware head(.h)
#include "hardware/led.h"
#include "hardware/led_extend.h"
#include "hardware/iic3.h"
#include "hardware/eth.h"
#include "hardware/eeprom.h"
#include "hardware/usart1.h"
#include "hardware/usart2.h"
#include "hardware/usart5.h"
#include "hardware/can1.h"
#include "hardware/timer.h"
#include "hardware/i2c_wd.h"
#include "hardware/adc.h"
#include "hardware/iwdg.h"

//#include "hardware/spi3.h"
//#include "hardware/usart3.h"
//#include "hardware/uart5.h"
//#include "hardware/dac.h"
//#include "hardware/i2c_wd.h"

// app / task head(.h)
#include "gyro_thread.h"
#include "sht30_thread.h"
#include "robolink_udp_task.h"
#include "robolink_usart_task.h"
#include "controller_switch_thread.h"
#include "robot_msg_server_thread.h"
#include "module_offline_detect_thread.h"
#include "altimeter_thread.h"
#include "wd_thread.h"
#include "gnss_thread.h"
#include "rov_move_thread.h"
#include "equipment_thread.h"
#include "can1_thread.h"
#include "dvl_json_tcp_thread.h"
#include "modbus_thread.h"
#include "wit_gnss_thread.h"
#include "xsens_ahrs_thread.h"
#include "yesens_thread.h"
#include "fid_ahrs_thread.h"
#include "hi12h4_ahrs_thread.h"
#include "motor_test_thread.h"
#include "config.h"

// user / userlib head(.h)
#include "userlib/robot_msg_json.h"
#include "userlib/module_offline_detection.h"

void DevicesInit();
void TasksInit();
void TasksConfigInit();
void TaskStart();

//#define NOT_AUTO_START_TELNET_SERVER
#include "userlib/run_in_new_thread.h"
extern void telnet_server(void);
inline void StartTelnetServer(void);

extern int config_get_first_status;
#define IS_SYSTEM_NORMAL_RUNNING() (config_get_first_status == 0)

static const rt_uint8_t min_thread_priority = 31;
static rt_thread_t flash_light_tid = RT_NULL;
static void FlashLightThreadEntry(void* parameter);

#define WATCH_DOG_SWITCH (1) // 看门狗开关 0.关闭 1.开启

int main(void)
{
	RobotMsgJsonInit();
	MonitoredModuleInit();
    DevicesInit();
    TasksInit();
    TasksConfigInit();
    TaskStart();

#ifndef NOT_AUTO_START_TELNET_SERVER
    RunInNewThread(StartTelnetServer, 1024, min_thread_priority, 100);
#endif

    flash_light_tid = rt_thread_create("flashLt", FlashLightThreadEntry, (void*)0, 256, min_thread_priority, 1);
    if (flash_light_tid != RT_NULL) rt_thread_startup(flash_light_tid);

    return RT_EOK;
}

static void FlashLightThreadEntry(void* parameter)
{
    for(;;) {
        rt_thread_mdelay(750);
		#if WATCH_DOG_SWITCH == 1
				HAL_IWDG_Refresh(&hiwdg);
		#endif

        if(IS_SYSTEM_NORMAL_RUNNING()){
            LedRgbWrite('b', !LedRgbRead('b'));
            LedExtendRgbWrite('r', !LedExtendRgbRead('r'));
        } else {
            LedRgbWrite('r', !LedRgbRead('r'));
            LedExtendRgbWrite('g', !LedExtendRgbRead('g'));
        }
   }
}

void DevicesInit()
{
    // pause interrupt
    register rt_base_t temp;
    temp = rt_hw_interrupt_disable();

    LedRgbGpioInit();
    LedExtendRgbGpioInit();
    I2c3BspInit();
    Usart1Init();
    Usart2Init();
    Uart5HalInterfaceInit();
    TimerInit();
    Ms5837_Softi2c_Gpio_Init();
    Can1Init();
    MX_ADC1_Init();
	#if WATCH_DOG_SWITCH == 1
		IWDG_Init();
	#endif
    // resume interrupt
    rt_hw_interrupt_enable(temp);
}

void TasksInit()
{
    *get_robolink_udp_tid() = rt_thread_create("rl_udp", RobolinkUdpThreadEntry, (void*)0, 2048, 10, 10);
    *get_controller_switch_tid() = rt_thread_create("CtRxCtTd", ControllerRecvCntProcessThreadEntry, (void*)0, 256+128, 13, 10);
    *get_robot_msg_server_tid() = rt_thread_create("rMsvr_t", RobotMsgServerThreadEntry, (void*)0, 2048, 26, 5);
    *get_module_offline_detect_tid() = rt_thread_create("mdStM_t", ModuleOfflineDetectThreadEntry, (void*)0, 512+256, 25, 5);
    if(IS_SYSTEM_NORMAL_RUNNING()) {  // 若iic正常
        if(get_config()->u1_config.function != 3)
            *get_gyro_tid() = rt_thread_create("gyro", GyroThreadEntry, (void*)0, 1024, 10, 10);
        *get_sht30_tid() = rt_thread_create("sht30", Sht30ThreadEntry, (void*)0, 512, 15, 10);
    }
    *get_can1_tid() = rt_thread_create("can1_t", Can1ThreadEntry, (void*)0, 2048, 20, 10);
    *get_altimeter_tid() = rt_thread_create("altim...", AltimeterThreadEntry, (void*)0, 1024, 16, 10);
    *get_wd_tid() = rt_thread_create("wd", WaterdepthThreadEntry, (void*)0, 1024, 19, 10);
    *get_rov_move_tid() = rt_thread_create("rov", RovThreadEntry, (void*)0, 2048 + 1024, 10, 10);
    *get_equipment_tid() = rt_thread_create("equip...", EquipmentThreadEntry, (void*)0, 1024 + 1024, 10, 10);
    *get_dvl_json_tcp_tid() = rt_thread_create("dvl_t", DvlJsonTcpThreadEntry, (void*)0, 2048, 10, 10);
    *get_motor_test_tid() = rt_thread_create("motor_test_t", MotorTestThreadEntry, (void*)0, 1024+512, 10, 10);
}

void TasksConfigInit()
{
	// 串口1 1.无线控制  2.sbus 3.xsens
	switch(get_config()->u1_config.function){
		case 1:
			*get_robolink_usart_tid() = rt_thread_create("rl_usart", ControllerUsartThreadEntry, (void*)0, 2048, 10, 10);
			break;
		case 2:
			*get_robolink_usart_tid() = rt_thread_create("rc_sbus", ControllerSbusRcThreadEntry, (void*)0, 2048, 10, 10);
			break;
		case 3:
			*get_xsens_ahrs_tid() = rt_thread_create("xsens", XsensAhrsThreadEntry, (void*)0, 1024, 10, 10);
			break;
		case 4:
		    *get_yesens_tid() = rt_thread_create("yesens", YsensThreadEntry, (void*)0 , 1024, 10, 10);
		    break;
		case 5:
		    *get_fid_ahrs_tid() = rt_thread_create("ahrs", FidThreadEntry, (void*)0 , 1024, 10, 10);
		case 6:
            *get_hi12h4_ahrs_tid() = rt_thread_create("hi14h4", HI12H4AhrsThreadEntry, (void*)0, 1024+512, 10, 10);
            break;
		default:
			break;
	}

	if(get_config()->u4_485_config.function != 0) {
	    *get_modbus_tid() = rt_thread_create("modbus_t", modbus_thread, (void*)0, 2048, 10, 10);
	}

	// 串口5 1.gps 2.wit gps
	switch(get_config()->u5_config.function){
		case 1:
			*get_gnss_tid() = rt_thread_create("gnss", GnssThreadEntry, (void*)0, 2048, 20, 10);
			break;
		case 2:
			*get_wit_gnss_data_tid() = rt_thread_create("wit_gnss", WitGnssThreadEntry, (void*)0, 1024+1024, 16, 10);
			break;
		default:
			break;
	}
}

void TaskStart()
{
    if (*get_robolink_udp_tid() != RT_NULL) rt_thread_startup(*get_robolink_udp_tid());
    if (*get_robolink_usart_tid() != RT_NULL) rt_thread_startup(*get_robolink_usart_tid());
    if (*get_controller_switch_tid() != RT_NULL) rt_thread_startup(*get_controller_switch_tid());
    if (*get_robot_msg_server_tid() != RT_NULL) rt_thread_startup(*get_robot_msg_server_tid());
    if (*get_module_offline_detect_tid() != RT_NULL) rt_thread_startup(*get_module_offline_detect_tid());
    if (*get_gyro_tid() != RT_NULL) rt_thread_startup(*get_gyro_tid());
    if (*get_xsens_ahrs_tid() != RT_NULL) rt_thread_startup(*get_xsens_ahrs_tid());
    if (*get_sht30_tid() != RT_NULL) rt_thread_startup(*get_sht30_tid());
    if (*get_altimeter_tid() != RT_NULL) rt_thread_startup(*get_altimeter_tid());
    if (*get_wd_tid() != RT_NULL) rt_thread_startup(*get_wd_tid());
    if (*get_gnss_tid() != RT_NULL) rt_thread_startup(*get_gnss_tid());
    if (*get_rov_move_tid() != RT_NULL) rt_thread_startup(*get_rov_move_tid());
    if (*get_equipment_tid() != RT_NULL) rt_thread_startup(*get_equipment_tid());
    if (*get_can1_tid() != RT_NULL) rt_thread_startup(*get_can1_tid());
    if (*get_dvl_json_tcp_tid() != RT_NULL) rt_thread_startup(*get_dvl_json_tcp_tid());
    if (*get_modbus_tid() != RT_NULL) rt_thread_startup(*get_modbus_tid());
    if (*get_wit_gnss_data_tid() != RT_NULL) rt_thread_startup(*get_wit_gnss_data_tid());
    if (*get_yesens_tid() != RT_NULL) rt_thread_startup(*get_yesens_tid());
    if (*get_fid_ahrs_tid() != RT_NULL) rt_thread_startup(*get_fid_ahrs_tid());
    if (*get_hi12h4_ahrs_tid() != RT_NULL) rt_thread_startup(*get_hi12h4_ahrs_tid());
    if (*get_motor_test_tid() != RT_NULL) rt_thread_startup(*get_motor_test_tid());
}

void StartTelnetServer(void) {
    rt_thread_delay(2500);
    telnet_server();
}
