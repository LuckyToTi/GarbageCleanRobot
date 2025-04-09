#include <can1_thread.h>
#include <stm32f4xx_hal.h>
#include <stm32f4xx_hal_usart.h>
#include <rthw.h>
#include <rtthread.h>
#include <string.h>
#include "hardware/can1.h"
#include "hardware/motor_control_center.h"
#include "userlib/can_msg.h"
#include "userlib/bytetransformers.h"
#include "robolink_udp_task.h"
#include "config.h"
#include "movelib/rov_move.h"
#include "userlib/module_offline_detection.h"

#define LOG_TAG "can1_thread"
#define LOG_LVL LOG_LVL_INFO
#include <ulog.h>

#define CAN1_WAIT_FOREVER (0)

extern CAN_HandleTypeDef hcan1;
const uint16_t can1_rx0_max_len = 8;

// mailbox 相关 声明
static rt_mailbox_t can1_msg_tx_mb_ = RT_NULL;  // 发送邮箱

static int Can1RxAnalysis(uint32_t id, uint32_t IDE, uint32_t RTR, uint32_t dlc, const uint8_t* rx_data);

//can1线程控制块
rt_thread_t can1_tid = RT_NULL;
rt_thread_t can1_rx_tid = RT_NULL;
rt_thread_t can1_tx_tid = RT_NULL;

inline rt_thread_t* get_can1_tid() {
	return &can1_tid;
}

void Can1ThreadEntry(void* parameter) {
	can1_msg_tx_mb_ = rt_mb_create("CAN1_mb_t", 16, RT_IPC_FLAG_PRIO); 				// 邮箱

	// 启动CAN1处理任务
	can1_tx_tid = rt_thread_create("can1_tx", Can1TxThreadEntry, (void*) 0, 1024 - 256, 16, 5);
	if (can1_tx_tid != RT_NULL) rt_thread_startup(can1_tx_tid);
	can1_rx_tid = rt_thread_create("can1_rx", Can1RxThreadEntry, (void*) 0, 1024 - 256, 17, 5);
	if (can1_rx_tid != RT_NULL) rt_thread_startup(can1_rx_tid);

	// 测试用
//	uint8_t Can_Send_Buffer[] = { 1, 2, 3, 4, 5, 6, 7, 8 };
//	for (;;) {
//		Can1CreateInitSendNewMsgToMb(CAN1_MOTOR_TX_ID, sizeof(Can_Send_Buffer), (uint8_t*) Can_Send_Buffer);
//		MotorTxDataConvertAndSend(2,1300,1300,1300,3);
//		EquipmentTxDataConvertAndSend(0,1300,1600,1300,3);
//		rt_thread_mdelay(100);
//	}

	return;
}

// CAN发送线程
void Can1TxThreadEntry(void* parameter) {
	do {
		rt_thread_mdelay(500);  // 等待邮箱初始化
	} while (can1_msg_tx_mb_ == RT_NULL);

	for (;;) {
		struct CanMsg* new_msg = RT_NULL;
		if (rt_mb_recv(can1_msg_tx_mb_, (rt_ubase_t *) (&new_msg),
		               RT_WAITING_FOREVER) == RT_EOK) {
			if (new_msg != RT_NULL) {
                CanSendMsg(&hcan1, new_msg->id, new_msg->IDE, new_msg->RTR, new_msg->data, new_msg->dlc);
                can_free_unused_msg(new_msg);
			}
		}
	}
}

void Can1RxThreadEntry(void* parameter) {
	rt_thread_mdelay(500);
	struct MonitoredModule* can1_dev_m = RegisterNewMonitoredModule("can1_dev", 0, RT_TICK_PER_SECOND * 1.2, 2, NULL);

	char* can1_rx_data = rt_malloc(can1_rx0_max_len);
	for (;;) {
		rt_memset(can1_rx_data, 0, can1_rx0_max_len);
		if(rt_sem_take(get_can1_hal_rx0_sem(), RT_TICK_PER_SECOND) == RT_EOK) {
            uint32_t id = (get_can1_rx0_msg()->IDE==CAN_ID_STD) ? get_can1_rx0_msg()->StdId : get_can1_rx0_msg()->ExtId;
			Can1RxAnalysis(id, get_can1_rx0_msg()->IDE, get_can1_rx0_msg()->RTR, get_can1_rx0_msg()->DLC, get_can1_rx0_data());

			MonitoredModuleReload(can1_dev_m);
		}

//		else{
//			if(PUSHES_OUT_SWITCH == 1 || PUSHES_OUT_SWITCH == 2)
//				print_rl_udp(3, "Motor Control Center Disconnect!!!");
//		}
	}

	UnregisterMonitoredModuleByPtr(can1_dev_m);
	rt_free(can1_rx_data);
}

/**
 * 发送一个新的标准帧CAN消息至邮箱，由发送任务自动发送
 * @param std_id   标识符id
 * @param dlc 	数据段长度
 * @param data  数据段
 * 长度未生效
 */
void Can1CreateInitSendNewMsgToMb(uint32_t id, uint32_t IDE, uint32_t RTR, uint32_t dlc, const uint8_t* data)
{
	if (can1_msg_tx_mb_ != RT_NULL) {
	    struct CanMsg* new_can_tx_msg = can_create_init_new_msg(id, IDE, RTR, dlc, data);
	    if(new_can_tx_msg != NULL)
            if (RT_EOK != rt_mb_send(can1_msg_tx_mb_, (rt_ubase_t) new_can_tx_msg)) {
                can_free_unused_msg(new_can_tx_msg);
            }
	}
}

inline static int Can1RxAnalysis(uint32_t id, uint32_t IDE, uint32_t RTR, uint32_t dlc, const uint8_t* rx_data) {
//	LOG_D("Id: %x: %x %x %x %x %x %x %x %x", id,rx_data[0],rx_data[1],rx_data[2],rx_data[3],rx_data[4],rx_data[5],rx_data[6],rx_data[7]);
	if(dlc == 8){
        uint32_t vesc_status_id = id & 0xff00;
        uint32_t motor_id = id & 0x00ff;
        if(vesc_status_id == CAN1_RX_ID_STATUS_1 || vesc_status_id == CAN1_RX_ID_STATUS_2 ||
        		vesc_status_id == CAN1_RX_ID_STATUS_3 || vesc_status_id == CAN1_RX_ID_STATUS_4 ||
				vesc_status_id == CAN1_RX_ID_STATUS_5){
        	MotorVescRxDataConvert(get_rx_motor_msg(), rx_data, motor_id, vesc_status_id);
        	return vesc_status_id;
        }
        if(id > CAN1_RX_ID_C610_RETURN && id < (CAN1_RX_ID_C610_RETURN + 4)){
        	return CAN1_RX_ID_C610_RETURN;
        }
		switch (id) {
			case CAN1_RX_ID_MOTOR:
				MotorRxDataConvert(get_rx_motor_msg(), rx_data);
				return CAN1_RX_ID_MOTOR;
			case CAN1_RX_ID_WATER_WARNING:
				print_rl_udp(0x04, "Motor Control Center Leakage warning !");
				return CAN1_RX_ID_WATER_WARNING;
			case CAN1_RX_ID_SHT30_AND_WATER_WARNING:
				Sht30RxDataConvert(rx_data);
			default:
				return -1;
				break;
		}
	}else{
		return -1;
	}
}


