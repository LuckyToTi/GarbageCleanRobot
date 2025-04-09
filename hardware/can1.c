#include "can1.h"
#include "config.h"
#include <string.h>
#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_can.h"
#include "rtthread.h"

#define malloc_(size)  rt_malloc(size)
#define free_(ptr)     rt_free(ptr)

#define LOG_TAG "CAN1"
#define LOG_LVL LOG_LVL_INFO
#include <ulog.h>

/**CAN1 GPIO Configuration
 PD0     ------> CAN1_RX
 PD1     ------> CAN1_TX
 */
CAN_HandleTypeDef hcan1;


CAN_RxHeaderTypeDef can1_rx0_meg;
inline const CAN_RxHeaderTypeDef* get_can1_rx0_msg() {
	return &can1_rx0_meg;
}

uint8_t Can1_Rx0_Data[8];
inline const uint8_t* get_can1_rx0_data() {
    return Can1_Rx0_Data;
}

rt_sem_t can1_hal_rx0_sem = RT_NULL;
inline rt_sem_t get_can1_hal_rx0_sem() {
	return can1_hal_rx0_sem;
}

static void Hal_Can1_Rx0_Complete_IRQHandler();

void Can1Init() {
	can1_hal_rx0_sem = rt_sem_create("Can1Rx0", 0, RT_IPC_FLAG_PRIO);
	CAN_FilterTypeDef FilterConfig;
	HAL_StatusTypeDef HAL_Status;
	GPIO_InitTypeDef GPIO_InitStruct = { 0 };

	__HAL_RCC_CAN1_CLK_ENABLE();
	__HAL_RCC_GPIOD_CLK_ENABLE();

	GPIO_InitStruct.Pin = GPIO_PIN_0 | GPIO_PIN_1;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	GPIO_InitStruct.Alternate = GPIO_AF9_CAN1;
	HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

	hcan1.Instance = CAN1;
	hcan1.Init.Prescaler = 2;
	hcan1.Init.Mode = CAN_MODE_NORMAL;
	hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
	hcan1.Init.TimeSeg1 = CAN_BS1_12TQ;
	hcan1.Init.TimeSeg2 = CAN_BS2_8TQ;
	hcan1.Init.TimeTriggeredMode = DISABLE;
	hcan1.Init.AutoBusOff = ENABLE;
	hcan1.Init.AutoWakeUp = DISABLE;
	hcan1.Init.AutoRetransmission = ENABLE;
	hcan1.Init.ReceiveFifoLocked = DISABLE;
	hcan1.Init.TransmitFifoPriority = ENABLE; //发送仲裁模式为FIFO 默认为优先级排序
	if (HAL_CAN_Init(&hcan1) != HAL_OK) {
		LOG_D("init CAN failed \r\n");
	}

	HAL_NVIC_SetPriority(CAN1_TX_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(CAN1_TX_IRQn);
	HAL_NVIC_SetPriority(CAN1_RX0_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(CAN1_RX0_IRQn);

	FilterConfig.FilterBank = 0;
	FilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
	FilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
	FilterConfig.FilterIdHigh = 0;
	FilterConfig.FilterIdLow = 0;
	FilterConfig.FilterMaskIdHigh = 0;
	FilterConfig.FilterMaskIdLow = 0;
	FilterConfig.FilterFIFOAssignment = 0;
	FilterConfig.FilterActivation = ENABLE;
	FilterConfig.SlaveStartFilterBank = 0;

	HAL_CAN_RegisterCallback(&hcan1, HAL_CAN_RX_FIFO0_MSG_PENDING_CB_ID, Hal_Can1_Rx0_Complete_IRQHandler);

	HAL_Status = HAL_CAN_ConfigFilter(&hcan1, &FilterConfig);
	HAL_Status = HAL_CAN_Start(&hcan1);
	if (HAL_Status != HAL_OK) {
		LOG_D("open CAN failed \r\n");
	}
	HAL_Status = HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);
	if (HAL_Status != HAL_OK) {
		LOG_D("open can hang-up interrupt failed\r\n");
	}
}

static void Hal_Can1_Rx0_Complete_IRQHandler() {
	HAL_StatusTypeDef ret;
	ret = HAL_CAN_GetRxMessage(&hcan1, CAN_RX_FIFO0, &can1_rx0_meg, Can1_Rx0_Data);
	if (HAL_OK == ret) {
		rt_sem_release(can1_hal_rx0_sem);
	}
}

void CAN1_RX0_IRQHandler(void) {
	rt_interrupt_enter();
	HAL_CAN_IRQHandler(&hcan1);
	rt_interrupt_leave();
}

uint8_t CanSendStdDataMsg(CAN_HandleTypeDef* hcan, uint16_t std_id, uint8_t *p_data, uint8_t data_lenth) {
	//	uint32_t free_level = 0U;
	CAN_TxHeaderTypeDef trans_msg;
	trans_msg.IDE = CAN_ID_STD;
	trans_msg.RTR = CAN_RTR_DATA;
	trans_msg.StdId = std_id;  //标准帧ID
	trans_msg.DLC = data_lenth;  //发送的帧长度
	trans_msg.IDE = CAN_RTR_DATA;  //数据帧
//	static uint16_t time = 0;
//	while (free_level == 0)  //一共有三个邮箱，只要有一个邮箱空闲就可以了。
//	{
//		time++;
//		free_level = HAL_CAN_GetTxMailboxesFreeLevel(hcan);
//		if(time > time_out && time_out != 0){
//			return -1;
//		}
//	}
	HAL_CAN_AddTxMessage(&hcan1, &trans_msg, p_data, (uint32_t*) CAN_TX_MAILBOX0);
	return 0;
}

inline uint8_t CanSendMsg(CAN_HandleTypeDef* hcan, uint32_t id, uint32_t IDE, uint32_t RTR, uint8_t *p_data, uint32_t data_lenth) {
    CAN_TxHeaderTypeDef trans_msg;
    trans_msg.IDE = IDE;
    trans_msg.RTR = RTR;

    if(trans_msg.IDE == CAN_ID_STD) {
        trans_msg.StdId = id; //标准帧ID
    } else {
        trans_msg.ExtId = id;
    }

    trans_msg.DLC = data_lenth;  //发送的帧长度

    static int mb_select = 0;
    uint32_t* can_mailbox = (uint32_t*) (CAN_TX_MAILBOX0 << mb_select);
    if(++mb_select > 2) mb_select = 0;
    return HAL_CAN_AddTxMessage(hcan, &trans_msg, p_data, can_mailbox);
}
