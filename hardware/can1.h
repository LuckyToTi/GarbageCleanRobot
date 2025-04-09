#ifndef HARDWARE_CAN1_H_
#define HARDWARE_CAN1_H_

#include <rthw.h>
#include <stm32f4xx.h>

const CAN_RxHeaderTypeDef* get_can1_rx0_msg();
const uint8_t* get_can1_rx0_data();
rt_sem_t get_can1_hal_rx0_sem();

void Can1Init();
uint8_t* GetCan1Rx0DataTest();
uint8_t CanSendStdDataMsg(CAN_HandleTypeDef* hcan, uint16_t std_id, uint8_t *p_data, uint8_t data_lenth);
uint8_t CanSendMsg(CAN_HandleTypeDef* hcan, uint32_t id, uint32_t IDE, uint32_t RTR, uint8_t *p_data, uint32_t data_lenth);

#endif /* HARDWARE_CAN1_H_ */
