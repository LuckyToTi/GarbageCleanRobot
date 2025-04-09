#ifndef _MOTOR_CONTROL_CENTER_H_
#define _MOTOR_CONTROL_CENTER_H_

#include "stm32f4xx.h"
#include <rtthread.h>

#define MOTOR_CONTROL_CENTER_TX_CAN_ID_MOTORS (0x0200)
#define MOTOR_CONTROL_CENTER_TX_CAN_ID_EQUIPMENTS (0x0205)
#define MOTOR_CONTROL_CENTER_TX_CAN_ID_TEMPORARY (0x0300)

#define CAN1_TX_ID_MOTOR (0x0200)
#define CAN1_TX_ID_EQUIPMENT (0x0205)
#define CAN1_TX_ID_TEMPORARY (0x0300)
#define CAN1_TX_ID_VESC_CTL_DUTY (0x0000)
#define CAN1_TX_ID_VESC_CTL_CURRENT (0x0100)
#define CAN1_TX_ID_VESC_CTL_CURRENT_BREAK (0x0200)
#define CAN1_TX_ID_VESC_CTL_RPM (0x0300)
#define CAN1_TX_ID_C610_CTL_CURRENT_1_TO_4 (0x0200)
#define CAN1_TX_ID_C610_CTL_CURRENT_5_TO_8 (0x01FF)

#define CAN1_RX_ID_MOTOR (0x0201)
#define CAN1_RX_ID_WATER_WARNING (0x0210)
#define CAN1_RX_ID_STATUS_1 (0x0900)
#define CAN1_RX_ID_STATUS_2 (0x0E00)
#define CAN1_RX_ID_STATUS_3 (0x0F00)
#define CAN1_RX_ID_STATUS_4 (0x1000)
#define CAN1_RX_ID_STATUS_5 (0x1b00)
#define CAN1_RX_ID_SHT30_AND_WATER_WARNING (0x0202)
#define CAN1_RX_ID_C610_RETURN (0x200)

struct motor_msg{
	int16_t duty_cycle; // *1000
	int16_t toal_current; // *10
	int32_t rpm;
	int32_t amp_hours_charged; // *10000
	int32_t amp_hours; // *10000
	int32_t watt_hours_charged; // *10000
	int32_t watt_hours; // *10000
	int16_t pid_pos; // *50
	int16_t toal_current_in; // *10
	int16_t motor_temp; // *10
	int16_t fet_temp; // *10
	int16_t reserved; // *10
	int16_t input_voltage; // *10
	int32_t tachometer_value;
};

void MotorControlCenterSendMsg(uint32_t can_id, uint8_t index, uint8_t number, const uint16_t* outputs);

struct motor_msg* get_rx_motor_msg();
void MotorRxDataConvert(struct motor_msg* m_msg, const uint8_t* c_msg);
void MotorTxDataConvertAndSend(uint8_t index, uint16_t output1, uint16_t output2, uint16_t output3, uint8_t number);
void MotorVescDutyControlTxDataConvertAndSend(uint8_t motor_id, int32_t output_percent);
void EquipmentTxDataConvertAndSend(uint8_t index, uint16_t output1, uint16_t output2, uint16_t output3, uint8_t number);
void TemporaryTxDataConvertAndSend(uint16_t output1, uint16_t output2, uint16_t output3, uint16_t output4, uint8_t number);
void MotorVescRxDataConvert(struct motor_msg* m_msg, const uint8_t* c_msg, uint32_t motor_id, uint32_t status_id);
void WheelTxDataConvertAndSend(int *arr);
void Sht30RxDataConvert(const uint8_t* c_msg);

#endif /* _MOTOR_CONTROL_CENTER_H_ */
