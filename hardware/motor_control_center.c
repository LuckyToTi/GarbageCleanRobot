#include "motor_control_center.h"
#include <rtthread.h>
#include <stm32f4xx_hal.h>
#include "can1_thread.h"
#include "userlib/bytetransformers.h"
#include "robolink_udp_task.h"
#include "config.h"

#define LOG_TAG "m_c_c"
#define LOG_LVL LOG_LVL_INFO
#include <ulog.h>

struct motor_msg rx_motor_msg[10];
struct motor_msg* get_rx_motor_msg(){
	return rx_motor_msg;
}


void MotorControlCenterSendMsg(uint32_t can_id, uint8_t index, uint8_t number, const uint16_t* outputs) {
    if(number > 3) return;
    uint8_t data[8];
    data[0] = index;
    data[1] = number;
    for(int _i = 0; _i < number; _i++){
        split_number(outputs + _i, sizeof(uint16_t), data + 2 + _i*2, MSB);
    }
    Can1CreateInitSendNewMsgToMb(can_id, CAN_ID_STD, CAN_RTR_DATA, 2 + number * 2, data);
}

void MotorTxDataConvertAndSend(uint8_t index, uint16_t output1, uint16_t output2, uint16_t output3, uint8_t number){
	uint8_t send_data[8];
	uint16_t output[3] = {output1,output2,output3};
	send_data[0] = index;
	send_data[1] = number;
	for(int _i = 0; _i < number; _i++){
		send_data[2 + _i * 2] = (output[_i] >> 8) & 0xFF;
		send_data[2 + _i * 2 + 1] = (output[_i]) & 0xFF;
	}
	Can1CreateInitSendNewMsgToMb(CAN1_TX_ID_MOTOR, CAN_ID_STD, CAN_RTR_DATA, sizeof(send_data), (uint8_t*) send_data);
}

void WheelTxDataConvertAndSend(int *arr){
	uint8_t send_data[8];
	for(int _i = 0; _i < 4; _i++){
		send_data[_i * 2] = (arr[_i] >> 8) & 0xFF;
		send_data[_i * 2 + 1] = (arr[_i]) & 0xFF;
	}
	Can1CreateInitSendNewMsgToMb(CAN1_TX_ID_C610_CTL_CURRENT_1_TO_4, CAN_ID_STD, CAN_RTR_DATA, sizeof(send_data), (uint8_t*) send_data);
}


void EquipmentTxDataConvertAndSend(uint8_t index, uint16_t output1, uint16_t output2, uint16_t output3, uint8_t number){
	uint8_t send_data[8];
	uint16_t output[3] = {output1,output2,output3};
	send_data[0] = index;
	send_data[1] = number;
	for(int _i = 0; _i < number; _i++){
		send_data[2 + _i * 2] = (output[_i] >> 8) & 0xFF;
		send_data[2 + _i * 2 + 1] = (output[_i]) & 0xFF;
	}
	Can1CreateInitSendNewMsgToMb(CAN1_TX_ID_EQUIPMENT, CAN_ID_STD, CAN_RTR_DATA, sizeof(send_data), (uint8_t*) send_data);
}

// 临时变量存储 云台默认值用
void TemporaryTxDataConvertAndSend(uint16_t output1, uint16_t output2, uint16_t output3, uint16_t output4, uint8_t number){
	uint8_t send_data[8];
	uint16_t output[4] = {output1,output2,output3,output4};
	for(int _i = 0; _i < number; _i++){
		send_data[0 + _i * 2] = (output[_i] >> 8) & 0xFF;
		send_data[0 + _i * 2 + 1] = (output[_i]) & 0xFF;
	}
	Can1CreateInitSendNewMsgToMb(CAN1_TX_ID_TEMPORARY, CAN_ID_STD, CAN_RTR_DATA, sizeof(send_data), (uint8_t*) send_data);
}

void MotorRxDataConvert(struct motor_msg* m_msg, const uint8_t* c_msg) {
	m_msg[c_msg[0]].rpm = get_u16(&c_msg[1], MSB);
	m_msg[c_msg[0]].toal_current = get_u16(&c_msg[3], MSB);
	m_msg[c_msg[0]].motor_temp = get_u16(&c_msg[5], MSB);

	int16_t id = c_msg[0];
	int32_t rpm = m_msg[c_msg[0]].rpm / 7;
	uint8_t data_tosend[6];
	rt_memcpy(data_tosend + 0, &id, sizeof(int16_t));
	rt_memcpy(data_tosend + 2, &rpm, sizeof(int32_t));
	RobolinkUdpCreateInitSendNewMsgToMb(data_tosend, 6, get_config()->robolink_id_config.local_id, 0x08, 0x01);
//	LOG_D("%d %d %d %d %d %d %d %d",m_msg[0].freq,m_msg[1].freq,m_msg[2].freq,m_msg[3].freq,
//			m_msg[4].freq,m_msg[5].freq,m_msg[6].freq,m_msg[7].freq);
}

union i32_byte {
    int32_t d1;
    uint8_t d2[4];
};

void MotorVescDutyControlTxDataConvertAndSend(uint8_t motor_id, int32_t output_percent) {
    union i32_byte data1;
    data1.d1 = output_percent;
	uint8_t send_data[4];
	send_data[0] = data1.d2[3];
	send_data[1] = data1.d2[2];
	send_data[2] = data1.d2[1];
	send_data[3] = data1.d2[0];
	Can1CreateInitSendNewMsgToMb((CAN1_TX_ID_VESC_CTL_DUTY | motor_id), CAN_ID_EXT, CAN_RTR_DATA, sizeof(send_data), (uint8_t*) send_data);
}

void MotorVescRxDataConvert(struct motor_msg* m_msg, const uint8_t* c_msg, uint32_t motor_id, uint32_t status_id) {
	if (motor_id > 8) {
		return;
	}
	switch (status_id) {
		case CAN1_RX_ID_STATUS_1:
			m_msg[motor_id].rpm = (c_msg[0] << 24 | c_msg[1] << 16 | c_msg[2] << 8 | c_msg[3]);
			m_msg[motor_id].toal_current = (c_msg[4] << 8 | c_msg[5]) * 10;
			m_msg[motor_id].duty_cycle = (c_msg[6] << 8 | c_msg[7]) * 1000;
			break;
		case CAN1_RX_ID_STATUS_2:
			m_msg[motor_id].amp_hours = (c_msg[0] << 24 | c_msg[1] << 16 | c_msg[2] << 8 | c_msg[3]) * 10000;
			m_msg[motor_id].amp_hours_charged = (c_msg[4] << 24 | c_msg[5] << 16 | c_msg[6] << 8 | c_msg[7]) * 10000;
			break;
		case CAN1_RX_ID_STATUS_3:
			m_msg[motor_id].watt_hours = (c_msg[0] << 24 | c_msg[1] << 16 | c_msg[2] << 8 | c_msg[3]) * 10000;
			m_msg[motor_id].watt_hours_charged = (c_msg[4] << 24 | c_msg[5] << 16 | c_msg[6] << 8 | c_msg[7]) * 10000;
			break;
		case CAN1_RX_ID_STATUS_4:
			m_msg[motor_id].fet_temp = (c_msg[0] << 8 | c_msg[1]) * 10;
			m_msg[motor_id].motor_temp = (c_msg[2] << 8 | c_msg[3]) * 10;
			m_msg[motor_id].toal_current_in = (c_msg[4] << 8 | c_msg[5]) * 10;
			m_msg[motor_id].pid_pos = (c_msg[6] << 8 | c_msg[7]) * 50;
			break;
		case CAN1_RX_ID_STATUS_5:
			m_msg[motor_id].tachometer_value = (c_msg[0] << 24 | c_msg[1] << 16 | c_msg[2] << 8 | c_msg[3]);
			m_msg[motor_id].input_voltage = (c_msg[4] << 8 | c_msg[5]) * 10;
			m_msg[motor_id].reserved = (c_msg[6] << 8 | c_msg[7]) * 1000;
			break;
		default:
			return;
			break;
	}
	int32_t rpm = m_msg[motor_id].rpm / 7;
	uint8_t data_tosend[6];
	rt_memcpy(data_tosend + 0, &motor_id, sizeof(int16_t));
	rt_memcpy(data_tosend + 2, &rpm, sizeof(int32_t));
	RobolinkUdpCreateInitSendNewMsgToMb(data_tosend, 6, get_config()->robolink_id_config.local_id, 0x08, 0x01);
	// todo: 完善其余信息上传
}

void Sht30RxDataConvert(const uint8_t* c_msg){
    uint16_t data_[3] = { };
        int j = 0;
        for (int i = 0;i < 3; i++) {
            data_[i] = get_u16(&c_msg[2 * j], MSB);
            j++;
        }
        float humi = (float)data_[0] / 100;
        float temp = (float)data_[1] / 100;
        uint8_t water_flag =  data_[2] >> 8 &&0xFF;
        if(water_flag){
        	print_rl_udp(3, "Water Warring!!!");
        }
        RobolinkUdpCreateInitSendNewMsgToMb((uint8_t*)&humi, sizeof(float), 0x03, 0x03, 0x04);
        RobolinkUdpCreateInitSendNewMsgToMb((uint8_t*)&temp, sizeof(float), 0x03, 0x03, 0x04);
}

