#include <wit_gnss_thread.h>
#include <string.h>
#include <stm32f4xx_hal.h>
#include <stm32f4xx_hal_usart.h>
#include "hardware/usart5.h"
#include "robolink_udp_task.h"
#include "config.h"

#define LOG_TAG "wit_gnss"
#define LOG_LVL LOG_LVL_INFO
#include <ulog.h>

#define WIT_SYS_ID (0x05)
#define WIT_GNSS_DEV_ID (0x04)
#define WIT_GYRO_DEV_ID (0x02)

static const int witgnss_rx_max_len = 128;
struct WitGnssData wit_gnss_data;

rt_thread_t wit_gnss_tid = RT_NULL;
inline rt_thread_t* get_wit_gnss_data_tid() {
	return &wit_gnss_tid;
}

uint8_t CheckWitGnssData_Available(const uint8_t *wit_gnss_data);
uint8_t ParseWitGnssData(const uint8_t *wit_gnss_raw_data, struct WitGnssData *wit_gnss_data);

void WitGnssThreadEntry(void* parameter) {

	uint8_t* witgnss_rx_data = rt_malloc(witgnss_rx_max_len);
	rt_thread_mdelay(750);

	for (;;) {
		rt_memset(witgnss_rx_data, 0, witgnss_rx_max_len);
		int get_len = Uart5HalReceiveWaitDataItStopIdle((uint8_t*) witgnss_rx_data, witgnss_rx_max_len);
		int gnssdata_num = (get_len / 11);
		if (get_len % 11 == 0 && gnssdata_num != 0) {
			for (int _i = 0; _i < gnssdata_num; _i++) {
				if (CheckWitGnssData_Available(witgnss_rx_data + _i * 11)) {
					ParseWitGnssData(witgnss_rx_data + _i * 11, &wit_gnss_data);
				}
			}

			LOG_D("acceleration  x:%.2f\ty:%.2f\tz:%.2f  ", wit_gnss_data.acceleration.x, wit_gnss_data.acceleration.y, wit_gnss_data.acceleration.z);
			LOG_D("angle  x:%.2f\ty:%.2f\tz:%.2f  ", wit_gnss_data.angle.roll_x, wit_gnss_data.angle.pitch_y, wit_gnss_data.angle.yaw_z);
			LOG_D("Angular_velocity  x:%.2f\ty:%.2f\tz:%.2f  ", wit_gnss_data.Angular_velocity.x, wit_gnss_data.Angular_velocity.y, wit_gnss_data.Angular_velocity.z);
			LOG_D("Magnetic  x:%.2f\ty:%.2f\tz:%.2f  ", wit_gnss_data.Magnetic.x, wit_gnss_data.Magnetic.y, wit_gnss_data.Magnetic.z);
			LOG_D("latitude_and_longitude  longitude:%f\tlatitude:%f  ", wit_gnss_data.latitude_and_longitude.longitude, wit_gnss_data.latitude_and_longitude.latitude);
			LOG_D("gpsdata  height:%.2f\tHeading_angle:%.2f\tgrouand_speed:%.2f  ", wit_gnss_data.gpsdata.height, wit_gnss_data.gpsdata.Heading_angle, wit_gnss_data.gpsdata.grouand_speed);
			LOG_D("Quaternion  Q0:%.2f\tQ1:%.2f\tQ2:%.2f\tQ3%.2f:  ", wit_gnss_data.Quaternion.Quaternion0, wit_gnss_data.Quaternion.Quaternion1, wit_gnss_data.Quaternion.Quaternion2, wit_gnss_data.Quaternion.Quaternion3);
		}
	}
	rt_free(witgnss_rx_data);

}

//经纬度数据计算
double degreeToDecimal(double a) {
	int degree = (int) (a / 100);
	double minute = a - degree * 100;
	double decimalDegree = degree + minute / 60.0;
	return decimalDegree;
}

uint8_t ParseWitGnssData(const uint8_t *wit_gnss_raw_data, struct WitGnssData *wit_gnss_data) {
	switch (wit_gnss_raw_data[1]) //判断当前接受的是哪一种（加速度，角速度，角度...）数据包
	{
		// 解析加速度数据包
		case 0x51:
			wit_gnss_data->acceleration.x = get_i16(wit_gnss_raw_data + 2, LSB) / 32768.0 * 16.0;
			wit_gnss_data->acceleration.y = get_i16(wit_gnss_raw_data + 4, LSB) / 32768.0 * 16.0;
			wit_gnss_data->acceleration.z = get_i16(wit_gnss_raw_data + 6, LSB) / 32768.0 * 16.0;
			RobolinkUdpCreateInitSendNewMsgToMb((uint8_t *) &wit_gnss_raw_data[2], 6, WIT_SYS_ID, WIT_GYRO_DEV_ID, 0x11);
			return 1;
			//解析角速度数据包
		case 0x52:
			wit_gnss_data->Angular_velocity.x = get_i16(wit_gnss_raw_data + 2, LSB) / 32768.0 * 2000.0;
			wit_gnss_data->Angular_velocity.y = get_i16(wit_gnss_raw_data + 4, LSB) / 32768.0 * 2000.0;
			wit_gnss_data->Angular_velocity.z = get_i16(wit_gnss_raw_data + 6, LSB) / 32768.0 * 2000.0;
			RobolinkUdpCreateInitSendNewMsgToMb((uint8_t *) &wit_gnss_raw_data[2], 6, WIT_SYS_ID, WIT_GYRO_DEV_ID, 0x12);
			return 1;

			// 依照数据协议，解析角度数据包
		case 0x53:
			wit_gnss_data->angle.roll_x = get_i16(wit_gnss_raw_data + 2, LSB) / 32768.0 * 180;
			wit_gnss_data->angle.pitch_y = get_i16(wit_gnss_raw_data + 4, LSB) / 32768.0 * 180;
			wit_gnss_data->angle.yaw_z = get_i16(wit_gnss_raw_data + 6, LSB) / 32768.0 * 180;
			RobolinkUdpCreateInitSendNewMsgToMb((uint8_t *) &wit_gnss_raw_data[2], 6, WIT_SYS_ID, WIT_GYRO_DEV_ID, 0x14);
			return 1;

			// 依照数据协议，解析磁场数据包
		case 0x54:
			wit_gnss_data->Magnetic.x = get_i16(wit_gnss_raw_data + 2, LSB);
			wit_gnss_data->Magnetic.y = get_i16(wit_gnss_raw_data + 4, LSB);
			wit_gnss_data->Magnetic.z = get_i16(wit_gnss_raw_data + 6, LSB);
			RobolinkUdpCreateInitSendNewMsgToMb((uint8_t *) &wit_gnss_raw_data[2], 6, WIT_SYS_ID, WIT_GYRO_DEV_ID, 0x13);
			return 1;
			//  依照数据协议，解析经纬度数据包
		case 0x57:
			wit_gnss_data->latitude_and_longitude.longitude = degreeToDecimal(get_u32(wit_gnss_raw_data + 2, LSB) / 100000.0);
			wit_gnss_data->latitude_and_longitude.latitude = degreeToDecimal(get_u32(wit_gnss_raw_data + 6, LSB) / 100000.0);
			char mini_rmc_data[17];
            float latitude = wit_gnss_data->latitude_and_longitude.latitude;
            float longitude = wit_gnss_data->latitude_and_longitude.longitude;
            float course = wit_gnss_data->gpsdata.Heading_angle;
            float speed = wit_gnss_data->gpsdata.grouand_speed;
            if(latitude == 0.0f && longitude == 0.0f)
            	mini_rmc_data[0] = 'V';
            else
            	mini_rmc_data[0] = 'A';
            rt_memcpy(mini_rmc_data + 1, &latitude, sizeof(float));
            rt_memcpy(mini_rmc_data + 5, &longitude, sizeof(float));
            rt_memcpy(mini_rmc_data + 9, &course, sizeof(float));
            rt_memcpy(mini_rmc_data + 13, &speed, sizeof(float));
            RobolinkUdpCreateInitSendNewMsgToMb((uint8_t*) mini_rmc_data, sizeof(mini_rmc_data), get_config()->robolink_id_config.local_id, WIT_GNSS_DEV_ID, 0x02);
			return 1;
			// 依照数据协议，解析高度，航向角，地速数据包
		case 0x58:
			wit_gnss_data->gpsdata.height = get_i16(wit_gnss_raw_data + 2, LSB) / 10.0; //(m)
			wit_gnss_data->gpsdata.Heading_angle = get_i16(wit_gnss_raw_data + 4, LSB) / 100.0; //(°)
			wit_gnss_data->gpsdata.grouand_speed = get_i32(wit_gnss_raw_data + 6, LSB) / 1000.0; //(km/h)
			return 1;
			//  依照数据协议，解析四元数数据包
		case 0x59:
			wit_gnss_data->Quaternion.Quaternion0 = get_i16(wit_gnss_raw_data + 2, LSB) / 32768.0;
			wit_gnss_data->Quaternion.Quaternion1 = get_i16(wit_gnss_raw_data + 4, LSB) / 32768.0;
			wit_gnss_data->Quaternion.Quaternion2 = get_i16(wit_gnss_raw_data + 6, LSB) / 32768.0;
			wit_gnss_data->Quaternion.Quaternion3 = get_i16(wit_gnss_raw_data + 8, LSB) / 32768.0;
			RobolinkUdpCreateInitSendNewMsgToMb((uint8_t *) &wit_gnss_raw_data[2], 8, WIT_SYS_ID, WIT_GYRO_DEV_ID, 0x15);
			return 1;

	}
	return 0;
}

uint8_t CheckWitGnssData_Available(const uint8_t *wit_gnss_data) {
	uint8_t sum_add = 0;

	if (wit_gnss_data[0] != 0x55) {
		return 0;
	}

	for (uint8_t i = 0; i < 10; i++) {
		sum_add += wit_gnss_data[i];
	}

	if (wit_gnss_data[10] != sum_add) {
		return 0;
	}

	return 1;
}

