#include "gyro_config.h"
#include "hardware/iic3.h"
#include "config.h"

#define LOG_TAG     "g_c"         // 该模块对应的标签。不定义时，默认：NO_TAG
#define LOG_LVL     LOG_LVL_INFO  // 该模块对应的日志输出级别。不定义时，默认：调试级别
#include <ulog.h>                 // 必须在 LOG_TAG 与 LOG_LVL 下面

uint16_t gyroGet(enum Gyro_Config_Setting gcs){
	uint8_t gyro_read_data[2];
	uint16_t read_value;
	if(gcs == AXIS)
		I2c3MemRead(GYRO_DEV_ADDR, 0x24,I2C_MEMADD_SIZE_8BIT,gyro_read_data,2);
	if(gcs == ORI)
		I2c3MemRead(GYRO_DEV_ADDR, 0x23,I2C_MEMADD_SIZE_8BIT,gyro_read_data,2);
	read_value = gyro_read_data[1] << 8 | gyro_read_data[0];
	return read_value;
}

void gyroResetCalibration(void){
	gyroSet(0x01,0x04,2);
	gyroSet(0x01,0x08,2);
}

// 0为水平 1为竖直
int gyroOrient(int ori){
	ori = (ori == 0)? 0 : ((ori == 1) ? 1 : -1);
	if(ori == -1) return -1;
    gyroSet(0x23,ori,2);
    return 1;
}
// 0,9为9轴,1,6为6轴
int gyroAxis(int axis){
	axis = (axis == 9) ? 0 : ((axis == 6) ? 1 : ((axis != 0 && axis != 1) ? -1 : axis));
	if(axis == -1) return -1;
    gyroSet(0x24,axis,2);
    return 1;
}

void gyroSet(uint8_t MemAddress, uint16_t data, uint8_t len){
    uint8_t gyro_free[] = {0x69,0x88,0xb5};   //解锁
    uint8_t gyro_save[] = {0x00,0x00,0x00};   //保存
    uint8_t gyro_set[] 	= {MemAddress,data,data >> 8};   //状态
    I2c3MemWrite(GYRO_DEV_ADDR,gyro_free[0],I2C_MEMADD_SIZE_8BIT,gyro_free+1,2);  //解锁
    rt_thread_delay(200);
    I2c3MemWrite(GYRO_DEV_ADDR,gyro_set[0],I2C_MEMADD_SIZE_8BIT,gyro_set+1,len);  //修改状态
    rt_thread_delay(3000);
    I2c3MemWrite(GYRO_DEV_ADDR,gyro_save[0],I2C_MEMADD_SIZE_8BIT,gyro_save+1,2);  //保存修改值
    rt_thread_delay(200);
}
