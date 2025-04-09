#include "gyro_thread.h"
#include "config.h"
#include "hardware/iic3.h"
#include "robolink_udp_task.h"
#include "userlib/module_offline_detection.h"

#define LOG_TAG     "g_t"         // 该模块对应的标签。不定义时，默认：NO_TAG
#define LOG_LVL     LOG_LVL_INFO  // 该模块对应的日志输出级别。不定义时，默认：调试级别
#include <ulog.h>                 // 必须在 LOG_TAG 与 LOG_LVL 下面

static void __ParseWitMotionGyroEulAngleData(const uint8_t *gyro_raw_data_0x3d_to_0x40, struct BspWitMotionGyroData* gyro_data);
static void __ParseWitMotionGyroQuaternionsData(const uint8_t *gyro_raw_data_0x3d_to_0x40, struct BspWitMotionGyroData* gyro_data);
static void __ParseWitMotionGyroAccelerationData(const uint8_t *gyro_raw_data_0x34_to_0x36, struct BspWitMotionGyroData* gyro_data);
static void __ParseWitMotionGyroAngularVelocityData(const uint8_t *gyro_raw_data_0x37_to_0x39, struct BspWitMotionGyroData* gyro_data);
static void __ParseWitMotionGyroMagneticFieldData(const uint8_t *gyro_raw_data_0x58_to_0x60, struct BspWitMotionGyroData* gyro_data);


uint8_t get_gyro_dev_id();
inline uint8_t get_gyro_dev_id() {return 0x02;}

#define GYRO_DEV_ADDR (0X50 << 1)

// 板载陀螺仪数据
struct BspWitMotionGyroData bsp_gyro_data = {};
inline const struct BspWitMotionGyroData* get_bsp_gyro_data()
{
    return &bsp_gyro_data;
}

struct GyroRxParm
{
    uint16_t dev_addr;
    uint16_t save_position;
    uint16_t size;
    uint8_t data_id;
    void (*parse_func)(const uint8_t*, struct BspWitMotionGyroData*);
};

static const struct GyroRxParm gyro_rx_parms[5] =
{
    {0x34,  0, 6, 0x11, __ParseWitMotionGyroAccelerationData},  // 0x34 ~ 0x36
    {0x37,  6, 6, 0x12, __ParseWitMotionGyroAngularVelocityData},  // 0x37 ~ 0x39
    {0x3A, 12, 6, 0x13, __ParseWitMotionGyroMagneticFieldData},  // 0x3A ~ 0x3C
    {0x3D, 18, 6, 0x14, __ParseWitMotionGyroEulAngleData},  // 0x3D ~ 0x3F
    {0x51, 24, 8, 0x15, __ParseWitMotionGyroQuaternionsData}  // 0x51 ~ 0x54
};

rt_thread_t gyro_tid = RT_NULL;
inline rt_thread_t* get_gyro_tid()
{
    return &gyro_tid;
}

void GyroThreadEntry(void* parameter)
{
    struct MonitoredModule* wit_gyro_m = RegisterNewMonitoredModule("bsp_wit_gyro", 0, 1000 / 50 *2, 10, NULL);
    rt_thread_mdelay(750);

    for(;;) {
        rt_thread_mdelay(1000 / 50);  // 50hz
        uint8_t* gyro_raw_data_0x34_to_0x3F_and_0x51_to_0x54 = (uint8_t*) rt_malloc(16 * sizeof(uint16_t));

        for(int _i=0; _i<5; _i++) {

            // 读取陀螺仪原始数据
            int iic3_read_err = I2c3MemRead(GYRO_DEV_ADDR, gyro_rx_parms[_i].dev_addr,I2C_MEMADD_SIZE_8BIT,
                    &gyro_raw_data_0x34_to_0x3F_and_0x51_to_0x54[gyro_rx_parms[_i].save_position],gyro_rx_parms[_i].size);

            // 读取无误则解析并上传
            if(iic3_read_err == 0) {
                gyro_rx_parms[_i].parse_func(&gyro_raw_data_0x34_to_0x3F_and_0x51_to_0x54[gyro_rx_parms[_i].save_position], &bsp_gyro_data);
                RobolinkUdpCreateInitSendNewMsgToMb(&gyro_raw_data_0x34_to_0x3F_and_0x51_to_0x54[gyro_rx_parms[_i].save_position], gyro_rx_parms[_i].size,
                        0x03, get_gyro_dev_id(), gyro_rx_parms[_i].data_id);
                MonitoredModuleReload(wit_gyro_m);
            }
        }
        rt_free(gyro_raw_data_0x34_to_0x3F_and_0x51_to_0x54);
    }
    UnregisterMonitoredModuleByPtr(wit_gyro_m);
    return ;
}

// 欧拉角与温度解析
static void __ParseWitMotionGyroEulAngleData(const uint8_t *gyro_raw_data_0x3d_to_0x40, struct BspWitMotionGyroData* gyro_data)
{
    gyro_data->euler_angle.roll_x = get_i16(gyro_raw_data_0x3d_to_0x40 + 0, LSB) / 32768.0f * 180.0f;
    gyro_data->euler_angle.pitch_y = get_i16(gyro_raw_data_0x3d_to_0x40 + 2, LSB) / 32768.0f * 180.0f;
    gyro_data->euler_angle.yaw_z = get_i16(gyro_raw_data_0x3d_to_0x40 + 4, LSB) / 32768.0f * 180.0f;
    LOG_D("GYRO euler angle(roll_x, pitch_y, yaw_z): %f, %f, %f", gyro_data->euler_angle.roll_x, gyro_data->euler_angle.pitch_y, gyro_data->euler_angle.yaw_z);
}

// 四元数解析
static void __ParseWitMotionGyroQuaternionsData(const uint8_t *gyro_raw_data_0x51_to_0x54, struct BspWitMotionGyroData* gyro_data)
{
    gyro_data->quaternions.q[0] = get_i16(gyro_raw_data_0x51_to_0x54 + 0, LSB) / 32768.0f;
    gyro_data->quaternions.q[1] = get_i16(gyro_raw_data_0x51_to_0x54 + 2, LSB) / 32768.0f;
    gyro_data->quaternions.q[2] = get_i16(gyro_raw_data_0x51_to_0x54 + 4, LSB) / 32768.0f;
    gyro_data->quaternions.q[3] = get_i16(gyro_raw_data_0x51_to_0x54 + 6, LSB) / 32768.0f;
    LOG_D("GYRO quaternions(q0, q1, q2, q3): %.2f, %.2f, %.2f, %.2f", gyro_data->quaternions.q[0], gyro_data->quaternions.q[1], gyro_data->quaternions.q[2], gyro_data->quaternions.q[3]);
}

// 加速度解析
static void __ParseWitMotionGyroAccelerationData(const uint8_t *gyro_raw_data_0x34_to_0x36, struct BspWitMotionGyroData* gyro_data)
{
    gyro_data->acceleration.x = get_i16(gyro_raw_data_0x34_to_0x36 + 0, LSB) / 32768.0 * 16.0;
    gyro_data->acceleration.y = get_i16(gyro_raw_data_0x34_to_0x36 + 2, LSB) / 32768.0 * 16.0;
    gyro_data->acceleration.z = get_i16(gyro_raw_data_0x34_to_0x36 + 4, LSB) / 32768.0 * 16.0;
    LOG_D("GYRO acc(x, y, z): %.2f, %.2f, %.2f", gyro_data->acceleration.x, gyro_data->acceleration.y, gyro_data->acceleration.z);
}

// 角速度解析
static void __ParseWitMotionGyroAngularVelocityData(const uint8_t *gyro_raw_data_0x37_to_0x39, struct BspWitMotionGyroData* gyro_data)
{
    gyro_data->angular_velocity.x = get_i16(gyro_raw_data_0x37_to_0x39 + 0, LSB) / 32768.0 * 2000.0;
    gyro_data->angular_velocity.y = get_i16(gyro_raw_data_0x37_to_0x39 + 2, LSB) / 32768.0 * 2000.0;
    gyro_data->angular_velocity.z = get_i16(gyro_raw_data_0x37_to_0x39 + 4, LSB) / 32768.0 * 2000.0;
    LOG_D("GYRO angular velocity(x, y, z): %.2f, %.2f, %.2f", gyro_data->angular_velocity.x, gyro_data->angular_velocity.y, gyro_data->angular_velocity.z);
}

// 磁场解析
static void __ParseWitMotionGyroMagneticFieldData(const uint8_t *gyro_raw_data_0x58_to_0x60, struct BspWitMotionGyroData* gyro_data)
{
    gyro_data->magnetic_field.hx = get_i16(gyro_raw_data_0x58_to_0x60 + 0, LSB);
    gyro_data->magnetic_field.hy = get_i16(gyro_raw_data_0x58_to_0x60 + 2, LSB);
    gyro_data->magnetic_field.hz = get_i16(gyro_raw_data_0x58_to_0x60 + 4, LSB);
    LOG_D("GYRO magnetic field(hx, hy, hz): %d, %d, %d", gyro_data->magnetic_field.hx, gyro_data->magnetic_field.hy, gyro_data->magnetic_field.hz);
}
