#include <hi12h4_ahrs_thread.h>
#include "stm32f4xx_hal_usart.h"
#include <string.h>
#include "config.h"
#include "hardware/usart1.h"
#include "hardware/usart5.h"
#include "robolink_udp_task.h"

#define LOG_TAG "hi12_t"
#define LOG_LVL LOG_LVL_INFO
#include <ulog.h>

const uint16_t hi12h4_ahrs_recv_max_len = 512 + 6;

static int HI12H4DataHeaderCheck(const uint8_t* data);
static int HI12H4DataCRCCheck(const uint16_t currectCrc, const uint8_t *src, uint32_t lengthInBytes);
static int HI12H4DataParse(const uint8_t* source_data, struct hi12h4_imu_data_t* set_imu_data);

struct hi12h4_imu_data_t hi12h4_imu_data = {};
inline const struct hi12h4_imu_data_t* get_hi12h4_imu_data()
{
    return &hi12h4_imu_data;
}

rt_thread_t hi12h4_ahrs_tid = RT_NULL;
inline rt_thread_t* get_hi12h4_ahrs_tid()
{
    return &hi12h4_ahrs_tid;
}

void HI12H4AhrsThreadEntry(void* parameter)
{
    uint8_t* hi12h4_ahrs_recv_data = rt_malloc(hi12h4_ahrs_recv_max_len);
    uint16_t payload_len, crc;
    rt_thread_mdelay(1000);
    for( ; ; ) {
        rt_memset(hi12h4_ahrs_recv_data, 0, hi12h4_ahrs_recv_max_len);
        int get_len = Uart1HalReceiveWaitDataItStopIdle((uint8_t*)hi12h4_ahrs_recv_data, hi12h4_ahrs_recv_max_len);
        if(!(get_len>0)) continue;
        payload_len = hi12h4_ahrs_recv_data[2] + (hi12h4_ahrs_recv_data[3] << 8);
        crc = hi12h4_ahrs_recv_data[4] + (hi12h4_ahrs_recv_data[5] << 8);
        if(HI12H4DataHeaderCheck(hi12h4_ahrs_recv_data) || HI12H4DataCRCCheck(crc, hi12h4_ahrs_recv_data, payload_len))
        {
            if(HI12H4DataParse(hi12h4_ahrs_recv_data + 6, &hi12h4_imu_data)){
                LOG_D("euler: roll_x = %.2f, pitch_y = %.2f, yaw_z = %.2f", get_hi12h4_imu_data()->euler.roll_x, get_hi12h4_imu_data()->euler.pitch_y, get_hi12h4_imu_data()->euler.yaw_z);
                LOG_D("mag: hx = %.2f, hy = %.2f, hz = %.2f", get_hi12h4_imu_data()->mag.hx, get_hi12h4_imu_data()->mag.hy, get_hi12h4_imu_data()->mag.hz);
                uint8_t euler_angles_raw[12] = {};
                rt_memcpy(euler_angles_raw + 0, &hi12h4_imu_data.euler.roll_x, sizeof(float));
                rt_memcpy(euler_angles_raw + 4, &hi12h4_imu_data.euler.pitch_y, sizeof(float));
                rt_memcpy(euler_angles_raw + 8, &hi12h4_imu_data.euler.yaw_z, sizeof(float));
                RobolinkUdpCreateInitSendNewMsgToMb((uint8_t *) euler_angles_raw, 12, get_config()->robolink_id_config.local_id, 0x02, 0x04);
            }
        }
    }
}

static int HI12H4DataHeaderCheck(const uint8_t* data)
{
    static const uint8_t header_check[] = {0x5A, 0xA5};
    for(uint8_t i = 0; i < 2; i++) {
        if(data[i] != header_check[i]) {
            return 1;
        }
    }
    return 0;
}

static void crc16_update(uint16_t *crc, const uint8_t *data, uint32_t len)
{
    for (uint32_t i = 0; i < len; ++i) {
        *crc ^= (uint16_t)data[i] << 8;
        
        for (uint8_t bit = 0; bit < 8; ++bit) {
            *crc = (*crc & 0x8000) ? (*crc << 1) ^ 0x1021 : *crc << 1;
        }
    }
}

static int HI12H4DataCRCCheck(uint16_t check_crc, const uint8_t *data, uint32_t len)
{
    uint16_t crc = 0;

    crc16_update(&crc, data, 4);          // 帧头数据
    crc16_update(&crc, data + 6, len);  // 有效载荷数据
    
    return (crc == check_crc);
}

static int HI12H4DataParse(const uint8_t* source_data, struct hi12h4_imu_data_t* set_imu_data)
{
    if (source_data[0] == 0x91)
    {
        set_imu_data->tag = source_data[1];
        set_imu_data->pressure = get_f(source_data + 4, LSB);
        set_imu_data->timestamp = get_u32(source_data + 8, LSB);
        set_imu_data->acc.x = get_f(source_data + 12, LSB);
        set_imu_data->acc.y = get_f(source_data + 16, LSB);
        set_imu_data->acc.z = get_f(source_data + 20, LSB);
        set_imu_data->gyr.x = get_f(source_data + 24, LSB);
        set_imu_data->gyr.y = get_f(source_data + 28, LSB);
        set_imu_data->gyr.z = get_f(source_data + 32, LSB);
        set_imu_data->mag.hx = get_f(source_data + 36, LSB);
        set_imu_data->mag.hy = get_f(source_data + 40, LSB);
        set_imu_data->mag.hz = get_f(source_data + 44, LSB);
        set_imu_data->euler.roll_x = get_f(source_data + 48, LSB);
        set_imu_data->euler.pitch_y = get_f(source_data + 52, LSB);
        set_imu_data->euler.yaw_z = get_f(source_data + 56, LSB);
        set_imu_data->quat.q[0] = get_f(source_data + 60, LSB);
        set_imu_data->quat.q[1] = get_f(source_data + 64, LSB);
        set_imu_data->quat.q[2] = get_f(source_data + 68, LSB);
        set_imu_data->quat.q[3] = get_f(source_data + 72, LSB);
        return 1;
    }else if(source_data[0] == 0x92){
        return 1;
    }
    return 0;
}

