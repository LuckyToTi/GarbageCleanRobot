#include "altimeter_thread.h"
#include "hardware/usart2.h"
#include "string.h"
#include "userlib/bytetransformers.h"
#include "robolink_udp_task.h"
#include "config.h"

#define LOG_TAG "altim..."
#define LOG_LVL LOG_LVL_INFO
#include <ulog.h>

static uint8_t start_underwater_elevatometer_hex[] =
    {0x42,0x52,0x02,0x00,0x78,0x05,0x00,0x00,0x14,0x05,0x2C,0x01};
static const uint16_t usart2_rx_max_len_ = 256;

struct AltimeterData altimeter_msg;
inline const struct AltimeterData* get_altimeter_msg()
{
    return &altimeter_msg;
}

uint8_t get_altimeter_dev_id();
inline uint8_t get_altimeter_dev_id() {return 0x07;}
static void ParseUnderwaterElevatometerData(struct AltimeterData* altimeter_data, const uint8_t* raw_data, int raw_data_len);

rt_thread_t altimeter_tid = RT_NULL;
inline rt_thread_t* get_altimeter_tid()
{
    return &altimeter_tid;
}

void AltimeterThreadEntry(void* parameter)
{
ALTIMERER_START:
    rt_thread_mdelay(350);

    // 开启定高计发送
    Usart2HalSendDataIt(start_underwater_elevatometer_hex, sizeof(start_underwater_elevatometer_hex));
//    LOG_I("aaa");
//    rt_thread_mdelay(500);

    for(;;) {
        uint8_t* usart2_rx_data = rt_malloc(usart2_rx_max_len_);
        int usart2_get_len = Usart2HalReceiveWaitTimeoutDataItStopIdle((uint8_t*)usart2_rx_data, usart2_rx_max_len_, 2000);
//        int usart2_get_len = -1;
//        rt_thread_mdelay(2000);
        if(usart2_get_len < 0) {
            rt_free(usart2_rx_data);
            goto ALTIMERER_START;
        }
        ParseUnderwaterElevatometerData(&altimeter_msg, usart2_rx_data, usart2_get_len);
        float d_m = altimeter_msg.distance / 1000.0f;
        RobolinkUdpCreateInitSendNewMsgToMb((uint8_t*)&d_m, 4, get_config()->robolink_id_config.local_id, 0x07, 0x03);
        RobolinkUdpCreateInitSendNewMsgToMb((uint8_t*)&altimeter_msg, sizeof(struct AltimeterData), get_config()->robolink_id_config.local_id, get_altimeter_dev_id(), 0x13);
        rt_free(usart2_rx_data);
    }
}

//static inline void ParseUnderwaterElevatometerData()
//{
//    if(underwater_elevatometer_rx_data[0]==0x42 && underwater_elevatometer_rx_data[1]==0x52
//            && usart2_get_len_ == 236)
//    {
//        altimeter_msg.distance = get_u32(underwater_elevatometer_rx_data+8,LSB);//距离
//        altimeter_msg.DataConfidence = get_u16(underwater_elevatometer_rx_data+12,LSB);//置信度
//        altimeter_msg.SendTime = get_u16(underwater_elevatometer_rx_data+14,LSB);//发送持续时间
//        altimeter_msg.ping_number = get_u32(underwater_elevatometer_rx_data+16,LSB);//脉冲/测量计数
//        altimeter_msg.scan_start = get_u32(underwater_elevatometer_rx_data+20,LSB);//扫描区域起点
//        altimeter_msg.scan_length = get_u32(underwater_elevatometer_rx_data+24,LSB);//扫描区域长度
//        altimeter_msg.gain_seting = get_u32(underwater_elevatometer_rx_data+28,LSB);//当前增益
//        altimeter_msg.profile_data_length = get_u16(underwater_elevatometer_rx_data+32,LSB);//样本数据数组长度
//        memcpy(altimeter_msg.profile_data,(uint8_t*)&underwater_elevatometer_rx_data[34],200);
//
//        LOG_D("Distance:   %dmm,DataConfidence: %d%%,SendTime:    %dus,ping_number:       %d\r\n",altimeter_msg.distance,altimeter_msg.DataConfidence,altimeter_msg.SendTime,altimeter_msg.ping_number);
//        LOG_D("scan_start: %dmm,scan_length:    %dmm,gain_seting: %d,profile_data_length: %d\r\n",altimeter_msg.scan_start,altimeter_msg.scan_length,altimeter_msg.gain_seting,altimeter_msg.profile_data_length);
//    }
//}

static void ParseUnderwaterElevatometerData(struct AltimeterData* altimeter_data, const uint8_t* raw_data, int raw_data_len)
{
    if(raw_data[0]==0x42 && raw_data[1]==0x52 && raw_data_len == 236) {
        altimeter_data->distance = get_u32(raw_data+8,LSB);//距离
        altimeter_data->DataConfidence = get_u16(raw_data+12,LSB);//置信度
        altimeter_data->SendTime = get_u16(raw_data+14,LSB);//发送持续时间
        altimeter_data->ping_number = get_u32(raw_data+16,LSB);//脉冲/测量计数
        altimeter_data->scan_start = get_u32(raw_data+20,LSB);//扫描区域起点
        altimeter_data->scan_length = get_u32(raw_data+24,LSB);//扫描区域长度
        altimeter_data->gain_seting = get_u32(raw_data+28,LSB);//当前增益
        altimeter_data->profile_data_length = get_u16(raw_data+32,LSB);//样本数据数组长度
        memcpy(altimeter_data->profile_data,(uint8_t*)&raw_data[34],200);

        LOG_D("Distance:   %dmm,DataConfidence: %d%%,SendTime:    %dus,ping_number:       %d\r\n",altimeter_msg.distance,altimeter_msg.DataConfidence,altimeter_msg.SendTime,altimeter_msg.ping_number);
        LOG_D("scan_start: %dmm,scan_length:    %dmm,gain_seting: %d,profile_data_length: %d\r\n",altimeter_msg.scan_start,altimeter_msg.scan_length,altimeter_msg.gain_seting,altimeter_msg.profile_data_length);
    }
}
