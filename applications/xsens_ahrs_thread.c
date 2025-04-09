#include "xsens_ahrs_thread.h"
#include "stm32f4xx_hal_usart.h"
#include <string.h>
#include "config.h"
#include "hardware/usart1.h"
#include "robolink_udp_task.h"

#define LOG_TAG "xsens_t"
#define LOG_LVL LOG_LVL_INFO
#include <ulog.h>

const uint16_t xsens_ahrs_recv_max_len = 512 + 5;

struct MTData2 mt_data_2 = {};
inline const struct MTData2* get_mt_data_2()
{
    return &mt_data_2;
}

rt_thread_t xsens_ahrs_tid = RT_NULL;
inline rt_thread_t* get_xsens_ahrs_tid()
{
    return &xsens_ahrs_tid;
}

void XsensAhrsThreadEntry(void* parameter)
{
    uint8_t* xsens_ahrs_recv_data = rt_malloc(xsens_ahrs_recv_max_len);
    rt_thread_mdelay(750);
    for( ; ; ) {
        rt_memset(xsens_ahrs_recv_data, 0, xsens_ahrs_recv_max_len);
        int get_len = Uart1HalReceiveWaitDataItStopIdle((uint8_t*)xsens_ahrs_recv_data, xsens_ahrs_recv_max_len);
        if(!(get_len>0)) continue;
        if(XBusMTData2HeaderCheck(xsens_ahrs_recv_data) || XBusCheckChecksum(xsens_ahrs_recv_data, get_len))
            continue;
        if((XBusCheckGetLen(xsens_ahrs_recv_data, get_len) == (39 - 5))
                || (XBusCheckGetLen(xsens_ahrs_recv_data, get_len) == (84 - 5)))
        {
            OrientationEulerAnglesDataParse(xsens_ahrs_recv_data + 16, &mt_data_2);
            uint8_t euler_angles_raw[12] = {};
            rt_memcpy(euler_angles_raw + 0, &mt_data_2.euler_angle.roll_x, sizeof(float));
            rt_memcpy(euler_angles_raw + 4, &mt_data_2.euler_angle.pitch_y, sizeof(float));
            rt_memcpy(euler_angles_raw + 8, &mt_data_2.euler_angle.yaw_z, sizeof(float));
            RobolinkUdpCreateInitSendNewMsgToMb((uint8_t *) euler_angles_raw, 12, get_config()->robolink_id_config.local_id, 0x02, 0x04);
        }
    }
}

