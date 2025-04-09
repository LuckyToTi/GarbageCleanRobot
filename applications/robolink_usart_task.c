#include "robolink_usart_task.h"
#include "stm32f4xx_hal_usart.h"
#include <string.h>
#include "config.h"
#include "hardware/usart1.h"
#include "controller_switch_thread.h"

#define LOG_TAG "RB_uart"
#define LOG_LVL LOG_LVL_INFO
#include <ulog.h>

const uint16_t robolink_usart_rx_max_len = 1024;
const uint16_t sbus_rc_rx_max_len = 128;

struct SbusRc sbus_rc_msg = {};
inline const struct SbusRc* get_sbus_rc_msg()
{
    return &sbus_rc_msg;
}

struct RobolinkGetMsg robolink_usart_get_msg = {};
inline const struct RobolinkGetMsg* get_robolink_usart_get_msg()
{
    return &robolink_usart_get_msg;
}


//串口1线程控制块
rt_thread_t robolink_usart_tid = RT_NULL;
inline rt_thread_t* get_robolink_usart_tid()
{
    return &robolink_usart_tid;
}

void ControllerUsartThreadEntry(void* parameter)
{
    sbus_rc_reset(&sbus_rc_msg);
    reset_controller_gamepad(&robolink_usart_get_msg.controller_gamepad);
    reset_controller_box_2023(&robolink_usart_get_msg.controller_box_2023);
    reset_controller_gamepad_web(&robolink_usart_get_msg.controller_gamepad_web);
    uint8_t* robolink_usart_rx_data = rt_malloc(robolink_usart_rx_max_len);
    rt_thread_mdelay(500);
    for(;;)
    {
        rt_memset(robolink_usart_rx_data, 0, robolink_usart_rx_max_len);
        int get_len = Uart1HalReceiveWaitDataItStopIdle((uint8_t*)robolink_usart_rx_data, robolink_usart_rx_max_len);
        if(robolink_data_check(robolink_usart_rx_data, get_len))
        {
            if(robolink_is_match_id(get_config()->robolink_id_config.remote_id, 0x05, 0x02, robolink_usart_rx_data, get_len)) {
                robolink_process_raw_controller_gamepad(robolink_usart_rx_data + 6, &robolink_usart_get_msg.controller_gamepad);
                ControllerNewMsgUpdate(C_UART_ROBOLINK_GAMEPAD);
                LOG_D("u gamepad %d %d %d %d %d %d %d %d %d",
                        robolink_usart_get_msg.controller_gamepad.dwXpos,// 左摇杆上下
                        robolink_usart_get_msg.controller_gamepad.dwYpos,// 左摇杆左右
                        robolink_usart_get_msg.controller_gamepad.dwZpos,// 右摇杆左右
                        robolink_usart_get_msg.controller_gamepad.dwRpos,// 右摇杆上下
                        robolink_usart_get_msg.controller_gamepad.dwUpos,// 左右扳机
                        robolink_usart_get_msg.controller_gamepad.dwVpos,
                        robolink_usart_get_msg.controller_gamepad.dwButtons,// X Y A B
                        robolink_usart_get_msg.controller_gamepad.dwButtonNumber,
                        robolink_usart_get_msg.controller_gamepad.dwPOV);// 方向键上下摄像头俯仰
            }

            if(robolink_is_match_id(get_config()->robolink_id_config.remote_id, 0x05, 0x01, robolink_usart_rx_data, get_len)) {
                robolink_process_raw_controller_box_2023(robolink_usart_rx_data + 6, &robolink_usart_get_msg.controller_box_2023);
                ControllerNewMsgUpdate(C_UART_ROBOLINK_CONTROLLER_BOX2023);
                LOG_D("u box %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d",
                        robolink_usart_get_msg.controller_box_2023.x_pos,// 左摇杆左右
                        robolink_usart_get_msg.controller_box_2023.y_pos,// 左摇杆前后
                        robolink_usart_get_msg.controller_box_2023.z_pos,// 右摇杆上下
                        robolink_usart_get_msg.controller_box_2023.r_pos,// 右摇杆左右
                        robolink_usart_get_msg.controller_box_2023.u_pos,// 左拨杆
                        robolink_usart_get_msg.controller_box_2023.v_pos,// 右拨杆
                        robolink_usart_get_msg.controller_box_2023.knob1[0],
                        robolink_usart_get_msg.controller_box_2023.knob1[1],
                        robolink_usart_get_msg.controller_box_2023.knob1[2],
                        robolink_usart_get_msg.controller_box_2023.knob1[3],
                        robolink_usart_get_msg.controller_box_2023.knob1[4],
                        robolink_usart_get_msg.controller_box_2023.knob2[0],
                        robolink_usart_get_msg.controller_box_2023.knob2[1],
                        robolink_usart_get_msg.controller_box_2023.knob2[2],
                        robolink_usart_get_msg.controller_box_2023.on_off[0],
                        robolink_usart_get_msg.controller_box_2023.on_off[1],
                        robolink_usart_get_msg.controller_box_2023.on_off[2]);
            }

        }
    }
    rt_free(robolink_usart_rx_data);
    return ;
}

void ControllerSbusRcThreadEntry(void* parameter)
{
    sbus_rc_reset(&sbus_rc_msg);
    reset_controller_gamepad(&robolink_usart_get_msg.controller_gamepad);
    reset_controller_box_2023(&robolink_usart_get_msg.controller_box_2023);
    reset_controller_gamepad_web(&robolink_usart_get_msg.controller_gamepad_web);
    uint8_t* sbus_rc_rx_data = rt_malloc(sbus_rc_rx_max_len);
    rt_thread_mdelay(1000);
    for( ; ; ) {
        rt_memset(sbus_rc_rx_data, 0, sbus_rc_rx_max_len);
        int get_len = Uart1HalReceiveWaitDataItStopIdle((uint8_t*)sbus_rc_rx_data, sbus_rc_rx_max_len);
        if(!sbus_rc_check_parse(sbus_rc_rx_data, get_len, &sbus_rc_msg))  // 检查并解析是否是SBUS遥控器数据
        {
            if(sbus_rc_msg.flag == 0)  // 如果遥控器数据有效，flag会等于0
                ControllerNewMsgUpdate(C_UART_RC_FORCE);
            LOG_D("u SBUS head: %d flag: %d end: %d", sbus_rc_msg.head, sbus_rc_msg.flag, sbus_rc_msg.end);
            LOG_D("u SBUS l1  %4d, %4d, %4d, %4d", sbus_rc_msg.ch[0], sbus_rc_msg.ch[1], sbus_rc_msg.ch[2], sbus_rc_msg.ch[3]);
            LOG_D("u SBUS l2  %4d, %4d, %4d, %4d", sbus_rc_msg.ch[4], sbus_rc_msg.ch[5], sbus_rc_msg.ch[6], sbus_rc_msg.ch[7]);
            LOG_D("u SBUS l3  %4d, %4d, %4d, %4d", sbus_rc_msg.ch[8], sbus_rc_msg.ch[9], sbus_rc_msg.ch[10], sbus_rc_msg.ch[11]);
            LOG_D("u SBUS l4  %4d, %4d, %4d, %4d", sbus_rc_msg.ch[12], sbus_rc_msg.ch[13], sbus_rc_msg.ch[14], sbus_rc_msg.ch[15]);
            continue;
        }
    }
}

