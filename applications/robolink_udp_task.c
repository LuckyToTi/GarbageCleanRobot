#include "robolink_udp_task.h"
#include <stm32f4xx_hal.h>
#include <sys/socket.h>
#include <stdio.h>
#include <string.h>
#include "config.h"
#include "controller_switch_thread.h"

#define LOG_TAG     "RUT"
#define LOG_LVL     LOG_LVL_INFO
#include <ulog.h>

static int robolink_socket_fd_ = -1;
static rt_mutex_t rl_socket_fd_mutex_ = RT_NULL;  // robolink_socket_fd_的访问互斥锁
static struct sockaddr_in robolink_local_;
static struct sockaddr_in robolink_remote_;

static rt_mailbox_t robolink_udp_msg_tx_mb_ = RT_NULL;  // 发送邮箱
static rt_thread_t rl_udp_heartbeat_tid_ = RT_NULL;
rt_thread_t udp_msg_rx_tid = RT_NULL;
rt_thread_t udp_msg_tx_tid = RT_NULL;
void RobolinkUdpHeartbeatPacketSendThreadEntry(void* parameter);
void UdpMsgRxThreadEntry(void* parameter);
void UdpMsgTxThreadEntry(void* parameter);
static void UdpMsgRxProcess(uint8_t* buffer, int len);

struct RobolinkGetMsg robolink_udp_get_msg = {};
inline const struct RobolinkGetMsg* get_robolink_udp_get_msg()
{
    return &robolink_udp_get_msg;
}

// 以太网线程控制块
rt_thread_t robolink_udp_tid = RT_NULL;
inline rt_thread_t* get_robolink_udp_tid()
{
    return &robolink_udp_tid;
}

void RobolinkUdpThreadEntry(void* parameter)
{
    robolink_socket_fd_ = socket(AF_INET, SOCK_DGRAM, 0);

    memset(&robolink_local_, '\0', sizeof(robolink_local_)); //先将内容清空
    robolink_local_.sin_family = AF_INET;          // 协议家族
    robolink_local_.sin_port = htons(get_config()->robolink_udp_config.local_port);
    robolink_local_.sin_addr.s_addr = INADDR_ANY;  //INADDR_ANY实际就是被#defind定义为0x0000000

    if ( bind(robolink_socket_fd_, (struct sockaddr*)&robolink_local_, sizeof(robolink_local_)) < 0) {
        return ;
    }

    memset(&robolink_remote_, '\0', sizeof(robolink_remote_));              // 先将内容清空
    robolink_remote_.sin_family = AF_INET;                        // 协议家族
    robolink_remote_.sin_port = htons(get_config()->robolink_udp_config.remote_port);
    robolink_remote_.sin_addr.s_addr = inet_addr(get_config()->robolink_udp_config.remote_ip_addr);  // INADDR_ANY实际就是被#defind定义为0x0000000

    rl_socket_fd_mutex_ = rt_mutex_create("rl_u_l", RT_IPC_FLAG_PRIO);
    robolink_udp_msg_tx_mb_ = rt_mb_create("RL_m_t", 16, RT_IPC_FLAG_PRIO);

    // 启动以太网处理任务
    udp_msg_rx_tid = rt_thread_create("uMsg_rx", UdpMsgRxThreadEntry, (void*)0, 2048, 16, 5);
    if (udp_msg_rx_tid != RT_NULL) rt_thread_startup(udp_msg_rx_tid);
    udp_msg_tx_tid = rt_thread_create("uMsg_tx", UdpMsgTxThreadEntry, (void*)0, 1024 + 512, 17, 5);
    if (udp_msg_tx_tid != RT_NULL) rt_thread_startup(udp_msg_tx_tid);

    rl_udp_heartbeat_tid_ = rt_thread_create("rl_u_h", RobolinkUdpHeartbeatPacketSendThreadEntry, (void*)0, 1536, 30, 5);
    if (rl_udp_heartbeat_tid_ != RT_NULL) rt_thread_startup(rl_udp_heartbeat_tid_);
}

// 500ms 定时发送心跳包
void RobolinkUdpHeartbeatPacketSendThreadEntry(void* parameter)
{
    // 初始化心跳包
    struct robolink_raw_msg* heartbeat_packet_msg =
            robolink_create_init_new_msg(1, get_config()->robolink_id_config.local_id, 0x00, 0x00);
    rt_memset(robolink_get_data_segment_ptr(heartbeat_packet_msg), 0x00, 1);

    for(;;) {
        rt_thread_delay(500);
        robolink_generate_cnt_autoincrement(heartbeat_packet_msg);
        robolink_generate_crc(heartbeat_packet_msg);
        int err_ret = rt_mutex_take(rl_socket_fd_mutex_, RT_WAITING_FOREVER);
        if(RT_EOK == err_ret) {
            sendto(robolink_socket_fd_, heartbeat_packet_msg->msg, heartbeat_packet_msg->len,
                    0, (struct sockaddr*)&robolink_remote_, sizeof(robolink_remote_));
            rt_mutex_release(rl_socket_fd_mutex_);
        }
    }
    robolink_free_unused_msg(heartbeat_packet_msg);
    return;
}

// UDP 机器人信息接受任务
void UdpMsgRxThreadEntry(void* parameter)
{
    reset_controller_gamepad(&robolink_udp_get_msg.controller_gamepad);
    reset_controller_box_2023(&robolink_udp_get_msg.controller_box_2023);
    reset_controller_gamepad_web(&robolink_udp_get_msg.controller_gamepad_web);
    while((robolink_udp_msg_tx_mb_ == RT_NULL) && (robolink_socket_fd_!=-1)) {
        rt_thread_mdelay(500);
    }

    for( ; ; )
    {
        static const int robolink_max_len = 255 + 8;
        struct sockaddr_in recv_addr;
        unsigned int recv_addr_length = sizeof(recv_addr);
        int recv_length = 0;
        char recv_line[robolink_max_len];  // robolink最大数据包大小为 255byte + 标志位固定 8byte
        // memset(recv_line, 0, robolink_max_len);

        recv_length = recvfrom(robolink_socket_fd_, recv_line, sizeof(recv_line), 0, (struct sockaddr *)&recv_addr, (socklen_t *)&recv_addr_length);

        if(recv_length <= 0){
            continue;  // 长度错误
        }
        if((recv_addr.sin_addr.s_addr != robolink_remote_.sin_addr.s_addr) || (recv_addr.sin_port != robolink_remote_.sin_port)) {
            continue;  // 地址端口不匹配
        }

        UdpMsgRxProcess((uint8_t *)recv_line, recv_length);
    }
    return ;
}

inline static void UdpMsgRxProcess(uint8_t* buffer, int len)
{
    if(robolink_data_check(buffer, len))
    {
        if(robolink_is_match_id(get_config()->robolink_id_config.remote_id, 0x05, 0x02, buffer, len)) {
            robolink_process_raw_controller_gamepad(buffer + 6, &robolink_udp_get_msg.controller_gamepad);
            ControllerNewMsgUpdate(C_UDP_ROBOLINK_GAMEPAD);
            LOG_D("gamepad %d %d %d %d %d %d %d %d %d",
                    robolink_udp_get_msg.controller_gamepad.dwXpos,// 左摇杆左右 右正
                    robolink_udp_get_msg.controller_gamepad.dwYpos,// 左摇杆上下 下正
                    robolink_udp_get_msg.controller_gamepad.dwZpos,// 左右扳机和 左扳机正
                    robolink_udp_get_msg.controller_gamepad.dwRpos,// 右摇杆上下 下正
                    robolink_udp_get_msg.controller_gamepad.dwUpos,// 右摇杆左右 右正
                    robolink_udp_get_msg.controller_gamepad.dwVpos,
                    robolink_udp_get_msg.controller_gamepad.dwButtons,// 按键
                    robolink_udp_get_msg.controller_gamepad.dwButtonNumber,
                    robolink_udp_get_msg.controller_gamepad.dwPOV);// 方向键上下摄像头俯仰
        }
        if(robolink_is_match_id(get_config()->robolink_id_config.remote_id, 0x05, 0x01, buffer, len)) {
            robolink_process_raw_controller_box_2023(buffer + 6, &robolink_udp_get_msg.controller_box_2023);
            ControllerNewMsgUpdate(C_UDP_ROBOLINK_CONTROLLER_BOX2023);
            LOG_D("box %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d",
                    robolink_udp_get_msg.controller_box_2023.x_pos,// 左摇杆左右
                    robolink_udp_get_msg.controller_box_2023.y_pos,// 左摇杆前后
                    robolink_udp_get_msg.controller_box_2023.z_pos,// 右摇杆上下
                    robolink_udp_get_msg.controller_box_2023.r_pos,// 右摇杆左右
                    robolink_udp_get_msg.controller_box_2023.u_pos,// 左拨杆
                    robolink_udp_get_msg.controller_box_2023.v_pos,// 右拨杆
                    robolink_udp_get_msg.controller_box_2023.knob1[0],
                    robolink_udp_get_msg.controller_box_2023.knob1[1],
                    robolink_udp_get_msg.controller_box_2023.knob1[2],
                    robolink_udp_get_msg.controller_box_2023.knob1[3],
                    robolink_udp_get_msg.controller_box_2023.knob1[4],
                    robolink_udp_get_msg.controller_box_2023.knob2[0],
                    robolink_udp_get_msg.controller_box_2023.knob2[1],
                    robolink_udp_get_msg.controller_box_2023.knob2[2],
                    robolink_udp_get_msg.controller_box_2023.on_off[0],
                    robolink_udp_get_msg.controller_box_2023.on_off[1],
                    robolink_udp_get_msg.controller_box_2023.on_off[2]);
        }

        if(robolink_is_match_id(get_config()->robolink_id_config.remote_id, 0x05, 0x04, buffer, len)) {
            robolink_process_raw_controller_gamepad_web(buffer + 6, &robolink_udp_get_msg.controller_gamepad_web);
            ControllerNewMsgUpdate(C_UDP_ROBOLINK_GAMEPAD_WEB);
            LOG_D("gamepad_web %d %d %d %d %d %d %d",
                    robolink_udp_get_msg.controller_gamepad_web.axes[0],// 左摇杆左右
                    robolink_udp_get_msg.controller_gamepad_web.axes[1],// 左摇杆前后
                    robolink_udp_get_msg.controller_gamepad_web.axes[2],// 右摇杆上下
                    robolink_udp_get_msg.controller_gamepad_web.axes[3],// 右摇杆左右
                    robolink_udp_get_msg.controller_gamepad_web.buttons,// 左拨杆
                    robolink_udp_get_msg.controller_gamepad_web.button6,// 右拨杆
                    robolink_udp_get_msg.controller_gamepad_web.button7);
        }

    }
}

// UDP 机器人信息发送任务
void UdpMsgTxThreadEntry(void* parameter)
{
    while((robolink_udp_msg_tx_mb_ == RT_NULL) && (robolink_socket_fd_!=-1)) {
        rt_thread_mdelay(500);
    }
    for( ; ; )
    {
        struct robolink_raw_msg* new_msg = RT_NULL;
        if (rt_mb_recv(robolink_udp_msg_tx_mb_, (rt_ubase_t *)(&new_msg), RT_WAITING_FOREVER) == RT_EOK)
        {
            if(new_msg != RT_NULL)
            {
                robolink_generate_cnt_autoincrement(new_msg);
                robolink_generate_crc(new_msg);
                int err_ret = rt_mutex_take(rl_socket_fd_mutex_, RT_WAITING_FOREVER);
                if(RT_EOK == err_ret) {
                    sendto(robolink_socket_fd_, new_msg->msg, new_msg->len,
                            0, (struct sockaddr*)&robolink_remote_, sizeof(robolink_remote_));
                    rt_mutex_release(rl_socket_fd_mutex_);
                }
                robolink_free_unused_msg(new_msg);
            }
        }
    }
    return ;
}

/**
 * 发送一个新的robolink消息至邮箱，由发送任务自动发送
 * @param data_ptr 数据段头指针
 * @param data_len 数据段长度
 * @param sys_id   系统id
 * @param dev_id   设备id
 * @param data_id  数据（标识）id
 */
void RobolinkUdpCreateInitSendNewMsgToMb(uint8_t* data_ptr, uint8_t data_len,
        uint8_t sys_id, uint8_t dev_id, uint8_t data_id)
{
    struct robolink_raw_msg* new_msg = robolink_create_init_new_msg(data_len,
            sys_id, dev_id, data_id);
    if((new_msg != NULL) && (robolink_udp_msg_tx_mb_ != RT_NULL))
    {
        rt_memcpy(robolink_get_data_segment_ptr(new_msg), data_ptr, data_len);
        if(RT_EOK != rt_mb_send(robolink_udp_msg_tx_mb_, (rt_ubase_t)new_msg))
            robolink_free_unused_msg(new_msg);
    } else {
    	robolink_free_unused_msg(new_msg);
	}
}

//int print_rl_udp(const char *format, ...)
//{
//    char* buf_ = rt_malloc(1024 * sizeof(char));
//    int len_ = 0;
//    va_list args;
//    va_start(args, format);
//    vsprintf(buf_, format, args);
//    va_end(args);
//    len_ = strlen(buf_);
//    if(len_ > 255) len_ = 255;
//    RobolinkUdpCreateInitSendNewMsgToMb((uint8_t*)buf_, len_, get_config()->robolink_id_config.local_id, 0xFF, 0x01);
//    rt_free(buf_);
//    return 0;
//}

int print_rl_udp(uint8_t l_vl, const char *format, ...)
{
    char* buf_ = rt_malloc(1024 * sizeof(char));
    int len_ = 0;
    va_list args;
    va_start(args, format);
    vsprintf(buf_, format, args);
    va_end(args);
    len_ = strlen(buf_);
    if(len_ > 255) len_ = 255;
    RobolinkUdpCreateInitSendNewMsgToMb((uint8_t*)buf_, len_, get_config()->robolink_id_config.local_id, 0xFF, l_vl);
    rt_free(buf_);
    return 0;
}
