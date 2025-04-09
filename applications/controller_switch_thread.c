#include "controller_switch_thread.h"
#include <string.h>
#include "hardware/led.h"
#include "robolink_udp_task.h"
#include "robolink_usart_task.h"
#include "userlib/robot_msg_json.h"
#include "movelib/rov_move.h"

#define LOG_TAG     "c_s_t"
#define LOG_LVL     LOG_LVL_INFO
#include <ulog.h>

struct ControlMsg control_msg;
inline const struct ControlMsg* get_control_msg() {
    return &control_msg;
}

inline struct ControlMsg* get_control_msg_to_change() {
    return &control_msg;
}

void reset_control_msg(struct ControlMsg* msg) {
    reset_robot_comm_motion(&msg->robot_common_motion);
    reset_rov_control(&msg->rov_control);
    reset_ship_control(&msg->ship_control);
}

// 操作controller_select_id_的所有函数
static const rt_int32_t controller_select_id_mutex_wait_time_ms_ = 200;
static rt_mutex_t controller_select_id_mutex_ = RT_NULL;  // controller_select_id_的锁
static uint32_t controller_select_id_ = 0;
static rt_err_t get_controller_select_id_(uint32_t* id) {
    int err_ret = rt_mutex_take(controller_select_id_mutex_, controller_select_id_mutex_wait_time_ms_);
    if(RT_EOK == err_ret){
        *id = controller_select_id_;
        rt_mutex_release(controller_select_id_mutex_);
    }
    return err_ret;
}
static rt_err_t set_controller_select_id_(const uint32_t id) {
    int err_ret = rt_mutex_take(controller_select_id_mutex_, controller_select_id_mutex_wait_time_ms_);
    if(RT_EOK == err_ret){
        controller_select_id_ = id;
        rt_mutex_release(controller_select_id_mutex_);
    }
    return err_ret;
}

static struct SbusRc sbus_rc_msg_last = {};
static struct RobolinkGetMsg robolink_udp_get_msg_last = {};
static struct RobolinkGetMsg robolink_usart_get_msg_last = {};
static void controller_clear_all_last_() {
    sbus_rc_reset(&sbus_rc_msg_last);
    reset_controller_gamepad(&robolink_udp_get_msg_last.controller_gamepad);
    reset_controller_box_2023(&robolink_udp_get_msg_last.controller_box_2023);
    reset_controller_gamepad_web(&robolink_udp_get_msg_last.controller_gamepad_web);
    reset_controller_gamepad(&robolink_usart_get_msg_last.controller_gamepad);
    reset_controller_box_2023(&robolink_usart_get_msg_last.controller_box_2023);
    reset_controller_gamepad_web(&robolink_usart_get_msg_last.controller_gamepad_web);
}

// 间隔时间
static uint32_t controller_recv_cnt_interval_ms_ = 2000;

// 一段时间内内各个设备获取的消息包数量
static int32_t controller_recv_cnt_[CONTROLLER_TYPE_NUM] = {0};

// 收到数据消息包数
static rt_mailbox_t controller_msg_process_mb_ = RT_NULL;  // 发送邮箱

static rt_thread_t controller_msg_process_tid = RT_NULL;  // 处理线程的线程控制块
static void ControllerMsgProcessThreadEntry(void* parameter);
static void ProcessControllerMsg(uint8_t controller_get_which);
static void ControlDataRegister();

// 设备搜到一个数据包执行这个函数一次
void ControllerNewMsgUpdate(const uint32_t id)
{
    if(id >= CONTROLLER_TYPE_NUM) return;
    rt_mb_send(controller_msg_process_mb_, (rt_ubase_t)id);
    controller_recv_cnt_[id] ++;
}

// controller_switch线程控制块
rt_thread_t controller_switch_tid = RT_NULL;
inline rt_thread_t* get_controller_switch_tid()
{
    return &controller_switch_tid;
}

// 根据每秒控制器接收数据数量仲裁控制器选择
void ControllerRecvCntProcessThreadEntry(void* parameter)
{
    controller_select_id_mutex_ = rt_mutex_create("ClSlMt", RT_IPC_FLAG_PRIO);
    controller_msg_process_mb_  = rt_mb_create("ClMPcMb", CONTROLLER_TYPE_NUM*2, RT_IPC_FLAG_PRIO);
    for(int _i=0; _i<CONTROLLER_TYPE_NUM; _i++) {
        controller_recv_cnt_[_i] = 0;
    }
    controller_msg_process_tid = rt_thread_create("CtMgPT", ControllerMsgProcessThreadEntry, (void*)0, 2048, 10, 10);
    if (controller_msg_process_tid != RT_NULL) rt_thread_startup(controller_msg_process_tid);
    rt_thread_delay(200);
    for( ; ; ) {

        // 获取当前数据
        uint32_t now_choose_dev = 0;  // 当前选择设备
        int now_choose_num = -1;  // 当前选择设备
        uint32_t now_max_dev = 0;  // 当前接收数据包最多设备
        int now_max_num = -1;  // 当前接收数据包数量

        // 通过更改controller_recv_cnt_数据，达到强制切换控制器的
        // TODO

        // 自动仲裁处理
        for(int _i=0; _i<CONTROLLER_TYPE_NUM; _i++) {
            if(controller_recv_cnt_[_i] > now_max_num) {
                now_max_num = controller_recv_cnt_[_i];
                now_max_dev = _i;
            }
        }
        int err_ret = get_controller_select_id_(&now_choose_dev);
        if(err_ret != RT_EOK) continue;
        now_choose_num = controller_recv_cnt_[now_choose_dev];

        // LOG_D("d_g num: %d %d %d %d %d %d", controller_recv_cnt_[0], controller_recv_cnt_[1], controller_recv_cnt_[2], controller_recv_cnt_[3], controller_recv_cnt_[4], controller_recv_cnt_[5]);
        // LOG_D("%d %d %d %d", now_choose_dev, now_choose_num, now_max_dev, now_max_num);

        if(now_choose_num == 0) {
            reset_control_msg(&control_msg);
        }

        // 执行控制源切换
        if((now_choose_num < 15) && (now_choose_num < now_max_num)){
            set_controller_select_id_(now_max_dev);
        }

        // 数据包清零
        for(int _i=0; _i<CONTROLLER_TYPE_NUM; _i++) {
            controller_recv_cnt_[_i] = 0;
        }
        rt_thread_delay(controller_recv_cnt_interval_ms_);
    }
}

static void ControllerMsgProcessThreadEntry(void* parameter)
{
    uint32_t controller_select_id_last = 0;
    controller_clear_all_last_();
    reset_control_msg(&control_msg);
    ControlDataRegister();
    rt_thread_delay(200);
    for( ; ; )
    {
        uint32_t controller_select_id_now = 0;
        int err_ret = get_controller_select_id_(&controller_select_id_now);
        if(err_ret != RT_EOK) continue;

        if(controller_select_id_last != controller_select_id_now) {
            controller_select_id_last = controller_select_id_now;
            controller_clear_all_last_();
            reset_control_msg(&control_msg);
            LOG_I("ControlGet now changed to NO.%d device.", controller_select_id_now);
            print_rl_udp(0x02, "ControlGet now changed to NO.%d device.", controller_select_id_now);
        }

        uint32_t recv_msg_dev_id_now = 0;
        if (rt_mb_recv(controller_msg_process_mb_, (rt_ubase_t *)(&recv_msg_dev_id_now), 1500) == RT_EOK) {
            if(recv_msg_dev_id_now == controller_select_id_now) {
                ProcessControllerMsg(controller_select_id_now);
                LedRgbWrite('g', 1);
                // 打印调试信息
                LOG_D("robot_common_motion: %.5f, %.5f, %.5f, %.5f, ", control_msg.robot_common_motion.x, control_msg.robot_common_motion.y, control_msg.robot_common_motion.z, control_msg.robot_common_motion.revolve_z);
                LOG_D("rov_control: %.2f %d %d %d %.2f %.2f %.2f  %.2f", control_msg.rov_control.z_force, control_msg.rov_control.roll_hold,
                        control_msg.rov_control.yaw_hold, control_msg.rov_control.depth_hold,
                        control_msg.rov_control.led_brightness, control_msg.rov_control.camera_pitch,
                        control_msg.rov_control.gripper_angle, control_msg.rov_control.gripper_speed);
            }
        } else {
            reset_control_msg(&control_msg);
            LedRgbWrite('g', 0);
        }
    }
}

static void ProcessControllerMsg(uint8_t controller_id)
{
    switch(controller_id) {

    case C_UART_RC_FORCE:
        sbus_to_robot_comm_motion(get_sbus_rc_msg(), &control_msg.robot_common_motion);
        sbus_to_ship_control(get_sbus_rc_msg(), &sbus_rc_msg_last, &control_msg.rov_control);
        memcpy(&sbus_rc_msg_last, get_sbus_rc_msg(), sizeof(struct SbusRc));
        break;

    case C_UDP_ROBOLINK_GAMEPAD:
        gamepad_to_robot_comm_motion(&get_robolink_udp_get_msg()->controller_gamepad, &control_msg.robot_common_motion);
        gamepad_to_rov_control(&get_robolink_udp_get_msg()->controller_gamepad, &robolink_udp_get_msg_last.controller_gamepad, &control_msg.rov_control);
        memcpy(&robolink_udp_get_msg_last.controller_gamepad, &get_robolink_udp_get_msg()->controller_gamepad, sizeof(struct ControllerGamepad));
        break;

    case C_UDP_ROBOLINK_CONTROLLER_BOX2023:
        box_2023_to_robot_comm_motion(&get_robolink_udp_get_msg()->controller_box_2023, &control_msg.robot_common_motion);
        box_2023_to_rov_control(&get_robolink_udp_get_msg()->controller_box_2023, &control_msg.rov_control);
        memcpy(&robolink_udp_get_msg_last.controller_box_2023, &get_robolink_udp_get_msg()->controller_box_2023, sizeof(struct ControllerBox2023));
        break;

    case C_UDP_ROBOLINK_GAMEPAD_WEB:
        gamepad_web_to_robot_comm_motion(&get_robolink_udp_get_msg()->controller_gamepad_web, &control_msg.robot_common_motion);
        gamepad_web_to_ship_control(&get_robolink_udp_get_msg()->controller_gamepad_web, &robolink_udp_get_msg_last.controller_gamepad_web, &control_msg.ship_control);
        memcpy(&robolink_udp_get_msg_last.controller_gamepad_web, &get_robolink_udp_get_msg()->controller_gamepad_web, sizeof(struct ControllerGamepadWeb));
        break;

    case C_UART_ROBOLINK_GAMEPAD:
        gamepad_to_robot_comm_motion(&get_robolink_usart_get_msg()->controller_gamepad, &control_msg.robot_common_motion);
        gamepad_to_rov_control(&get_robolink_usart_get_msg()->controller_gamepad, &robolink_usart_get_msg_last.controller_gamepad, &control_msg.rov_control);
        memcpy(&robolink_usart_get_msg_last.controller_gamepad, &get_robolink_usart_get_msg()->controller_gamepad, sizeof(struct ControllerGamepad));
        break;

    case C_UART_ROBOLINK_CONTROLLER_BOX2023:
        box_2023_to_robot_comm_motion(&get_robolink_usart_get_msg()->controller_box_2023, &control_msg.robot_common_motion);
        box_2023_to_rov_control(&get_robolink_usart_get_msg()->controller_box_2023, &control_msg.rov_control);
        memcpy(&robolink_usart_get_msg_last.controller_box_2023, &get_robolink_usart_get_msg()->controller_box_2023, sizeof(struct ControllerBox2023));
        break;

    default:
        break;
    }
}

static void ControlDataRegister()
{
//    RobotMsgJsonRegister("control_data.rov_control.roll_hold", J_T_Bool,
//                    &control_msg.rov_control.roll_hold, J_P_RW);
//    RobotMsgJsonRegister("control_data.rov_control.pitch_hold", J_T_Bool,
//                    &control_msg.rov_control.pitch_hold, J_P_RW);
//    RobotMsgJsonRegister("control_data.rov_control.yaw_hold", J_T_Bool,
//                    &control_msg.rov_control.yaw_hold, J_P_RW);
//    RobotMsgJsonRegister("control_data.rov_control.depth_hold", J_T_Bool,
//                    &control_msg.rov_control.depth_hold, J_P_RW);
    RobotMsgJsonRegister("control_data.rov_control.position_hold", J_T_Bool,
                    &control_msg.rov_control.position_hold, J_P_RW);
    RobotMsgJsonRegister("control_data.rov_control.position_head_move", J_T_Bool,
                    &control_msg.rov_control.position_head_move, J_P_RW);
//    RobotMsgJsonRegister("control_data.rov_control.line_hold", J_T_Bool,
//                    &control_msg.rov_control.line_hold, J_P_RW);

//    RobotMsgJsonRegister("control_data.rov_control.led_brightness", J_T_NumberFloat,
//                &control_msg.rov_control.led_brightness, J_P_RW);
//    RobotMsgJsonRegister("control_data.rov_control.camera_pitch", J_T_NumberFloat,
//                &control_msg.rov_control.camera_pitch, J_P_RW);
//    RobotMsgJsonRegister("control_data.rov_control.cloud_platform_revolve", J_T_NumberFloat,
//                &control_msg.rov_control.cloud_platform_revolve, J_P_RW);
//    RobotMsgJsonRegister("control_data.rov_control.cloud_platform_pitch", J_T_NumberFloat,
//                &control_msg.rov_control.cloud_platform_pitch, J_P_RW);
//    RobotMsgJsonRegister("control_data.rov_control.gripper_angle", J_T_NumberFloat,
//                &control_msg.rov_control.gripper_angle, J_P_RW);
//    RobotMsgJsonRegister("control_data.rov_control.gripper_speed", J_T_NumberFloat,
//                &control_msg.rov_control.gripper_speed, J_P_RW);
    RobotMsgJsonRegister("control_data.equip_switch.sonar", J_T_Bool,
                &control_msg.equip_switch.sonar, J_P_RW);


//	RobotMsgJsonRegister("move_tar.roll_tar", J_T_NumberFloat,
//	        	&get_move_tar()->roll_tar, J_P_RW);
//	RobotMsgJsonRegister("move_tar.pitch_tar", J_T_NumberFloat,
//	        	&get_move_tar()->pitch_tar, J_P_RW);
//	RobotMsgJsonRegister("move_tar.yaw_hold_tar", J_T_NumberFloat,
//				&get_move_tar()->yaw_hold_tar, J_P_RW);
//	RobotMsgJsonRegister("move_tar.depth_fixed_tar", J_T_NumberFloat,
//				&get_move_tar()->depth_fixed_tar, J_P_RW);
//	RobotMsgJsonRegister("move_tar.line_tar", J_T_NumberFloat,
//				&get_move_tar()->line_tar, J_P_RW);
//	RobotMsgJsonRegister("move_tar.line_yaw_tar", J_T_NumberFloat,
//				&get_move_tar()->line_yaw_tar, J_P_RW);
	RobotMsgJsonRegister("move_tar.Dvl_x_tar", J_T_NumberFloat,
	        	&get_move_tar()->Dvl_x_tar, J_P_RW);
	RobotMsgJsonRegister("move_tar.Dvl_y_tar", J_T_NumberFloat,
	        	&get_move_tar()->Dvl_y_tar, J_P_RW);
	RobotMsgJsonRegister("move_tar.head_move_x_tar", J_T_NumberFloat,
	        	&get_move_tar()->head_move_x_tar, J_P_RW);
	RobotMsgJsonRegister("move_tar.head_move_y_tar", J_T_NumberFloat,
	        	&get_move_tar()->head_move_y_tar, J_P_RW);
	RobotMsgJsonRegister("move_tar.head_move_yaw", J_T_NumberFloat,
	        	&get_move_tar()->head_move_yaw, J_P_RW);
}
