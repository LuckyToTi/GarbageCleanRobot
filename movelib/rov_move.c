#include "rov_move.h"
#include "math.h"
#include "userlib/math2.h"
#include "userlib/pid2.h"
#include "gyro_thread.h"
#include "xsens_ahrs_thread.h"
#include "wd_thread.h"
#include "dvl_json_tcp_thread.h"
#include "userlib/imu_data.h"

#define LOG_TAG     "r_e_p"
#define LOG_LVL     LOG_LVL_INFO
#include <ulog.h>

#define X_OFFSET (0)
#define Y_OFFSET (1)
#define Z_OFFSET (2)
#define ROLL_X_OFFSET (3)
#define PITCH_Y_OFFSET (4)
#define YAW_Z_OFFSET (5)

#if USE_IMU_DATA == 1 || USE_IMU_DATA == 0
#define roll_x_value (-get_bsp_gyro_data()->euler_angle.roll_x)
#define pitch_y_value (-get_bsp_gyro_data()->euler_angle.pitch_y)
#define yaw_z_value (get_bsp_gyro_data()->euler_angle.yaw_z)
#elif USE_IMU_DATA == 2
#define roll_x_value (get_mt_data_2()->euler_angle.roll_x)
#define pitch_y_value (get_mt_data_2()->euler_angle.pitch_y)
#define yaw_z_value (get_mt_data_2()->euler_angle.yaw_z)
#endif

#define water_depth_m (Get_Water_Depth() / 1000.0f)
#define dvl_x_value (get_dvl_position_local_data_fixed()->x)
#define dvl_y_value (-get_dvl_position_local_data_fixed()->y)
#define dvl_yaw_value (get_dvl_yaw())
#define line_dvl_x_value (dvl_x_value)
#define line_dvl_y_value (get_dvl_position_local_data()->y)
#define line_cal_dvl_value (GetAngleOver360(dvl_yaw_value))
//#define dvl_yaw_value (-get_dvl_position_local_data()->yaw)

static const struct {
    float roll_hold_s;
    float pitch_hold_s;
    float yaw_hold_s;
    float depth_hold_s;
    float x_hold_s;
    float y_hold_s;
    float line_hold_s;
} CtrlScale = { 1.0, 1.0, 0.8, 1.0, 1.0, 1.0, 0.8};

static struct Targets move_tar = { 0, 0, 0, 0, 0, 0, 0, 0};
struct Targets* get_move_tar() {
    return &move_tar;
}

static int IsSwitchChanged(uint8_t key, uint8_t* last_key);
void CalcHeadMoveXY(float x_ori, float y_ori, float yaw, float x_to, float y_to, float *x1, float *y1);
void TargetSlowlyIncrease(float cur, float* tar, float diff);
float CalculateDistance(float x1, float y1, float x2, float y2);

void MoveAutoAndManual(struct MotionSpeed* motion_speed, const struct ControlMsg* ctrl_msg, float T) {
    static uint8_t last_DVL_reset = 0;
    static uint8_t last_depth_hold = 0, last_yaw_hold = 0, last_position_hold = 0, last_position_head_move = 0, last_line_hold = 0; // 记录初始按钮
    static uint8_t head_moveing_flag = 0;
    static float x1y_yaw = 0;
    static float last_move_x_tar = 0;
    static float last_move_y_tar = 0;

//  -----参考点位移功能相关函数(基于非直角坐标系)-----
    static float x1y_x = 0, x1y_y = 0; // 点B在x1坐标系中的位置
    static float x2y_x;
    static float x2y_y;
//  --------------------------------------------------

//    static float line_hold_x_ori = 0, line_hold_y_ori = 0;
//    static float line_slope = 0.0f;
//    static float line_b = 0;

    float outs_manual[6] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0}; // x y z roll_x pitch_y yaw_z
    float outs_auto[6] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0}; // x y z roll_x pitch_y yaw_z

    // reset DVL
    struct EulerAngle final_euler,origin_euler;
    origin_euler.roll_x = roll_x_value;
    origin_euler.pitch_y = pitch_y_value;
    origin_euler.yaw_z = yaw_z_value;

    EulerModeChange(ctrl_msg->rov_control.attitude_type, origin_euler, &final_euler); // 姿态类型转换
    uint8_t dlv_reset_button = (((get_robolink_udp_get_msg()->controller_gamepad).dwButtons & XBOX_VIEW) == XBOX_VIEW) ? 1u:0u;
    int is_dlv_reset_button_changed = IsSwitchChanged(dlv_reset_button, &last_DVL_reset);
    if(is_dlv_reset_button_changed == -1 || get_control_msg()->rov_control.reset_knob == 1) {
        StartDvlReset();
        move_tar.Dvl_x_tar = dvl_x_value;
        move_tar.Dvl_y_tar = dvl_y_value;
        move_tar.head_move_x_tar = 0;
        move_tar.head_move_y_tar = 0;
        last_move_x_tar = 0;
        last_move_y_tar = 0;
        x1y_yaw = 0;
    }

    /* roll(横滚)保持 */
    if (ctrl_msg->rov_control.roll_hold || ctrl_msg->rov_control.attitude_type != 0) {
        outs_auto[ROLL_X_OFFSET] += CtrlScale.roll_hold_s * CalcRollHold(&get_config_future()->pid2[ROV_PID_ROLL], move_tar.roll_tar, final_euler.roll_x, T);
    }

    /* pitch(俯仰)保持 */
    if(ctrl_msg->rov_control.pitch_hold || ctrl_msg->rov_control.attitude_type != 0) {
        outs_auto[PITCH_Y_OFFSET] += CtrlScale.pitch_hold_s * CalcPitchHold(&get_config_future()->pid2[ROV_PID_PITCH], move_tar.pitch_tar, final_euler.pitch_y, T);
    }

    /* 深度保持初始值 */
    int is_depth_hold_changed = IsSwitchChanged(ctrl_msg->rov_control.depth_hold, &last_depth_hold);
    if(is_depth_hold_changed == 1) {  // 开启定深（0->1）
    	move_tar.depth_fixed_tar = water_depth_m;  // 记录当前深度为目标深度
    } else if (is_depth_hold_changed == -1) {  // 关闭定深（1->0）
    	move_tar.depth_fixed_tar = 0.0;  // 记录当前深度为目标深度
    }

    /* 深度保持 */
    if(ctrl_msg->rov_control.depth_hold) {  ///< 深度传感器有效且开启定深
        // 一秒最多上浮2M或者下沉2M， 前段缓慢后段快速增加
    	move_tar.depth_fixed_tar -=  x3_Slowly_Increasing(ctrl_msg->robot_common_motion.z / 1000.0f) * T * 1.25; // （-1 - 1）  * 时间常数  * 2
    	if(ctrl_msg->robot_common_motion.z != 0){
    	    TargetSlowlyIncrease(water_depth_m, &move_tar.depth_fixed_tar, 1); // 米
    	}
        if (move_tar.depth_fixed_tar < 0) move_tar.depth_fixed_tar = 0;
        outs_auto[Z_OFFSET] -= CtrlScale.depth_hold_s
        		* CalcDepthHold(&get_config_future()->pid2[ROV_PID_DEPTH], move_tar.depth_fixed_tar, water_depth_m, T);
        if(ctrl_msg->robot_common_motion.z != 0)
            print_rl_udp(2,"Depth tar change to: %.2f.",move_tar.depth_fixed_tar);
    } else {
        outs_manual[Z_OFFSET] += ctrl_msg->robot_common_motion.z;
    }

    /* 航向保持初始值 */
    int is_yaw_hold_changed = IsSwitchChanged(ctrl_msg->rov_control.yaw_hold, &last_yaw_hold);
    if(is_yaw_hold_changed == 1) {  // 开启yaw保持（0->1）
    	move_tar.yaw_hold_tar = GetAngle360FromAngle180(yaw_z_value);  // 记录当前深度为目标深度
    } else if (is_yaw_hold_changed == -1) {
    	move_tar.yaw_hold_tar = 0.0;
    }

    /* yaw航向保持 */
    if(ctrl_msg->rov_control.yaw_hold) {
    	move_tar.yaw_hold_tar += ctrl_msg->robot_common_motion.revolve_z * T / 10.0;
    	move_tar.yaw_hold_tar = GetAngle360FromAngle180(move_tar.yaw_hold_tar);  // if yaw_hold_tar< 0 then yaw_hold_tar+=360
    	move_tar.yaw_hold_tar = (move_tar.yaw_hold_tar > 360) ? (move_tar.yaw_hold_tar - 360) : move_tar.yaw_hold_tar;
        outs_auto[YAW_Z_OFFSET] += CtrlScale.yaw_hold_s * CalcYawHold(&get_config_future()->pid2[ROV_PID_YAW],
        		move_tar.yaw_hold_tar, GetAngle360FromAngle180(yaw_z_value), T);
        if(ctrl_msg->robot_common_motion.revolve_z != 0)
        	print_rl_udp(2, "yaw tar change to: %.2f", move_tar.yaw_hold_tar);
    } else {
        outs_manual[YAW_Z_OFFSET] += ctrl_msg->robot_common_motion.revolve_z;
    }

    /* 航向保持状态下的修正运动 */
    if(ctrl_msg->rov_control.yaw_hold){
        float included_angle; // 航向角与当前角夹角
        float yaw_hold_fixed_x = 0, yaw_hold_fixed_y = 0;
        included_angle = GetAngle360FromAngle180(yaw_z_value) - move_tar.yaw_hold_tar;
        if(!ctrl_msg->rov_control.line_hold || !ctrl_msg->rov_control.position_hold)
        	CalcHeadMoveMotorsSpeed(included_angle, ctrl_msg->robot_common_motion.x, ctrl_msg->robot_common_motion.y, &yaw_hold_fixed_x, &yaw_hold_fixed_y);
        outs_manual[X_OFFSET] += yaw_hold_fixed_x;
        outs_manual[Y_OFFSET] += yaw_hold_fixed_y;
    } else {
        outs_manual[X_OFFSET] += ctrl_msg->robot_common_motion.x;
        outs_manual[Y_OFFSET] += ctrl_msg->robot_common_motion.y;
    }



    /* 直线保持  */
    int is_last_line_hold = IsSwitchChanged(ctrl_msg->rov_control.line_hold, &last_line_hold);
    if(is_last_line_hold == 1) {  // 在当前点建立假坐标系
        get_control_msg_to_change()->rov_control.position_head_move = 1;
    }

    if(ctrl_msg->rov_control.line_hold){
        move_tar.head_move_y_tar += ctrl_msg->robot_common_motion.x * T / 2000.0; // 直角坐标系
        float distance_ = CalculateDistance(-dvl_y_value, dvl_x_value, x1y_x, x1y_y);
        if(ctrl_msg->robot_common_motion.x != 0){
            TargetSlowlyIncrease(distance_, &move_tar.head_move_y_tar, 1); // 米
        }
        get_control_msg_to_change()->robot_common_motion.x = 0;
        get_control_msg_to_change()->robot_common_motion.y = 0;

    } else {

    }

    int is_position_head_move_changed = IsSwitchChanged(ctrl_msg->rov_control.position_head_move, &last_position_head_move);
    if(is_position_head_move_changed == 1) {  // 在当前点建立假坐标系
        x1y_x = -dvl_y_value;
        x1y_y = dvl_x_value;
        x1y_yaw = get_absolute_yaw_z_value(); // B位置时朝向, x2y坐标系的X轴朝向
        move_tar.Dvl_x_tar = dvl_x_value;
        move_tar.Dvl_y_tar = dvl_y_value;
        get_control_msg_to_change()->rov_control.position_head_move = 0;
        print_rl_udp(2, "Reset zero point.");
    }

    /* 定向(参考当前头)定量位移 */
    if (move_tar.head_move_x_tar != last_move_x_tar || move_tar.head_move_y_tar != last_move_y_tar) {
    	if(ctrl_msg->rov_control.position_hold != 1){
    		print_rl_udp(3, "Robot x y is changed, but position hold UNopened.");
    		move_tar.head_move_x_tar = last_move_x_tar;
    		move_tar.head_move_y_tar = last_move_y_tar;
    	}else{
    	    head_moveing_flag = 1; // 标志位用于提示用户是否到达
    	    last_move_x_tar = move_tar.head_move_x_tar; // 判断变化
            last_move_y_tar = move_tar.head_move_y_tar;
            x2y_x = move_tar.head_move_x_tar;
            x2y_y = move_tar.head_move_y_tar;
		    float x2y_to_x1y_x, x2y_to_x1y_y;
			GetX1YRealCoordinate(x1y_x, x1y_y, x2y_x, x2y_y, x1y_yaw, &x2y_to_x1y_x, &x2y_to_x1y_y);
			move_tar.Dvl_x_tar = x2y_to_x1y_y;
			move_tar.Dvl_y_tar = -x2y_to_x1y_x;
			print_rl_udp(2, "Robot move to: %.2f, %.2f", move_tar.head_move_x_tar, move_tar.head_move_y_tar);
    	}
    }

    /* 动力定位 */
    int is_position_hold_changed = IsSwitchChanged(ctrl_msg->rov_control.position_hold, &last_position_hold);
    if (is_position_hold_changed == 1) { // 获取保持角度
    	move_tar.Dvl_x_tar = dvl_x_value;
    	move_tar.Dvl_y_tar = dvl_y_value;
        print_rl_udp(2, "Robot position hold opened: %.2f, %.2f.", move_tar.Dvl_x_tar,move_tar.Dvl_y_tar);
    }
    if (ctrl_msg->rov_control.position_hold && ctrl_msg->rov_control.yaw_hold && (ctrl_msg->robot_common_motion.x == 0 && ctrl_msg->robot_common_motion.y == 0)) {
    	if(head_moveing_flag == 1 && move_tar.Dvl_x_tar - dvl_x_value < 0.2 && move_tar.Dvl_y_tar - dvl_y_value < 0.2){
    		head_moveing_flag = 0;

    		// 提示上位机 直线运动完成
    		{
    		    uint8_t test_ = 1;
                RobolinkUdpCreateInitSendNewMsgToMb(&test_, 1, get_config()->robolink_id_config.local_id, 0x10, 0x01);
    		}

    		print_rl_udp(2, "Robot x y move (%.2f, %.2f) done.", move_tar.head_move_x_tar, move_tar.head_move_y_tar);
    	}
        float dvl_x_cal = CalcDvlXHold(&get_config_future()->pid2[ROV_PID_X], move_tar.Dvl_x_tar, dvl_x_value, T);
        float dvl_y_cal = CalcDvlYHold(&get_config_future()->pid2[ROV_PID_Y], move_tar.Dvl_y_tar, dvl_y_value, T);
        CalcHeadMoveMotorsSpeed(get_gloal_yaw(), dvl_x_cal, dvl_y_cal, &dvl_x_cal, &dvl_y_cal); // dvl前进为x方向 , dvl陀螺仪与板载反向
        outs_auto[X_OFFSET] += dvl_x_cal * CtrlScale.x_hold_s;
        outs_auto[Y_OFFSET] += dvl_y_cal * CtrlScale.y_hold_s;
    }


  /* 直线保持 */
//    int is_line_hold_changed = IsSwitchChanged(ctrl_msg->rov_control.line_hold, &last_line_hold);
//    if (is_line_hold_changed == 1) { // 获取保持角度
//    	move_tar.line_yaw_tar = line_cal_dvl_value;
//        line_hold_y_ori = dvl_x_value; // 记录初始量计算偏移 以二维直角坐标系计算 xy与机器人及dvl控制相反
//        line_hold_x_ori = dvl_y_value;
//        line_slope = tan((move_tar.line_yaw_tar) * 3.14159f / 180.0f);
//        line_b = line_hold_y_ori - line_slope * line_hold_x_ori;
//        LOG_D("tar_point -> (%f,%f) angle:%f",line_hold_x_ori,line_hold_y_ori,move_tar.line_yaw_tar);
//    }
//    if (ctrl_msg->rov_control.line_hold) {
//      float line_sign = 0;                // 点在线的哪边
//      float point_to_line_distance = 0;   // 点到线距离
//      float included_angle = 0;           // 当前角与目标角夹角 cur - tar
//      float line_yaw_cur = line_cal_dvl_value;
//      included_angle = line_yaw_cur - move_tar.line_yaw_tar; // 夹角计算 做修正运动用
//      line_sign = (line_slope * dvl_y_value + line_b - dvl_x_value) > 0 ? 1 : -1;
//      point_to_line_distance = line_sign * (fabs(line_slope *
//                                      dvl_x_value -
//                                      dvl_y_value +
//                                      line_b) /
//                                      sqrt(1 + powf(line_slope, 2)));
//      float line_fixed = CtrlScale.line_hold_s * CalcLineHold(&get_config_future()->pid2[ROV_PID_LINE], move_tar.line_tar, point_to_line_distance, T);
//
//      float line_hold_fixed_x =  0;
//      float line_hold_fixed_y =  0;
//      CalcHeadMoveMotorsSpeed(included_angle, ctrl_msg->robot_common_motion.x, line_fixed, &line_hold_fixed_x, &line_hold_fixed_y);
//      outs_manual[X_OFFSET] += line_hold_fixed_x;
//      outs_manual[Y_OFFSET] += line_hold_fixed_y;
//  }
    outs_manual[ROLL_X_OFFSET] += ctrl_msg->robot_common_motion.revolve_x;
    outs_manual[PITCH_Y_OFFSET] += ctrl_msg->robot_common_motion.revolve_y;

    float outs_final[6] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0}; // x y z roll_x pitch_y yaw_z
    for(int _i = 0; _i < 6; _i ++){
        outs_final[_i] = outs_manual[_i] + outs_auto[_i];
    }
    ScaleDownDataF(outs_final, 1000, 6);

    motion_speed->x = outs_final[X_OFFSET];
    motion_speed->y = outs_final[Y_OFFSET];
    motion_speed->z = outs_final[Z_OFFSET];
    motion_speed->roll_x = outs_final[ROLL_X_OFFSET];
    motion_speed->pitch_y = outs_final[PITCH_Y_OFFSET];
    motion_speed->yaw_z = outs_final[YAW_Z_OFFSET];

//    LOG_D("x:%f y:%f z:%f roll:%f pitch:%f yaw:%f"
//            , motion_speed->x, motion_speed->y, motion_speed->z
//            , motion_speed->roll_x, motion_speed->pitch_y, motion_speed->yaw_z);
}

// 依据直角坐标系  偏差角度对比直角坐标系的Y轴 角度左正
void CalcHeadMoveXY(float x_ori, float y_ori, float yaw, float x_to, float y_to, float *x1, float *y1) {
    *x1 = x_ori + x_to * cosf(yaw * 3.14159256f / 180.0f) - y_to * sinf(yaw * 3.14159256f / 180.0f);
    *y1 = y_ori + x_to * sinf(yaw * 3.14159256f / 180.0f) + y_to * cosf(yaw * 3.14159256f / 180.0f);
}

static int IsSwitchChanged(uint8_t key, uint8_t* last_key) {
    int ret_value = 0;
    if (*last_key == 0 && key == 1) {
        ret_value = 1;
    } else if (*last_key == 1 && key == 0) {
        ret_value = -1;
    }
    *last_key = key;
    return ret_value;
}

void TargetSlowlyIncrease(float cur, float* tar, float diff){
    float gap = *tar - cur;
    if (fabs(gap) > diff) {
        float adjust = (gap > 0) ? diff : -diff;
        *tar = cur + adjust;
    }
}

void CalcMovePoint(float x, float y, float angle, float distance, float* result_x, float* result_y) {
    // 函数内基于直角坐标系 角度左为正
    float radians = (angle + 90) * (M_PI / 180.0);
    *result_x = x + distance * cos(radians);
    *result_y = y + distance * sin(radians);
}

float CalculateDistance(float x1, float y1, float x2, float y2) {
    float dx = x2 - x1;
    float dy = y2 - y1;
    return sqrt(dx * dx + dy * dy);
}
