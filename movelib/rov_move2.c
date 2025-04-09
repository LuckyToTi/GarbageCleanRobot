#include "rov_move2.h"
#include "dvl_json_tcp_thread.h"

#define X_OFFSET (0)
#define Y_OFFSET (1)
#define Z_OFFSET (2)
#define ROLL_X_OFFSET (3)
#define PITCH_Y_OFFSET (4)
#define YAW_Z_OFFSET (5)

static const struct {
    float roll_hold_s;
    float pitch_hold_s;
    float yaw_hold_s;
    float depth_hold_s;
    float x_hold_s;
    float y_hold_s;
    float line_hold_s;
} CtrlScale = { 1.0, 1.0, 0.8, 1.0, 1.0, 1.0, 0.8};

#define dvl_yaw_value (get_dvl_yaw())
#define line_cal_dvl_value (GetAngleOver360(dvl_yaw_value + 90))

static int IsSwitchChanged(int key, int* last_key) {
    int ret_value = 0;
    if (*last_key == 0 && key == 1) {
        ret_value = 1;
    } else if (*last_key == 1 && key == 0) {
        ret_value = -1;
    }
    *last_key = key;
    return ret_value;
}

static void CalcHeadMoveXY(float x_ori, float y_ori, float yaw, float x_to, float y_to, float *x1, float *y1) {
    *x1 = x_ori + x_to * cosf(yaw * 3.14159256f / 180.0f) - y_to * sinf(yaw * 3.14159256f / 180.0f);
    *y1 = y_ori + x_to * sinf(yaw * 3.14159256f / 180.0f) + y_to * cosf(yaw * 3.14159256f / 180.0f);
}

void RovMoveManual(float* outs_manual, const struct ControlMsg* ctrl_msg){
    if(!ctrl_msg->rov_control.depth_hold){ // 定深开启后 控制器的Z由深度目标值接管
        outs_manual[Z_OFFSET] += ctrl_msg->robot_common_motion.z;
    }

    if(!ctrl_msg->rov_control.yaw_hold){  // 定航开启后 控制器的YAW由航向目标角接管
        outs_manual[YAW_Z_OFFSET] += ctrl_msg->robot_common_motion.revolve_z;
    }

    if(!ctrl_msg->rov_control.yaw_hold){  // 定航开启后 控制器的X,Y由航向修正运动接管
        outs_manual[X_OFFSET] += ctrl_msg->robot_common_motion.x;
        outs_manual[Y_OFFSET] += ctrl_msg->robot_common_motion.y;
    }
    // todo:直线航行开启后置Y为0
    // todo:move_second
}

void CalcRollHold_Rewrite_(float* out_auto_roll_hold, float roll_hold_scale, const float tar, const float cur, const float T){
    static struct PidFeedforwardPositionRun pid_run = NEW_FEED_POSITION_PID2_RUN();
    *out_auto_roll_hold += roll_hold_scale * PidFeedPositionCalc(&get_config_future()->pid2[ROV_PID_ROLL], &pid_run, tar, cur, T);
}

void CalcPitchHold_Rewrite_(float* out_auto_roll_hold, float roll_hold_scale, const float tar, const float cur, const float T){
    static struct PidFeedforwardPositionRun pid_run = NEW_FEED_POSITION_PID2_RUN();
    *out_auto_roll_hold += roll_hold_scale * PidFeedPositionCalc(&get_config_future()->pid2[ROV_PID_PITCH], &pid_run, tar, cur, T);
}

void CalcYawHold_Rewrite_(float* out_auto_roll_hold, float roll_hold_scale, const float tar, const float cur, const float T){
    static struct PidFeedforwardPositionRun pid_run = NEW_FEED_POSITION_PID2_RUN();
    float yaw_tar_angle = tar;
    float yaw_cur_angle = cur;
    Handle_Angle360_PID_Over_Zero(&yaw_tar_angle, &yaw_cur_angle);
    *out_auto_roll_hold += roll_hold_scale * PidFeedPositionCalc(&get_config_future()->pid2[ROV_PID_YAW], &pid_run, yaw_tar_angle, yaw_cur_angle, T);
}

void CalcDvlX_Rewrite_(float* out_auto_dvl_x_hold, float roll_hold_scale, const float tar, const float cur, const float T){
    static struct PidFeedforwardPositionRun pid_run = NEW_FEED_POSITION_PID2_RUN();
    *out_auto_dvl_x_hold += roll_hold_scale * PidFeedPositionCalc(&get_config_future()->pid2[ROV_PID_X], &pid_run, tar, cur, T);
}

void CalcDvlY_Rewrite_(float* out_auto_dvl_y_hold, float roll_hold_scale, const float tar, const float cur, const float T){
    static struct PidFeedforwardPositionRun pid_run = NEW_FEED_POSITION_PID2_RUN();
    *out_auto_dvl_y_hold += roll_hold_scale * PidFeedPositionCalc(&get_config_future()->pid2[ROV_PID_Y], &pid_run, tar, cur, T);
}

void RovMoveAuto(float* out_auto, const struct ControlMsg* ctrl_msg,
        struct ControlMsg* ctrl_msg_last, struct RovMoveTarget move_tar,
        struct EulerAngle euler_angle, struct Quaternions quaternions,
        struct PositionData pos_data, float water_depth_m, float height_m, float T){

    static uint8_t head_moveing_flag = 0; // 设定机器人朝头移动的标志位,定义在这是因为ctrl_msg为const无法修改

    // TargetChangeFirst-------------------------------------------------------
    /* 航向保持初始值 */
    int is_yaw_hold_changed = IsSwitchChanged(ctrl_msg->rov_control.yaw_hold, &ctrl_msg_last->rov_control.yaw_hold);
    if(is_yaw_hold_changed == 1) {
        move_tar.yaw_hold_tar = GetAngle360FromAngle180(euler_angle.yaw_z);
    } else if (is_yaw_hold_changed == -1) {
        move_tar.yaw_hold_tar = 0.0;
    }

    /* 深度保持初始值 */
    int is_depth_hold_changed = IsSwitchChanged(ctrl_msg->rov_control.depth_hold, &ctrl_msg_last->rov_control.depth_hold);
    if(is_depth_hold_changed == 1) {  // 开启定深（0->1）
        move_tar.depth_fixed_tar = water_depth_m;  // 记录当前深度为目标深度
    } else if (is_depth_hold_changed == -1) {  // 关闭定深（1->0）
        move_tar.depth_fixed_tar = 0.0;  // 记录当前深度为目标深度
    }

    /* 动力定位开启时(非重置点定位) */
    int is_position_hold_changed = IsSwitchChanged(ctrl_msg->rov_control.position_hold, &ctrl_msg_last->rov_control.position_hold);
    if (is_position_hold_changed == 1) { // 获取保持角度
        move_tar.Dvl_x_tar = pos_data.dvl.x;
        move_tar.Dvl_y_tar = pos_data.dvl.y;
        print_rl_udp(2, "dvl x y change to: %.2f, %.2f", move_tar.Dvl_x_tar,move_tar.Dvl_y_tar);
    }
    // ------------------------------------------------------------------------

    // TargetChange============================================================
    /* yaw航向保持 */
    if(ctrl_msg->rov_control.yaw_hold) {
        move_tar.yaw_hold_tar += ctrl_msg->robot_common_motion.revolve_z * T / 10.0;
        move_tar.yaw_hold_tar = GetAngle360FromAngle180(move_tar.yaw_hold_tar);
        if(ctrl_msg->robot_common_motion.revolve_z != 0)
            print_rl_udp(2, "yaw tar change to: %.2f", move_tar.yaw_hold_tar);
    }

    /* 深度保持 */
    if(ctrl_msg->rov_control.depth_hold) {
        move_tar.depth_fixed_tar -= ctrl_msg->robot_common_motion.z * T / 300.0;
        if (move_tar.depth_fixed_tar < 0) move_tar.depth_fixed_tar = 0;
        if(ctrl_msg->robot_common_motion.z != 0)
            print_rl_udp(2,"depth tar change to: %.2f",move_tar.depth_fixed_tar);
    }
    // ========================================================================

    // -------------------------AttitudeHolePIDCalc---------------------------
    // 辅助航行PID直接接管部分
    static struct PidFeedforwardPositionRun pid_run_roll = NEW_FEED_POSITION_PID2_RUN();
    static struct PidFeedforwardPositionRun pid_run_pitch = NEW_FEED_POSITION_PID2_RUN();
    static struct PidFeedforwardPositionRun pid_run_yaw = NEW_FEED_POSITION_PID2_RUN();
    static struct PidFeedforwardPositionRun pid_run_dvl_x = NEW_FEED_POSITION_PID2_RUN();
    static struct PidFeedforwardPositionRun pid_run_dvl_y = NEW_FEED_POSITION_PID2_RUN();
//    static struct PidFeedforwardPositionRun pid_run[5] = NEW_FEED_POSITION_PID2_RUN();
//    for(int i_ = 0; i_ < 5; i_++){
//        pid_run[i_] = NEW_FEED_POSITION_PID2_RUN();
//    }


    out_auto[ROLL_X_OFFSET] += CtrlScale.roll_hold_s * PidFeedPositionCalc(&get_config_future()->pid2[ROV_PID_ROLL], &pid_run_roll, move_tar.roll_tar, euler_angle.roll_x, T);
    out_auto[PITCH_Y_OFFSET] += CtrlScale.pitch_hold_s * PidFeedPositionCalc(&get_config_future()->pid2[ROV_PID_PITCH], &pid_run_pitch, move_tar.pitch_tar, euler_angle.pitch_y, T);
    out_auto[YAW_Z_OFFSET] += CtrlScale.yaw_hold_s * PidFeedPositionCalc(&get_config_future()->pid2[ROV_PID_YAW], &pid_run_yaw, move_tar.pitch_tar, euler_angle.yaw_z, T);

    CalcRollHold_Rewrite_(&out_auto[ROLL_X_OFFSET], CtrlScale.roll_hold_s, move_tar.roll_tar, euler_angle.roll_x, T);
    CalcPitchHold_Rewrite_(&out_auto[PITCH_Y_OFFSET], CtrlScale.pitch_hold_s, move_tar.pitch_tar, euler_angle.pitch_y, T);
    CalcYawHold_Rewrite_(&out_auto[YAW_Z_OFFSET], CtrlScale.yaw_hold_s, move_tar.pitch_tar, euler_angle.yaw_z, T);
    // ------------------------------------------------------------------------

    // ===========================姿态保持外的辅助=============================
    // todo: 补充控制有无额外辅助航行的变量
    /* 航向保持状态下的修正运动 */
    if(ctrl_msg->rov_control.yaw_hold){
        float included_angle; // 航向角与当前角夹角
        float yaw_hold_fixed_x = 0, yaw_hold_fixed_y = 0;
        included_angle = GetAngle360FromAngle180(euler_angle.yaw_z) - move_tar.yaw_hold_tar;
        // 修正运动接管平面XY运动
        if(!ctrl_msg->rov_control.line_hold)
            CalcHeadMoveMotorsSpeed(included_angle, ctrl_msg->robot_common_motion.x, ctrl_msg->robot_common_motion.y, &yaw_hold_fixed_x, &yaw_hold_fixed_y);
        out_auto[X_OFFSET] += yaw_hold_fixed_x;
        out_auto[Y_OFFSET] += yaw_hold_fixed_y;
    }

    /* 动力定位 */
    if (ctrl_msg->rov_control.position_hold && ctrl_msg->rov_control.yaw_hold
            && (ctrl_msg->robot_common_motion.x == 0 && ctrl_msg->robot_common_motion.y == 0)) { // 手动接管则停止动力定位动作
        if(head_moveing_flag == 1 && move_tar.Dvl_x_tar - pos_data.dvl.x < 0.2 && move_tar.Dvl_y_tar - pos_data.dvl.y < 0.2){
            head_moveing_flag = 0;
            print_rl_udp(2, "dvl x y head move done");
        }
        float dvl_x_cal = 0.0f;
        dvl_x_cal += CtrlScale.x_hold_s * PidFeedPositionCalc(&get_config_future()->pid2[ROV_PID_X], &pid_run_dvl_x, move_tar.Dvl_x_tar, pos_data.dvl.x, T);
//        CalcPitchHold_Rewrite_(&dvl_x_cal, CtrlScale.x_hold_s, move_tar.Dvl_x_tar, pos_data.dvl.x, T);
        float dvl_y_cal = 0.0f;
        dvl_y_cal += CtrlScale.y_hold_s * PidFeedPositionCalc(&get_config_future()->pid2[ROV_PID_Y], &pid_run_dvl_y, move_tar.Dvl_y_tar, pos_data.dvl.y, T);
//        CalcPitchHold_Rewrite_(&dvl_y_cal, CtrlScale.y_hold_s, move_tar.Dvl_y_tar, pos_data.dvl.y, T);
        CalcHeadMoveMotorsSpeed(dvl_yaw_value, dvl_x_cal, dvl_y_cal, &dvl_x_cal, &dvl_y_cal); // dvl前进为x方向 , dvl陀螺仪与板载反向
        out_auto[X_OFFSET] += dvl_x_cal * CtrlScale.x_hold_s;
        out_auto[Y_OFFSET] += dvl_y_cal * CtrlScale.y_hold_s;
    }

    /* 定向(参考当前头)定量位移 */
    if (move_tar.head_move_x_tar != 0 || move_tar.head_move_y_tar != 0) {
        if(ctrl_msg->rov_control.position_hold != 1){
            print_rl_udp(2, "position_hold unopened");
        }else{
            head_moveing_flag = 1;
            print_rl_udp(2, "head_move start");
            float head_move_x_to = 0;
            float head_move_y_to = 0;
            CalcHeadMoveXY(move_tar.Dvl_y_tar, move_tar.Dvl_x_tar, line_cal_dvl_value + move_tar.head_move_yaw, move_tar.head_move_x_tar, move_tar.head_move_y_tar, &head_move_y_to, &head_move_x_to);
            move_tar.Dvl_x_tar = head_move_x_to;
            move_tar.Dvl_y_tar = head_move_y_to;
            print_rl_udp(2, "dvl x y change to: %.2f, %.2f", move_tar.Dvl_x_tar,move_tar.Dvl_y_tar);
            move_tar.head_move_y_tar = 0;
            move_tar.head_move_x_tar = 0;
        }
    }
    // todo: 直线保持
    // ========================================================================
}

void RovMoveAutoAndManual2(struct MotionSpeed* motion_speed, int first_enter,
                           const struct ControlMsg* ctrl_msg, struct RovMoveTarget move_tar,
                           struct EulerAngle euler_angle, struct Quaternions quaternions,
                           struct PositionData pos_data,float water_depth_m, float height_m,
                           float T){
    static struct ControlMsg* ctrl_msg_last = NULL;
    if(ctrl_msg_last == NULL) {
        ctrl_msg_last = rt_malloc(sizeof(struct ControlMsg));
        reset_control_msg(ctrl_msg_last);
    }

    float outs_manual[6] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0}; // x y z roll_x pitch_y yaw_z
    float outs_auto[6] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0}; // x y z roll_x pitch_y yaw_z
    RovMoveManual(outs_manual,ctrl_msg);
    RovMoveAuto(outs_auto, ctrl_msg, ctrl_msg_last, move_tar, euler_angle, quaternions, pos_data, water_depth_m, height_m, T);

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

    // save last time MSG
    rt_memcpy(ctrl_msg_last, ctrl_msg, sizeof(struct ControlMsg));
}




//void RovMoveAutoAndManual2(struct MotionSpeed* motion_speed, int first_enter,
//                           const struct ControlMsg* ctrl_msg, struct RovMoveTarget move_tar,
//                           struct EulerAngle euler_angle, struct Quaternions quaternions,
//                           struct PositionData pos_data,float water_depth_m, float height_m,
//                           float T)
//{//    static uint8_t head_moveing_flag = 0;
//
//    static float line_hold_x_ori = 0, line_hold_y_ori = 0;
//    static float line_slope = 0.0f;
//    static float line_b = 0;
//
//    // INIT used data
//    static struct ControlMsg* ctrl_msg_last = NULL;
//    if(ctrl_msg_last == NULL) {
//        ctrl_msg_last = rt_malloc(sizeof(struct ControlMsg));
//        reset_control_msg(ctrl_msg_last);
//    }
//
//    float outs_manual[6] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0}; // x y z roll_x pitch_y yaw_z
//    float outs_auto[6] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0}; // x y z roll_x pitch_y yaw_z
//
//
//    /* roll(横滚)保持 */
//    if (ctrl_msg->rov_control.roll_hold) {
//        outs_auto[ROLL_X_OFFSET] += CtrlScale.roll_hold_s * CalcRollHold(&get_config_future()->pid2[ROV_PID_ROLL], move_tar.roll_tar, euler_angle.roll_x, T);
//    }
//
//    /* pitch(俯仰)保持 */
//    if(ctrl_msg->rov_control.pitch_hold) {
//        outs_auto[PITCH_Y_OFFSET] += CtrlScale.pitch_hold_s * CalcPitchHold(&get_config_future()->pid2[ROV_PID_PITCH], move_tar.pitch_tar, euler_angle.pitch_y, T);
//    }
//
//    /* 深度保持初始值 */
//    int is_depth_hold_changed = IsSwitchChanged(ctrl_msg->rov_control.depth_hold, &ctrl_msg_last->rov_control.depth_hold);
//    if(is_depth_hold_changed == 1) {  // 开启定深（0->1）
//        move_tar.depth_fixed_tar = water_depth_m;  // 记录当前深度为目标深度
//    } else if (is_depth_hold_changed == -1) {  // 关闭定深（1->0）
//        move_tar.depth_fixed_tar = 0.0;  // 记录当前深度为目标深度
//    }
//
//    /* 深度保持 */
//    if(ctrl_msg->rov_control.depth_hold) {  ///< 深度传感器有效且开启定深
//        move_tar.depth_fixed_tar -= ctrl_msg->robot_common_motion.z * T / 300.0;
//        if (move_tar.depth_fixed_tar < 0) move_tar.depth_fixed_tar = 0;
//        outs_auto[Z_OFFSET] -= CtrlScale.depth_hold_s
//                * CalcDepthHold(&get_config_future()->pid2[ROV_PID_DEPTH], move_tar.depth_fixed_tar, water_depth_m, T);
//        if(ctrl_msg->robot_common_motion.z != 0)
//            print_rl_udp(2,"depth tar change to: %.2f",move_tar.depth_fixed_tar);
//    } else {
//        outs_manual[Z_OFFSET] += ctrl_msg->robot_common_motion.z;
//    }
//
//    /* 航向保持初始值 */
//    int is_yaw_hold_changed = IsSwitchChanged(ctrl_msg->rov_control.yaw_hold, &ctrl_msg_last->rov_control.yaw_hold);
//    if(is_yaw_hold_changed == 1) {  // 开启yaw保持（0->1）
//        move_tar.yaw_hold_tar = GetAngle360FromAngle180(euler_angle.yaw_z);  // 记录当前深度为目标深度
//    } else if (is_yaw_hold_changed == -1) {
//        move_tar.yaw_hold_tar = 0.0;
//    }
//
//    /* yaw航向保持 */
//    if(ctrl_msg->rov_control.yaw_hold) {
//        move_tar.yaw_hold_tar += ctrl_msg->robot_common_motion.revolve_z * T / 10.0;
//        move_tar.yaw_hold_tar = GetAngle360FromAngle180(move_tar.yaw_hold_tar);  // if yaw_hold_tar< 0 then yaw_hold_tar+=360
//        move_tar.yaw_hold_tar = (move_tar.yaw_hold_tar > 360) ? (move_tar.yaw_hold_tar - 360) : move_tar.yaw_hold_tar;
//        outs_auto[YAW_Z_OFFSET] += CtrlScale.yaw_hold_s * CalcYawHold(&get_config_future()->pid2[ROV_PID_YAW],
//                move_tar.yaw_hold_tar, GetAngle360FromAngle180(euler_angle.yaw_z), T);
//        if(ctrl_msg->robot_common_motion.revolve_z != 0)
//            print_rl_udp(2, "yaw tar change to: %.2f", move_tar.yaw_hold_tar);
//    } else {
//        outs_manual[YAW_Z_OFFSET] += ctrl_msg->robot_common_motion.revolve_z;
//    }
//
//    /* 航向保持状态下的修正运动 */
//    if(ctrl_msg->rov_control.yaw_hold){
//        float included_angle; // 航向角与当前角夹角
//        float yaw_hold_fixed_x = 0, yaw_hold_fixed_y = 0;
//        included_angle = GetAngle360FromAngle180(euler_angle.yaw_z) - move_tar.yaw_hold_tar;
//        if(!ctrl_msg->rov_control.line_hold)
//            CalcHeadMoveMotorsSpeed(included_angle, ctrl_msg->robot_common_motion.x, ctrl_msg->robot_common_motion.y, &yaw_hold_fixed_x, &yaw_hold_fixed_y);
//        outs_manual[X_OFFSET] += yaw_hold_fixed_x;
//        outs_manual[Y_OFFSET] += yaw_hold_fixed_y;
//    } else {
//        outs_manual[X_OFFSET] += ctrl_msg->robot_common_motion.x;
//        outs_manual[Y_OFFSET] += ctrl_msg->robot_common_motion.y;
//    }
//
//    /* 定向(参考当前头)定量位移 */
//    if (move_tar.head_move_x_tar != 0 || move_tar.head_move_y_tar != 0) {
//        if(ctrl_msg->rov_control.position_hold != 1){
//            print_rl_udp(2, "position_hold unopened");
//        }else{
//            head_moveing_flag = 1;
//            print_rl_udp(2, "head_move start");
//            float head_move_x_to = 0;
//            float head_move_y_to = 0;
//            CalcHeadMoveXY(move_tar.Dvl_y_tar, move_tar.Dvl_x_tar, line_cal_dvl_value + move_tar.head_move_yaw, move_tar.head_move_x_tar, move_tar.head_move_y_tar, &head_move_y_to, &head_move_x_to);
//            move_tar.Dvl_x_tar = head_move_x_to;
//            move_tar.Dvl_y_tar = head_move_y_to;
//            print_rl_udp(2, "dvl x y change to: %.2f, %.2f", move_tar.Dvl_x_tar,move_tar.Dvl_y_tar);
//            move_tar.head_move_y_tar = 0;
//            move_tar.head_move_x_tar = 0;
//        }
//    }
//
//    /* 动力定位 */
//    int is_position_hold_changed = IsSwitchChanged(ctrl_msg->rov_control.position_hold, &ctrl_msg_last->rov_control.position_hold);
//    if (is_position_hold_changed == 1) { // 获取保持角度
//        move_tar.Dvl_x_tar = pos_data.dvl.x;
//        move_tar.Dvl_y_tar = pos_data.dvl.y;
//        print_rl_udp(2, "dvl x y change to: %.2f, %.2f", move_tar.Dvl_x_tar,move_tar.Dvl_y_tar);
//    }
//    if (ctrl_msg->rov_control.position_hold && ctrl_msg->rov_control.yaw_hold && (ctrl_msg->robot_common_motion.x == 0 && ctrl_msg->robot_common_motion.y == 0)) {
//        if(head_moveing_flag == 1 && move_tar.Dvl_x_tar - pos_data.dvl.x < 0.2 && move_tar.Dvl_y_tar - pos_data.dvl.y < 0.2){
//            head_moveing_flag = 0;
//            print_rl_udp(2, "dvl x y head move done");
//        }
//        float dvl_x_cal = CalcDvlXHold(&get_config_future()->pid2[ROV_PID_X], move_tar.Dvl_x_tar, pos_data.dvl.x, T);
//        float dvl_y_cal = CalcDvlYHold(&get_config_future()->pid2[ROV_PID_Y], move_tar.Dvl_y_tar, pos_data.dvl.y, T);
//        CalcHeadMoveMotorsSpeed(pos_data.dvl.euler_angle.yaw_z, dvl_x_cal, dvl_y_cal, &dvl_x_cal, &dvl_y_cal); // dvl前进为x方向 , dvl陀螺仪与板载反向
//        outs_auto[X_OFFSET] += dvl_x_cal * CtrlScale.x_hold_s;
//        outs_auto[Y_OFFSET] += dvl_y_cal * CtrlScale.y_hold_s;
//    }
//
//
//  /* 直线保持 */
//    int is_line_hold_changed = IsSwitchChanged(ctrl_msg->rov_control.line_hold, &ctrl_msg_last->rov_control.line_hold);
//    if (is_line_hold_changed == 1) { // 获取保持角度
//        move_tar.line_yaw_tar = line_cal_dvl_value;
//        line_hold_y_ori = pos_data.dvl.x; // 记录初始量计算偏移 以二维直角坐标系计算 xy与机器人及dvl控制相反
//        line_hold_x_ori = pos_data.dvl.y;
//        line_slope = tan((move_tar.line_yaw_tar) * 3.14159f / 180.0f);
//        line_b = line_hold_y_ori - line_slope * line_hold_x_ori;
////        LOG_D("tar_point -> (%f,%f) angle:%f",line_hold_x_ori,line_hold_y_ori,move_tar.line_yaw_tar);
//    }
//    if (ctrl_msg->rov_control.line_hold) {
//      float line_sign = 0;                // 点在线的哪边
//      float point_to_line_distance = 0;   // 点到线距离
//      float included_angle = 0;           // 当前角与目标角夹角 cur - tar
//      float line_yaw_cur = line_cal_dvl_value;
//      included_angle = line_yaw_cur - move_tar.line_yaw_tar; // 夹角计算 做修正运动用
//      line_sign = (line_slope * pos_data.dvl.y + line_b - pos_data.dvl.x) > 0 ? 1 : -1;
//      point_to_line_distance = line_sign * (fabs(line_slope *
//                                      pos_data.dvl.x -
//                                      pos_data.dvl.y +
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
//    outs_manual[ROLL_X_OFFSET] += ctrl_msg->robot_common_motion.revolve_x;
//    outs_manual[PITCH_Y_OFFSET] += ctrl_msg->robot_common_motion.revolve_y;
//
//    float outs_final[6] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0}; // x y z roll_x pitch_y yaw_z
//    for(int _i = 0; _i < 6; _i ++){
//        outs_final[_i] = outs_manual[_i] + outs_auto[_i];
//    }
//    ScaleDownDataF(outs_final, 1000, 6);
//
//    motion_speed->x = outs_final[X_OFFSET];
//    motion_speed->y = outs_final[Y_OFFSET];
//    motion_speed->z = outs_final[Z_OFFSET];
//    motion_speed->roll_x = outs_final[ROLL_X_OFFSET];
//    motion_speed->pitch_y = outs_final[PITCH_Y_OFFSET];
//    motion_speed->yaw_z = outs_final[YAW_Z_OFFSET];
//
//    // save last time MSG
//    rt_memcpy(ctrl_msg_last, ctrl_msg, sizeof(struct ControlMsg));
//}

