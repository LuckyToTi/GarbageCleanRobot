#include <movelib/mecanum_move.h>
#include "math.h"
#include "userlib/math2.h"
#include "userlib/pid2.h"
#include "gyro_thread.h"
#include "controller_switch_def.h"

#define LOG_TAG     "mecanum_move"
//#define LOG_LVL     LOG_LVL_INFO
#include <ulog.h>

#define X_OFFSET (0)
#define Y_OFFSET (1)
#define YAW_Z_OFFSET (2)

//#define roll_x_value (-get_bsp_gyro_data()->euler_angle.roll_x)
//#define pitch_y_value (-get_bsp_gyro_data()->euler_angle.pitch_y)
//#define yaw_z_value (get_bsp_gyro_data()->euler_angle.yaw_z)
#define water_depth_m (Get_Water_Depth() / 1000.0f)
#define dvl_x_value (get_dvl_position_local_data()->x)
#define dvl_y_value (-get_dvl_position_local_data()->y)
#define dvl_yaw_value (get_dvl_yaw())
#define line_dvl_x_value (dvl_x_value)
#define line_dvl_y_value (get_dvl_position_local_data()->y)
#define line_cal_dvl_value (GetAngleOver360(dvl_yaw_value + 90))
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

//static struct Targets move_tar = { 0, 0, 0, 0, 0, 0 };
//struct Targets* get_move_tar() {
//    return &move_tar;
//}

//static int IsSwitchChanged(uint8_t key, uint8_t* last_key);
//static void CalcHeadMoveXY(float x_ori, float y_ori, float yaw, float x_to, float y_to, float *x1, float *y1);
void MoveAutoAndManualForMecanum(struct MotionSpeed* motion_speed, const struct ControlMsg* ctrl_msg, float T) {
    float outs_manual[4] = {0.0, 0.0, 0.0, 0.0 }; // x y yaw_z
    float outs_auto[4] = {0.0, 0.0, 0.0, 0.0 }; // x y yaw_z

    outs_manual[X_OFFSET] += ctrl_msg->robot_common_motion.x;
    outs_manual[Y_OFFSET] += ctrl_msg->robot_common_motion.y;
    outs_manual[YAW_Z_OFFSET] += ctrl_msg->robot_common_motion.revolve_z;

    float outs_final[4] = {0.0, 0.0, 0.0, 0.0 }; // x y roll_x
    for(int _i = 0; _i < 4; _i ++){
        outs_final[_i] = outs_manual[_i] + outs_auto[_i];
    }
    motion_speed->x = outs_final[X_OFFSET];
    motion_speed->y = outs_final[Y_OFFSET];
    motion_speed->yaw_z = outs_final[YAW_Z_OFFSET];
}

/**
  * @brief	朝着头方向做基于当前坐标移动后的坐标 (函数内部基于直角坐标系计算)
  */
//static void CalcHeadMoveXY(float x_ori, float y_ori, float yaw, float x_to, float y_to, float *x1, float *y1) {
//    *x1 = x_ori + x_to * cosf(yaw * 3.14159256f / 180.0f) - y_to * sinf(yaw * 3.14159256f / 180.0f);
//    *y1 = y_ori + x_to * sinf(yaw * 3.14159256f / 180.0f) + y_to * cosf(yaw * 3.14159256f / 180.0f);
//}

//static int IsSwitchChanged(uint8_t key, uint8_t* last_key) {
//    int ret_value = 0;
//    if (*last_key == 0 && key == 1) {
//        ret_value = 1;
//    } else if (*last_key == 1 && key == 0) {
//        ret_value = -1;
//    }
//    *last_key = key;
//    return ret_value;
//}
