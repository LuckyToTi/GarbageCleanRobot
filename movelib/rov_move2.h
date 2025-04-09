#ifndef __ROV_MOVE2_H_
#define __ROV_MOVE2_H_

#include "stm32f4xx.h"
#include "math.h"
#include "userlib/math2.h"
#include "userlib/pid.h"
#include "userlib/imu_data.h"
#include "userlib/math2.h"
#include "controller_switch_thread.h"
#include "motion_algorithm.h"

struct RovMoveTarget {
    float roll_tar;
    float pitch_tar;
    float yaw_hold_tar;

    float line_tar;
    float Dvl_x_tar;
    float Dvl_y_tar;
    float depth_fixed_tar;

    float line_yaw_tar;
    float head_move_x_tar;
    float head_move_y_tar;
    float head_move_yaw;
};

struct PositionData {
    struct
    {
        float x;
        float y;
        float z;
        struct EulerAngle euler_angle;
    }dvl;

    struct {
        float a;
    } gnss_data_rmc;
};

#endif  // __ROV_MOVE2_H_
