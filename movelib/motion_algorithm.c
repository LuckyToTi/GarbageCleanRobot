#include "motion_algorithm.h"
#include "math.h"
#include "userlib/math2.h"
#include "userlib/pid2.h"
#include "controller_switch_def.h"
#include "rov_move.h"

#define LOG_TAG     "m_a"
//#define LOG_LVL     LOG_LVL_INFO
#include <ulog.h>

#define OUT_MAX_VALUE (1000.0)

struct MotionSpeed motion_speed;
inline struct MotionSpeed* get_motion_speed() {
    return &motion_speed;
}

static void PidLogOrUdplog(const struct PidFeedforwardPositionRun* pid_run, uint16_t id){
    if(get_temp()->log_level[id] > 0){
        LOG_I("tar cur: %.2f ,%.2f, %.2f ,%.2f",pid_run->pid_position_run.tar ,pid_run->pid_position_run.cur
                ,pid_run->pid_position_run.err_integral ,pid_run->pid_position_run.output);
    }
    if(get_temp()->log_level[id] > 1){
        print_rl_udp(1,"tar cur: %.2f ,%.2f",pid_run->pid_position_run.tar ,pid_run->pid_position_run.cur);
    }
}

// return: +roll
float CalcRollHold(const struct PidFeedforwardPositionParam* pid_param,
        const float tar, const float cur, const float T)
{
    static struct PidFeedforwardPositionRun pid_run = NEW_FEED_POSITION_PID2_RUN();
    PidLogOrUdplog(&pid_run, ROV_PID_ROLL);
    return PidFeedPositionCalc(pid_param, &pid_run, tar, cur, T);
}

// return: +pitch
float CalcPitchHold(const struct PidFeedforwardPositionParam* pid_param,
        const float tar, const float cur, const float T)
{
    static struct PidFeedforwardPositionRun pid_run = NEW_FEED_POSITION_PID2_RUN();
    PidLogOrUdplog(&pid_run, ROV_PID_PITCH);
    return PidFeedPositionCalc(pid_param, &pid_run, tar, cur, T);
}

// return: +yaw
float CalcYawHold(const struct PidFeedforwardPositionParam* pid_param,
        const float tar, const float cur, const float T)
{
    static struct PidFeedforwardPositionRun pid_run = NEW_FEED_POSITION_PID2_RUN();
    float yaw_tar_angle = tar;
    float yaw_cur_angle = cur;
    Handle_Angle360_PID_Over_Zero(&yaw_tar_angle, &yaw_cur_angle);
    PidLogOrUdplog(&pid_run, ROV_PID_YAW);
    return PidFeedPositionCalc(pid_param, &pid_run, yaw_tar_angle, yaw_cur_angle, T);
}

// return: +depth
float CalcDepthHold(const struct PidFeedforwardPositionParam* pid_param,
        const float tar, const float cur, const float T) // tar目标值  cur当前值
{
    static struct PidFeedforwardPositionRun pid_run = NEW_FEED_POSITION_PID2_RUN();
    PidLogOrUdplog(&pid_run, ROV_PID_DEPTH);
    return PidFeedPositionCalc(pid_param, &pid_run, tar, cur, T);
}

// return: +x
float CalcDvlXHold(const struct PidFeedforwardPositionParam* pid_param,
        const float tar, const float cur, const float T)
{
    static struct PidFeedforwardPositionRun pid_run = NEW_FEED_POSITION_PID2_RUN();
    PidLogOrUdplog(&pid_run, ROV_PID_X);
    return PidFeedPositionCalc(pid_param, &pid_run, tar, cur, T);
}

// return: +y
float CalcDvlYHold(const struct PidFeedforwardPositionParam* pid_param,
        const float tar, const float cur, const float T)
{
    static struct PidFeedforwardPositionRun pid_run = NEW_FEED_POSITION_PID2_RUN();
    PidLogOrUdplog(&pid_run, ROV_PID_Y);
    return PidFeedPositionCalc(pid_param, &pid_run, tar, cur, T);
}

// return: +line
float CalcLineHold(const struct PidFeedforwardPositionParam* pid_param,
        const float tar, const float cur, const float T)
{
    static struct PidFeedforwardPositionRun pid_run = NEW_FEED_POSITION_PID2_RUN();
    PidLogOrUdplog(&pid_run, ROV_PID_LINE);
    return PidFeedPositionCalc(pid_param, &pid_run, tar, cur, T);
}



/* 小陀螺运动 */
/**
  * @brief          朝着头动
  *
  * 效果：底盘以设置速度小陀螺旋转或处于特殊状态，同时可以以云台枪管为头，前进后退、左右平移
  *
  * @author         Bashpow
  * @param[in]      相对头的偏移角度[运动的方向角度0°~360°，以机器朝向为0°(云台GM6020电机相对底盘的角度)]
  * @param[in]      x轴速度输入（X摇杆）
  * @param[in]      y轴速度输入（Y摇杆）
  * @param[in]      x轴速度输出
  * @param[in]      y轴速度输出
  * @retval         返回空
  */
void CalcHeadMoveMotorsSpeed(const float offset_angle,
        const float x_move_speed, const float y_move_speed,
        float* x_out, float* y_out)
{
    if(offset_angle > 360 || offset_angle < -360) {  // 判断角度是否错误
        return;
    }

    //角度换算弧度
    float move_radin = offset_angle * 3.14159256f / 180.0f;
    float radin_sin = sinf(move_radin);
    float radin_cos = cosf(move_radin);

    //计算速度增加值
    float x_x_speed =  x_move_speed*radin_cos;
    float x_y_speed = -x_move_speed*radin_sin;

    float y_x_speed =  y_move_speed*radin_sin;
    float y_y_speed =  y_move_speed*radin_cos;

    *x_out = x_x_speed + y_x_speed;
    *y_out = x_y_speed + y_y_speed;
}



