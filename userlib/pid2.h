#ifndef __PID2_H_
#define __PID2_H_

#include "stm32f4xx.h"

#define NEW_FEED_POSITION_PID2_RUN() {\
    .feedforward_control_run.input           = 0.0,\
    .feedforward_control_run.output          = 0.0,\
    .pid_position_run.tar                    = 0.0,\
    .pid_position_run.cur                    = 0.0,\
    .pid_position_run.err                    = 0.0,\
    .pid_position_run.old_err                = 0.0,\
    .pid_position_run.err_integral           = 0.0,\
    .pid_position_run.p_out                  = 0.0,\
    .pid_position_run.i_out                  = 0.0,\
    .pid_position_run.d_out                  = 0.0,\
    .pid_position_run.output                 = 0.0,\
}

struct FeedforwardControlParam {
    uint16_t type;  // 0:无前馈 1:定值 2:K*input
    float add_value;
    float start_add_input;
    float k;
    float max_out;
};

struct FeedforwardControlRun {
    float input;
    float output;
};

struct PidPositionParam {
    float kp;
    float ki;
    float kd;

    uint16_t is_integral_spare;     // 选择是否使用变积分分离
    float   begin_integral;         // 当误差(err)大于begin_integral时，不积分
    float   stop_grow_integral;     // 当误差(err)小于stop_grow_integral时，积分系数处于最大值

    float max_err_integral;  // 误差累计最大值
    float max_out;           // 最大输出值
};

struct PidPositionRun {

    float tar;
    float cur;

    float err;          // 误差值
    float old_err;      // 上次误差值
    float err_integral; // 积分值

    float p_out;
    float i_out;
    float d_out;
    float output;  // 最终输出值
};

struct PidFeedforwardPositionParam {
    struct FeedforwardControlParam feedforward_control_param;
    struct PidPositionParam pid_position_param;
};

struct PidFeedforwardPositionRun {
    struct FeedforwardControlRun feedforward_control_run;
    struct PidPositionRun pid_position_run;
};

float PidFeedPositionCalc(const struct PidFeedforwardPositionParam* pid_param,
                          struct PidFeedforwardPositionRun* pid_run,
                          const float tar, const float cur, const float T);

#endif
