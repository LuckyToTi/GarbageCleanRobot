#include "pid2.h"
#include "math.h"

static float PidLimit_(float value, const float min, const float max)
{
    if(value < min) return min;
    else if(value > max) return max;
    return value;
}

static void FeedforwardControl_(const struct FeedforwardControlParam* feedforward_control_param,
                                struct FeedforwardControlRun* feedforward_control_run)
{
    if(feedforward_control_param->type == 1) {
        if(feedforward_control_run->input > 0 && feedforward_control_run->input > feedforward_control_param->start_add_input) {
            feedforward_control_run->output = feedforward_control_param->add_value * feedforward_control_param->k;
        } else if(feedforward_control_run->input < 0 && feedforward_control_run->input < -feedforward_control_param->start_add_input) {
            feedforward_control_run->output = -feedforward_control_param->add_value * feedforward_control_param->k;
        } else {
            feedforward_control_run->output = 0.0;
        }
    } else if (feedforward_control_param->type == 2) {
        feedforward_control_run->output = feedforward_control_param->k * feedforward_control_run->input;
        feedforward_control_run->output = PidLimit_(feedforward_control_run->output,
                                                -feedforward_control_param->max_out,
                                                feedforward_control_param->max_out);
    } else {
        feedforward_control_run->output = 0.0;
    }
}

static void CalcPositionPid_(const struct PidPositionParam* pid_param,
                             struct PidPositionRun* pid_run, const float T)
{
    float intergal_spare_k = 1.0f;
    pid_run->err = pid_run->tar - pid_run->cur;

    if(pid_param->is_integral_spare) {  // 变积分(积分分离)
        if(fabs(pid_run->err) > pid_param->begin_integral) {
            intergal_spare_k = 0.0f;
        } else if(fabs(pid_run->err) < pid_param->stop_grow_integral) {
            intergal_spare_k = 1.0f;
            pid_run->err_integral += pid_run->err * T;
        } else {
            intergal_spare_k = (pid_param->begin_integral - fabs(pid_run->err))
                               / (pid_param->begin_integral - pid_param->stop_grow_integral);
            pid_run->err_integral += pid_run->err * T;
        }
    } else {
        pid_run->err_integral += pid_run->err * T;
    }

    pid_run->err_integral = PidLimit_(pid_run->err_integral,
                            -pid_param->max_err_integral, pid_param->max_err_integral);  // 积分累计限幅

    pid_run->p_out = pid_param->kp * pid_run->err;
    pid_run->i_out = pid_param->ki * pid_run->err_integral * intergal_spare_k;
    pid_run->d_out = pid_param->kd * (pid_run->err - pid_run->old_err);

    pid_run->output = pid_run->p_out + pid_run->i_out + pid_run->d_out;
    pid_run->output = PidLimit_(pid_run->output, -pid_param->max_out, pid_param->max_out);  // 输出限幅
    pid_run->old_err = pid_run->err;
}

float PidFeedPositionCalc(const struct PidFeedforwardPositionParam* pid_param,
                          struct PidFeedforwardPositionRun* pid_run,
                          const float tar, const float cur, const float T)
{
    pid_run->feedforward_control_run.input = tar;
    FeedforwardControl_(&pid_param->feedforward_control_param,
                        &pid_run->feedforward_control_run);
    pid_run->pid_position_run.tar = tar;
    pid_run->pid_position_run.cur = cur;
    CalcPositionPid_(&pid_param->pid_position_param, &pid_run->pid_position_run, T);
    float out = PidLimit_(pid_run->feedforward_control_run.output + pid_run->pid_position_run.output,
                          -pid_param->pid_position_param.max_out, pid_param->pid_position_param.max_out);
    return out;
}
