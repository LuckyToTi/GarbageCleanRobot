#ifndef __MOTION_ALGORITHM_H__
#define __MOTION_ALGORITHM_H__

#include "stm32f4xx.h"
#include "userlib/pid.h"

#define ROV_PID_ROLL 	(0)
#define ROV_PID_PITCH 	(1)
#define ROV_PID_YAW 	(2)
#define ROV_PID_DEPTH 	(3)
#define ROV_PID_X 		(4)
#define ROV_PID_Y 		(5)
#define ROV_PID_LINE	(6)

struct MotionSpeed {
    float x;
    float y;
    float z;
    float roll_x;
    float pitch_y;
    float yaw_z;
};

struct MotionSpeed* get_motion_speed();
float CalcRollHold(const struct PidFeedforwardPositionParam* pid_param,
        const float tar, const float cur, const float T);
float CalcPitchHold(const struct PidFeedforwardPositionParam* pid_param,
        const float tar, const float cur, const float T);
float CalcYawHold(const struct PidFeedforwardPositionParam* pid_param,
        const float tar, const float cur, const float T);
float CalcDepthHold(const struct PidFeedforwardPositionParam* pid_param,
        const float tar, const float cur, const float T);
float CalcDvlXHold(const struct PidFeedforwardPositionParam* pid_param,
        const float tar, const float cur, const float T);
float CalcDvlYHold(const struct PidFeedforwardPositionParam* pid_param,
        const float tar, const float cur, const float T);
float CalcLineHold(const struct PidFeedforwardPositionParam* pid_param,
        const float tar, const float cur, const float T);
void CalcHeadMoveMotorsSpeed(const float offset_angle,
        const float x_move_speed, const float y_move_speed,
        float* x_out, float* y_out);

#endif  // __MOTION_ALGORITHM_H__
