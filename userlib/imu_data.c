#include "imu_data.h"
#include "string.h"
#include "math2.h"
#include "math.h"

#define LOG_TAG     "imu_data"         // 该模块对应的标签。不定义时，默认：NO_TAG
#define LOG_LVL     LOG_LVL_INFO  // 该模块对应的日志输出级别。不定义时，默认：调试级别
#include <ulog.h>                 // 必须在 LOG_TAG 与 LOG_LVL 下面

#define RAD_TO_DEG_COEFFICIENT (57.29577951308232f)
#define sqrt05 (0.70710678118654752440084436210485f)

inline const struct WitGyro get_wit_gyro_data(void){
    struct WitGyro wit_gyro;
    memcpy(&wit_gyro.acceleration, &get_bsp_gyro_data()->acceleration, sizeof(struct Acceleration));
    memcpy(&wit_gyro.angular_velocity, &get_bsp_gyro_data()->angular_velocity, sizeof(struct Angular_Velocity));
    memcpy(&wit_gyro.eulerangle, &get_bsp_gyro_data()->euler_angle, sizeof(struct EulerAngle));
    memcpy(&wit_gyro.quaternions, &get_bsp_gyro_data()->quaternions, sizeof(struct Quaternions));
    return wit_gyro;
}

inline const struct XsensGyro get_xsens_gyro_data(void){
    struct XsensGyro xsens_gyro;
    memcpy(&xsens_gyro.acceleration, &get_mt_data_2()->acceleration, sizeof(struct Acceleration));
    memcpy(&xsens_gyro.angular_velocity, &get_mt_data_2()->angular_velocity, sizeof(struct Angular_Velocity));
    memcpy(&xsens_gyro.eulerangle, &get_mt_data_2()->euler_angle, sizeof(struct EulerAngle));
    return xsens_gyro;
}

// q = q_1 * q_2
void QuaternionMultiply(const struct Quaternions* q_1, const struct Quaternions* q_2, struct Quaternions* q)
{
    q->q[0] = q_1->q[0] * q_2->q[0] - q_1->q[1] * q_2->q[1] - q_1->q[2] * q_2->q[2] - q_1->q[3] * q_2->q[3];
    q->q[1] = q_1->q[0] * q_2->q[1] + q_1->q[1] * q_2->q[0] + q_1->q[2] * q_2->q[3] - q_1->q[3] * q_2->q[2];
    q->q[2] = q_1->q[0] * q_2->q[2] - q_1->q[1] * q_2->q[3] + q_1->q[2] * q_2->q[0] + q_1->q[3] * q_2->q[1];
    q->q[3] = q_1->q[0] * q_2->q[3] + q_1->q[1] * q_2->q[2] - q_1->q[2] * q_2->q[1] + q_1->q[3] * q_2->q[0];
}

// q = q2 * q1^-1
void CalcRelativeQuaternion(const struct Quaternions* q_1, const struct Quaternions* q_2, struct Quaternions* q)
{
    struct Quaternions q_1_ = {.q[0] = q_1->q[0], .q[1] = -q_1->q[1], .q[2] = -q_1->q[2], .q[3] = -q_1->q[3]};
    QuaternionMultiply(q_2, &q_1_, q);
}

// q --> Euler angle
void Quaternion2EulerAngle(const struct Quaternions* q, struct EulerAngle* euler_angle)
{
    euler_angle->roll_x = math2_fast_atan2(2 * q->q[2] * q->q[3] + 2 * q->q[0] * q->q[1], -2 * q->q[1] * q->q[1] - 2 * q->q[2]* q->q[2] + 1) * RAD_TO_DEG_COEFFICIENT;
    euler_angle->pitch_y = arcsin_cordic(-2 * q->q[1] * q->q[3] + 2 * q->q[0]* q->q[2], 16) * RAD_TO_DEG_COEFFICIENT;
    euler_angle->yaw_z = math2_fast_atan2(2 * (q->q[1]*q->q[2] + q->q[0]*q->q[3]), q->q[0]*q->q[0] + q->q[1]*q->q[1] - q->q[2]*q->q[2] - q->q[3]*q->q[3]) * RAD_TO_DEG_COEFFICIENT;
}

// q1, q2 -> q12 -> Euler Angle
void GetEulerAngleFromQuaternion(const struct Quaternions* q_base, const struct Quaternions* q_now, struct EulerAngle* euler_angle)
{
    struct Quaternions q;
    CalcRelativeQuaternion(q_base, q_now, &q);
    Quaternion2EulerAngle(&q, euler_angle);
}

void GetQuaternionAngleFromEuler(const struct EulerAngle* euler_angle, struct Quaternions* q) {
    double cy = cos(euler_angle->yaw_z / RAD_TO_DEG_COEFFICIENT * 0.5);
    double sy = sin(euler_angle->yaw_z / RAD_TO_DEG_COEFFICIENT * 0.5);
    double cp = cos(euler_angle->pitch_y / RAD_TO_DEG_COEFFICIENT * 0.5);
    double sp = sin(euler_angle->pitch_y / RAD_TO_DEG_COEFFICIENT * 0.5);
    double cr = cos(euler_angle->roll_x / RAD_TO_DEG_COEFFICIENT * 0.5);
    double sr = sin(euler_angle->roll_x / RAD_TO_DEG_COEFFICIENT * 0.5);

    q->q[0] = cr * cp * cy + sr * sp * sy;
    q->q[1] = sr * cp * cy - cr * sp * sy;
    q->q[2] = cr * sp * cy + sr * cp * sy;
    q->q[3] = cr * cp * sy - sr * sp * cy;
}

void EulerModeChange(int mode, const struct EulerAngle origin_euler, struct EulerAngle* final_euler){
    struct Quaternions q_base;
    struct EulerAngle euler_;
    euler_.pitch_y = origin_euler.pitch_y;
    euler_.roll_x = origin_euler.roll_x;
    euler_.yaw_z = origin_euler.yaw_z;

    switch(mode){
        case 0:
            final_euler->roll_x = euler_.roll_x;
            final_euler->pitch_y = euler_.pitch_y;
            final_euler->yaw_z = euler_.yaw_z;
            return;
            break;
        case -2:
            q_base.q[0] = -sqrt05;
            q_base.q[1] = sqrt05;
            q_base.q[2] = 0;
            q_base.q[3] = 0;
            break;
        case 2:
            q_base.q[0] = sqrt05;
            q_base.q[1] = sqrt05;
            q_base.q[2] = 0;
            q_base.q[3] = 0;
            break;
        case -1:
        case 1:
            q_base.q[0] = 0;
            q_base.q[1] = 1;
            q_base.q[2] = 0;
            q_base.q[3] = 0;
            break;
        case 999: // 测试
            q_base.q[0] = 1;
            q_base.q[1] = 0;
            q_base.q[2] = 0;
            q_base.q[3] = 0;
            break;
        default:break;
    }

    struct Quaternions q_now;
//    memcpy(q_now.q, get_bsp_gyro_data()->quaternions.q, sizeof(q_now.q));
    GetQuaternionAngleFromEuler(&euler_,&q_now);
    GetEulerAngleFromQuaternion(&q_base, &q_now, final_euler);

    // 翻转前后欧拉角对比
//    LOG_D("---- %6.2f %6.2f %6.2f ---- %6.2f %6.2f %6.2f",euler_.roll_x, euler_.pitch_y, euler_.yaw_z, final_euler->roll_x, final_euler->pitch_y, final_euler->yaw_z);
//    LOG_D("w:%f x:%f y:%f z:%f",q_now.q[0], q_now.q[1], q_now.q[2], q_now.q[3]);
}


