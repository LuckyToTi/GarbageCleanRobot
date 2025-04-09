#ifndef __IMU_DATA_H_
#define __IMU_DATA_H_

#include "stm32f4xx.h"
#include "gyro_thread.h"
#include "xsens_ahrs_thread.h"

// 加速度 m/s^2
struct Acceleration {
    float x;
    float y;
    float z;
};

// 角速度 rad/s
struct Angular_Velocity {
    float x;
    float y;
    float z;
};

// 欧拉角
struct EulerAngle {
    float roll_x;
    float pitch_y;
    float yaw_z;
};

// 四元数
struct Quaternions {
    float q[4];
};

struct WitGyro {
    struct Acceleration acceleration;
    struct Angular_Velocity angular_velocity;
    struct EulerAngle eulerangle;
    struct Quaternions quaternions;
};

struct XsensGyro {
    struct Acceleration acceleration;
    struct Angular_Velocity angular_velocity;
    struct EulerAngle eulerangle;
    struct Quaternions quaternions;
};

const struct WitGyro get_wit_gyro_data(void);
const struct XsensGyro get_xsens_gyro_data(void);

void CalcRelativeQuaternion(const struct Quaternions* q_1, const struct Quaternions* q_2, struct Quaternions* q);
void Quaternion2EulerAngle(const struct Quaternions* q, struct EulerAngle* euler_angle);
void GetEulerAngleFromQuaternion(const struct Quaternions* q_base, const struct Quaternions* q_now, struct EulerAngle* euler_angle);
void GetQuaternionAngleFromEuler(const struct EulerAngle* euler_angle, struct Quaternions* q);
void EulerModeChange(int mode, const struct EulerAngle origin_euler, struct EulerAngle* final_euler);

#endif  // __IMU_DATA_H_
