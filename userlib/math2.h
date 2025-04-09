#ifndef __MATH2_H_
#define __MATH2_H_

#include "thirdparty/arm_math/arm_math.h"

float math2_fast_atan2(float y, float x);
float arcsin_cordic(float t, int n);

float RockerRawDataToRockerData_Float(float data,
        float min, float mid, float max,
        float target_range, float inert_zone);
void Float_Constrain(float* data, float min_value, float max_value);
void Handle_Angle360_PID_Over_Zero(float *tar, float *cur);
float GetAngle360FromAngle180(float angle);
float GetAngleOver360(float angle);
void ScaleDownDataF(float* datas, float max_out, int num);
void MatrixMultiply(float A[3][3], float B[3][3], float result[3][3]);
float x3_Slowly_Increasing(float x);

#endif //__MATH2_H_
