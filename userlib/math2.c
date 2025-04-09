#include "math2.h"
#include "arm_math.h"
#include <stdlib.h>

/**
 * Fast atan2
 *
 * See http://www.dspguru.com/dsp/tricks/fixed-point-atan2-with-self-normalization
 *     https://github.com/vedderb/bldc/blob/master/util/utils_math.c
 *
 * @param y
 * y
 *
 * @param x
 * x
 *
 * @return
 * The angle in radians
 */
float math2_fast_atan2(float y, float x) {
    float abs_y = fabsf(y) + 1e-20; // kludge to prevent 0/0 condition
    float angle;

    if (x >= 0) {
        float r = (x - abs_y) / (x + abs_y);
        float rsq = r * r;
        angle = ((0.1963 * rsq) - 0.9817) * r + (M_PI / 4.0);
    } else {
        float r = (x + abs_y) / (abs_y - x);
        float rsq = r * r;
        angle = ((0.1963 * rsq) - 0.9817) * r + (3.0 * M_PI / 4.0);
    }
#define UTILS_IS_INF(x)     ((x) == (1.0 / 0.0) || (x) == (-1.0 / 0.0))
#define UTILS_IS_NAN(x)     ((x) != (x))
#define UTILS_NAN_ZERO(x)   (x = UTILS_IS_NAN(x) ? 0.0 : x)
    UTILS_NAN_ZERO(angle);

    if (y < 0) {
        return(-angle);
    } else {
        return(angle);
    }
}

/**
 * ARCSIN_CORDIC returns the arcsine of an angle using the CORDIC method.
 *
 * @param input
 * Input, double T, the sine of an angle.  -1 <= T <= 1.
 * Input, int N, the number of iterations to take.
 * A value of 10 is low.  Good accuracy is achieved with 20 or more
 * iterations.
 *
 * @return
 * Output, float ARCSIN_CORDIC, an angle whose sine is T.
 *
 * @other
 * Local Parameters:
 *   Local, double ANGLES(60) = arctan ( (1/2)^(0:59) );
 */
float arcsin_cordic ( float t, int n )
{
# define ANGLES_LENGTH 60
    float angle;
    float angles[ANGLES_LENGTH] = {
      7.8539816339744830962E-01,
      4.6364760900080611621E-01,
      2.4497866312686415417E-01,
      1.2435499454676143503E-01,
      6.2418809995957348474E-02,
      3.1239833430268276254E-02,
      1.5623728620476830803E-02,
      7.8123410601011112965E-03,
      3.9062301319669718276E-03,
      1.9531225164788186851E-03,
      9.7656218955931943040E-04,
      4.8828121119489827547E-04,
      2.4414062014936176402E-04,
      1.2207031189367020424E-04,
      6.1035156174208775022E-05,
      3.0517578115526096862E-05,
      1.5258789061315762107E-05,
      7.6293945311019702634E-06,
      3.8146972656064962829E-06,
      1.9073486328101870354E-06,
      9.5367431640596087942E-07,
      4.7683715820308885993E-07,
      2.3841857910155798249E-07,
      1.1920928955078068531E-07,
      5.9604644775390554414E-08,
      2.9802322387695303677E-08,
      1.4901161193847655147E-08,
      7.4505805969238279871E-09,
      3.7252902984619140453E-09,
      1.8626451492309570291E-09,
      9.3132257461547851536E-10,
      4.6566128730773925778E-10,
      2.3283064365386962890E-10,
      1.1641532182693481445E-10,
      5.8207660913467407226E-11,
      2.9103830456733703613E-11,
      1.4551915228366851807E-11,
      7.2759576141834259033E-12,
      3.6379788070917129517E-12,
      1.8189894035458564758E-12,
      9.0949470177292823792E-13,
      4.5474735088646411896E-13,
      2.2737367544323205948E-13,
      1.1368683772161602974E-13,
      5.6843418860808014870E-14,
      2.8421709430404007435E-14,
      1.4210854715202003717E-14,
      7.1054273576010018587E-15,
      3.5527136788005009294E-15,
      1.7763568394002504647E-15,
      8.8817841970012523234E-16,
      4.4408920985006261617E-16,
      2.2204460492503130808E-16,
      1.1102230246251565404E-16,
      5.5511151231257827021E-17,
      2.7755575615628913511E-17,
      1.3877787807814456755E-17,
      6.9388939039072283776E-18,
      3.4694469519536141888E-18,
      1.7347234759768070944E-18 };

    int i;
    int j;
    float poweroftwo;
    float sigma;
    float sign_z1;
    float theta;
    float x1;
    float x2;
    float y1;
    float y2;

    if ( (1.0f < t) || (-1.0f > t) ) {
        return 0.0f;
    }

    theta = 0.0;
    x1 = 1.0;
    y1 = 0.0;
    poweroftwo = 1.0;

    for ( j = 1; j <= n; j++ )
    {
        if ( x1 < 0.0 ) {
            sign_z1 = -1.0;
        } else {
            sign_z1 = +1.0;
        }

        if ( y1 <= t ) {
            sigma = + sign_z1;
        } else {
            sigma = - sign_z1;
        }

        if ( j <= ANGLES_LENGTH ) {
            angle = angles[j-1];
        } else {
            angle = angle / 2.0;
        }

        for ( i = 1; i <= 2; i++ ) {
            x2 =                      x1 - sigma * poweroftwo * y1;
            y2 = sigma * poweroftwo * x1 +                      y1;

            x1 = x2;
            y1 = y2;
        }

        theta  = theta + 2.0 * sigma * angle;

        t = t + t * poweroftwo * poweroftwo;

        poweroftwo = poweroftwo / 2.0;
    }

    return theta;
# undef ANGLES_LENGTH
}


/**
 * @brief adc采集的原始数据转化为摇杆数据
 *
 * @param data ADC原始数据
 * @param min 原始数据最小值
 * @param mid 原始数据中值
 * @param max 原始数据最大值
 * @param target_range 目标范围，最终返回值位-target_range~target_range
 * @param inert_zone 死区
 * @return short 摇杆数据
 */
float RockerRawDataToRockerData_Float(float data,
        float min, float mid, float max,
        float target_range, float inert_zone)
{
    if (data <= (min + inert_zone))
        return -target_range;
    else if (data > (min + inert_zone) && data < (mid - inert_zone))
        return (data - (mid - inert_zone)) * target_range / (mid - min - 2 * inert_zone);
    else if (data >= (mid - inert_zone) && data <= (mid + inert_zone))
        return 0;
    else if (data > (mid + inert_zone) && data < (max - inert_zone))
        return (data - (mid + inert_zone)) * target_range / (max - mid - 2 * inert_zone);
    else if (data >= (max - inert_zone))
        return target_range;
    return 0;
}

// 限幅
void Float_Constrain(float* data, float min_value, float max_value)
{
    if(*data > max_value) *data = max_value;
    else if(*data < min_value) *data = min_value;
}

/* 角度Pid时，在获取tar和cur之后紧接着调用 */
void Handle_Angle360_PID_Over_Zero(float *tar, float *cur)
{
    if(*tar - *cur > 180)   //4096 ：半圈机械角度
    {
        *cur += 360;
    }
    else if(*tar - *cur < -180)
    {
        *cur = *cur - 360;
    }
}

float GetAngle360FromAngle180(float angle) {
    return (angle < 0.0) ? (angle + 360.0) : angle;
}

float GetAngleOver360(float angle) {
    return (angle > 360.0) ? (angle - 360.0) : angle;
}

void ScaleDownDataF(float* datas, float max_out, int num)
{
    float max_data = 0.0;
    max_out = fabs(max_out);
    for(int i=0; i< num; i++) {
        float abs_v = fabs(datas[i]);
        if(abs_v > max_data) max_data = abs_v;
    }
    if(max_out > max_data) return;
    float scale = max_out / max_data;
    for(int i=0; i< num; i++) {
        datas[i] *= scale;
    }
}

void MatrixMultiply(float A[3][3], float B[3][3], float result[3][3]) {
    int i, j, k;
    for (i = 0; i < 3; i++) {
        for (j = 0; j < 3; j++) {
            result[i][j] = 0;
            for (k = 0; k < 3; k++) {
                result[i][j] += A[i][k] * B[k][j];
            }
        }
    }
}

float x3_Slowly_Increasing(float x)
{
    if(x > 1) return 1;
    else if(x < -1) return -1;
    else {
        return x*x*x;
    }
}
