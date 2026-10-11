#include "mahonyRK4.h"
#include <math.h>

static void quaternion_rk4(float gx, float gy, float gz, float dt,
                           float *q0, float *q1, float *q2, float *q3);
static void update_euler_angles(void);

#define DEFAULT_Kp  2.0f
#define DEFAULT_Ki  0.005f
#define RAD_TO_DEG  57.2957795f

// 内部状态
static float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f;
static float exInt = 0.0f, eyInt = 0.0f, ezInt = 0.0f;
static float Kp = DEFAULT_Kp, Ki = DEFAULT_Ki;

// 输出欧拉角（度）
float roll = 0.0f;
float pitch = 0.0f;
float yaw = 0.0f;

// 快速平方根倒数
static float invSqrt(float x)
{
    return 1.0f / sqrtf(x);
}

void Mahony_Init(void)
{
    q0 = 1.0f; q1 = 0.0f; q2 = 0.0f; q3 = 0.0f;
    exInt = 0.0f; eyInt = 0.0f; ezInt = 0.0f;
    Kp = DEFAULT_Kp;
    Ki = DEFAULT_Ki;
    roll = pitch = yaw = 0.0f;
}

void Mahony_SetPI(float kp, float ki)
{
    Kp = kp;
    Ki = ki;
}

void Mahony_Update_IMU(float gx, float gy, float gz,
                       float ax, float ay, float az,
                       float dt)
{
    float norm;
    float vx, vy, vz;
    float ex, ey, ez;

    // 加速度无效则退化为纯陀螺积分
    if ((ax * ax + ay * ay + az * az) < 1e-8f)
    {
        quaternion_rk4(gx, gy, gz, dt, &q0, &q1, &q2, &q3);
        update_euler_angles();
        return;
    }

    // 归一化加速度
    norm = invSqrt(ax * ax + ay * ay + az * az);
    ax *= norm;
    ay *= norm;
    az *= norm;

    // 由当前四元数估计重力方向
    vx = 2.0f * (q1 * q3 - q0 * q2);
    vy = 2.0f * (q0 * q1 + q2 * q3);
    vz = q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3;

    // 叉积误差
    ex = (ay * vz - az * vy);
    ey = (az * vx - ax * vz);
    ez = (ax * vy - ay * vx);

    // 积分项
    if (Ki > 0.0f)
    {
        exInt += ex * Ki * dt;
        eyInt += ey * Ki * dt;
        ezInt += ez * Ki * dt;
    }
    else
    {
        exInt = 0.0f;
        eyInt = 0.0f;
        ezInt = 0.0f;
    }

    // 比例 + 积分修正陀螺
    gx += Kp * ex + exInt;
    gy += Kp * ey + eyInt;
    gz += Kp * ez + ezInt;

    quaternion_rk4(gx, gy, gz, dt, &q0, &q1, &q2, &q3);
    update_euler_angles();
}

// 纯陀螺更新：给 imu.c 弯道门控时用
void Mahony_Update_GyroOnly(float gx, float gy, float gz, float dt)
{
    quaternion_rk4(gx, gy, gz, dt, &q0, &q1, &q2, &q3);
    update_euler_angles();
}

void Mahony_GetAngles(float *r, float *p, float *y)
{
    if (r) *r = roll;
    if (p) *p = pitch;
    if (y) *y = yaw;
}

static void update_euler_angles(void)
{
    pitch = asinf(-2.0f * q1 * q3 + 2.0f * q0 * q2) * RAD_TO_DEG;
    roll  = atan2f(2.0f * q2 * q3 + 2.0f * q0 * q1,
                   -2.0f * q1 * q1 - 2.0f * q2 * q2 + 1.0f) * RAD_TO_DEG;
    yaw   = atan2f(2.0f * (q1 * q2 + q0 * q3),
                   q0 * q0 + q1 * q1 - q2 * q2 - q3 * q3) * RAD_TO_DEG;
}

static void quaternion_rk4(float gx, float gy, float gz, float dt,
                           float *q0, float *q1, float *q2, float *q3)
{
    float k00, k01, k02, k03;
    float k10, k11, k12, k13;
    float k20, k21, k22, k23;
    float k30, k31, k32, k33;

    float q0_tmp, q1_tmp, q2_tmp, q3_tmp;

    k00 = 0.5f * (-(*q1)*gx - (*q2)*gy - (*q3)*gz);
    k01 = 0.5f * ( (*q0)*gx + (*q2)*gz - (*q3)*gy);
    k02 = 0.5f * ( (*q0)*gy - (*q1)*gz + (*q3)*gx);
    k03 = 0.5f * ( (*q0)*gz + (*q1)*gy - (*q2)*gx);

    q0_tmp = *q0 + 0.5f * dt * k00;
    q1_tmp = *q1 + 0.5f * dt * k01;
    q2_tmp = *q2 + 0.5f * dt * k02;
    q3_tmp = *q3 + 0.5f * dt * k03;

    k10 = 0.5f * (-q1_tmp*gx - q2_tmp*gy - q3_tmp*gz);
    k11 = 0.5f * ( q0_tmp*gx + q2_tmp*gz - q3_tmp*gy);
    k12 = 0.5f * ( q0_tmp*gy - q1_tmp*gz + q3_tmp*gx);
    k13 = 0.5f * ( q0_tmp*gz + q1_tmp*gy - q2_tmp*gx);

    q0_tmp = *q0 + 0.5f * dt * k10;
    q1_tmp = *q1 + 0.5f * dt * k11;
    q2_tmp = *q2 + 0.5f * dt * k12;
    q3_tmp = *q3 + 0.5f * dt * k13;

    k20 = 0.5f * (-q1_tmp*gx - q2_tmp*gy - q3_tmp*gz);
    k21 = 0.5f * ( q0_tmp*gx + q2_tmp*gz - q3_tmp*gy);
    k22 = 0.5f * ( q0_tmp*gy - q1_tmp*gz + q3_tmp*gx);
    k23 = 0.5f * ( q0_tmp*gz + q1_tmp*gy - q2_tmp*gx);

    q0_tmp = *q0 + dt * k20;
    q1_tmp = *q1 + dt * k21;
    q2_tmp = *q2 + dt * k22;
    q3_tmp = *q3 + dt * k23;

    k30 = 0.5f * (-q1_tmp*gx - q2_tmp*gy - q3_tmp*gz);
    k31 = 0.5f * ( q0_tmp*gx + q2_tmp*gz - q3_tmp*gy);
    k32 = 0.5f * ( q0_tmp*gy - q1_tmp*gz + q3_tmp*gx);
    k33 = 0.5f * ( q0_tmp*gz + q1_tmp*gy - q2_tmp*gx);

    *q0 += dt / 6.0f * (k00 + 2.0f*k10 + 2.0f*k20 + k30);
    *q1 += dt / 6.0f * (k01 + 2.0f*k11 + 2.0f*k21 + k31);
    *q2 += dt / 6.0f * (k02 + 2.0f*k12 + 2.0f*k22 + k32);
    *q3 += dt / 6.0f * (k03 + 2.0f*k13 + 2.0f*k23 + k33);

    {
        float norm = invSqrt((*q0)*(*q0) + (*q1)*(*q1) + (*q2)*(*q2) + (*q3)*(*q3));
        *q0 *= norm;
        *q1 *= norm;
        *q2 *= norm;
        *q3 *= norm;
    }
}
