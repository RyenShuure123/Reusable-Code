#ifndef MAHONY_H
#define MAHONY_H

// 全局欧拉角（单位：度）
extern float roll;
extern float pitch;
extern float yaw;

/**
 * @brief  Mahony 算法初始化（重置四元数）
 */
void Mahony_Init(void);

/**
 * @brief  设置 PI 参数（可根据需要修改默认值）
 * @param  kp: 比例增益
 * @param  ki: 积分增益
 */
void Mahony_SetPI(float kp, float ki);

/**
 * @brief  Mahony 算法更新（纯 IMU 版本，无磁力计）
 * @param  gx, gy, gz : 陀螺仪数据（单位：弧度/秒 rad/s）
 * @param  ax, ay, az : 加速度计数据（单位：g，即 1g = 9.8m/s²）
 * @param  dt         : 采样周期（单位：秒）
 */
void Mahony_Update_IMU(float gx, float gy, float gz,
                       float ax, float ay, float az,
                       float dt);

/**
 * @brief  获取当前的欧拉角（单位：度）
 * @param  r: 指向 roll 的指针
 * @param  p: 指向 pitch 的指针
 * @param  y: 指向 yaw 的指针
 */
void Mahony_GetAngles(float *r, float *p, float *y);

void Mahony_Update_GyroOnly(float gx, float gy, float gz, float dt);

#endif
