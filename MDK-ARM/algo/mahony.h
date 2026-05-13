#ifndef MAHONY_H
#define MAHONY_H

#include "stdint.h"

// ==================== 欧拉角全局变量 (单位: Radian) ====================
// 这些变量由 vision.c 引用并发送给上位机
extern float imu_pitch; 
extern float imu_yaw;   
extern float imu_roll;  

// ==================== 四元数全局变量 ====================
extern float q0, q1, q2, q3;

/**
 * @brief Mahony 姿态解算算法更新
 * @param gx, gy, gz 陀螺仪数据 (单位: rad/s)
 * @param ax, ay, az 加速度计数据 (单位: g)
 */
void Mahony_Update(float gx, float gy, float gz, float ax, float ay, float az);

#endif
