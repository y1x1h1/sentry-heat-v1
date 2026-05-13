#include "mahony.h"
#include <math.h>

// ==================== 算法常量 ====================
// 比例增益 Kp 决定加速度计纠正陀螺仪漂移的速度
#define Kp_Mahony 2.0f
// 积分增益 Ki 用于消除常值偏差
#define Ki_Mahony 0.01f
// 采样周期 dt (必须与 TIM6 中断频率 1000Hz 严格对应)
#define IMU_DT    0.001f 

// ==================== 全局变量 ====================
float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f;
float integralFBx = 0.0f, integralFBy = 0.0f, integralFBz = 0.0f;

float imu_pitch = 0.0f; 
float imu_yaw = 0.0f;   
float imu_roll = 0.0f;

/**
 * @brief Mahony 姿态解算更新
 * @note 必须在 1ms 定时器中断中调用
 */
void Mahony_Update(float gx, float gy, float gz, float ax, float ay, float az) {
    float norm;
    float vx, vy, vz;
    float ex, ey, ez;
    float pa, pb, pc;

    // 1. 加速度计数据归一化
    norm = sqrtf(ax * ax + ay * ay + az * az);
    if (norm > 0.0f) {
        ax /= norm;
        ay /= norm;
        az /= norm;

        // 2. 提取当前四元数下的重力分量 (机体坐标系)
        vx = 2.0f * (q1 * q3 - q0 * q2);
        vy = 2.0f * (q0 * q1 + q2 * q3);
        vz = q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3;

        // 3. 计算重力向量误差 (叉乘)
        ex = (ay * vz - az * vy);
        ey = (az * vx - ax * vz);
        ez = (ax * vy - ay * vx);

        // 4. 误差补偿
        if (Ki_Mahony > 0.0f) {
            integralFBx += ex * Ki_Mahony * IMU_DT;
            integralFBy += ey * Ki_Mahony * IMU_DT;
            integralFBz += ez * Ki_Mahony * IMU_DT;
            gx += integralFBx;
            gy += integralFBy;
            gz += integralFBz;
        }
        gx += Kp_Mahony * ex;
        gy += Kp_Mahony * ey;
        gz += Kp_Mahony * ez;
    }

    // 5. 四元数微分方程更新 (一阶龙格库塔)
    pa = q1; pb = q2; pc = q3;
    q0 += (-pa * gx - pb * gy - pc * gz) * (0.5f * IMU_DT);
    q1 += (q0 * gx + pb * gz - pc * gy) * (0.5f * IMU_DT);
    q2 += (q0 * gy - pa * gz + pc * gx) * (0.5f * IMU_DT);
    q3 += (q0 * gz + pa * gy - pb * gx) * (0.5f * IMU_DT);

    // 6. 四元数归一化
    norm = sqrtf(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
    if (norm > 0.0f) {
        q0 /= norm; q1 /= norm; q2 /= norm; q3 /= norm;
    }

    // 7. 转换输出欧拉角 (弧度 Radian)
    // Pitch: 俯仰角
    imu_pitch = asin(-2.0f * q1 * q3 + 2.0f * q0 * q2);
    // Roll: 横滚角
    imu_roll  = atan2(2.0f * q2 * q3 + 2.0f * q0 * q1, q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3);
    // Yaw: 航向角 (由陀螺仪 Z 轴直接积分，保证量程线性)
    imu_yaw  += gz * IMU_DT; 
}
