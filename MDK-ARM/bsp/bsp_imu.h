#ifndef BSP_IMU_H
#define BSP_IMU_H

#include "stm32f4xx_hal.h"

// 对外输出给 Mahony 算法的干净结构体
typedef struct {
    float ax, ay, az; // 加速度计数据 (g)
    float gx, gy, gz; // 陀螺仪数据 (rad/s)
    float temp;       // 温度
} imu_data_t;

extern imu_data_t imu_data;

// 函数声明，与你原有的 main.c 保持完美兼容
uint8_t BSP_IMU_Init(void);
void BSP_IMU_Update(void);

#endif