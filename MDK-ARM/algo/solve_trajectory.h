#ifndef SOLVE_TRAJECTORY_H
#define SOLVE_TRAJECTORY_H

#include "stdint.h"

typedef struct {
    float x;        // 目标相对枪口水平距离 (m)
    float y;        // 目标相对枪口垂直距离 (m)
    float v;        // 弹速 (m/s)
    float angle;    // 输出补偿后的角度 (rad)
} trajectory_t;

// 弹道解算函数：输入目标坐标和弹速，返回需要抬高的 pitch 角度
float Solve_Pitch_Control(float x, float y, float v);

#endif
