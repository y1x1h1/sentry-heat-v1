#ifndef PID_H
#define PID_H

#include "stdint.h"

typedef struct {
    float Kp;
    float Ki;
    float Kd;

    float max_out;  // 最大输出限幅
    float max_iout; // 积分限幅

    float set;      // 目标值
    float fdb;      // 反馈值

    float out;      // 总输出
    float Pout;
    float Iout;
    float Dout;

    float error[3]; // 0:当前误差, 1:上次误差, 2:上上次误差
} pid_type_def;

// 基础初始化
void PID_Init(pid_type_def *pid, float max_out, float max_iout, float kp, float ki, float kd);

// 普通 PID 计算 (用于速度环等不需要处理过零的物理量)
float PID_Calc(pid_type_def *pid, float fdb, float set);

// 处理连续圆周过零点的 PID 计算 (专用于云台角度环)
float PID_Calc_Angle(pid_type_def *pid, float fdb, float set);

#endif
