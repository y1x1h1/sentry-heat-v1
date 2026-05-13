#include "pid.h"

void PID_Init(pid_type_def *pid, float max_out, float max_iout, float kp, float ki, float kd) {
    pid->Kp = kp;
    pid->Ki = ki;
    pid->Kd = kd;
    pid->max_out = max_out;
    pid->max_iout = max_iout;
}

// 普通 PID 计算
float PID_Calc(pid_type_def *pid, float fdb, float set) {
    pid->set = set;
    pid->fdb = fdb;
    
    pid->error[2] = pid->error[1];
    pid->error[1] = pid->error[0];
    pid->error[0] = set - fdb;

    pid->Pout = pid->Kp * pid->error[0];
    pid->Iout += pid->Ki * pid->error[0];
    pid->Dout = pid->Kd * (pid->error[0] - pid->error[1]);

    // 积分限幅
    if (pid->Iout > pid->max_iout) pid->Iout = pid->max_iout;
    else if (pid->Iout < -pid->max_iout) pid->Iout = -pid->max_iout;

    pid->out = pid->Pout + pid->Iout + pid->Dout;

    // 输出限幅
    if (pid->out > pid->max_out) pid->out = pid->max_out;
    else if (pid->out < -pid->max_out) pid->out = -pid->max_out;

    return pid->out;
}

// 角度环专用 PID 计算 (带过零点最短物理路径处理)
float PID_Calc_Angle(pid_type_def *pid, float fdb, float set) {
    pid->set = set;
    pid->fdb = fdb;
    
    pid->error[2] = pid->error[1];
    pid->error[1] = pid->error[0];
    
    // 计算原始误差
    float error = set - fdb;
    
    // 编码器一圈是 8192，误差绝对值超过半圈(4096)说明走远路了
    // 强制它走劣弧（最短物理路径）
    if (error > 4096.0f) {
        error -= 8192.0f;
    } else if (error < -4096.0f) {
        error += 8192.0f;
    }
    
    pid->error[0] = error;

    pid->Pout = pid->Kp * pid->error[0];
    pid->Iout += pid->Ki * pid->error[0];
    pid->Dout = pid->Kd * (pid->error[0] - pid->error[1]);

    // 积分限幅
    if (pid->Iout > pid->max_iout) pid->Iout = pid->max_iout;
    else if (pid->Iout < -pid->max_iout) pid->Iout = -pid->max_iout;

    pid->out = pid->Pout + pid->Iout + pid->Dout;

    // 输出限幅
    if (pid->out > pid->max_out) pid->out = pid->max_out;
    else if (pid->out < -pid->max_out) pid->out = -pid->max_out;

    return pid->out;
}
