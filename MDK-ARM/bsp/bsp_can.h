#ifndef BSP_CAN_H
#define BSP_CAN_H

#include "stdint.h"

// 电机反馈数据结构体
typedef struct {
    uint16_t ecd;          // 编码器数值 (0-8191)
    uint16_t last_ecd;     // 【新增】上一次编码器数值
    int16_t speed_rpm;     // 转速
    int16_t given_current; // 实际电流
    uint8_t temperate;     // 温度
    
    int32_t round_cnt;     // 【新增】转过的圈数
    float total_angle;     // 【新增】连续累计总角度 (单位: 度)
} motor_measure_t;

extern motor_measure_t motor_yaw;
extern motor_measure_t motor_pitch;

extern motor_measure_t motor_fric_l;    // ID 1 (3508左摩擦轮)
extern motor_measure_t motor_fric_r;    // ID 2 (3508右摩擦轮)
extern motor_measure_t motor_trigger;   // ID 3 (2006拨弹盘)

void BSP_CAN_Init(void);
void CAN_Cmd_Gimbal(int16_t yaw_v, int16_t pitch_v);
void CAN_Cmd_Shooter(int16_t motor1, int16_t motor2, int16_t motor3);

#endif
// 【注意】：请在这一行下面敲一个回车，留一个空行！