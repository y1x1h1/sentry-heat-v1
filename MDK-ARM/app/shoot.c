#include "shoot.h"
#include "pid.h"
#include "bsp_can.h"
#include "vision.h"
#include "gimbal.h"
#include "bsp_uart8.h"  
#include "math.h"

// ================= RMUL 2026 哨兵射击参数 =================
#define SHOOTER_HEAT_LIMIT      260      // 哨兵热量上限 
#define HEAT_PER_BULLET         10       // 17mm 单发热量
#define HEAT_SAFE_BUFFER        30       // 预留缓冲热量 (抵抗延迟，防超限)

// ================= 单向 7 孔拨弹盘参数 =================
#define TRIGGER_GEAR_RATIO      36.0f    // M2006 电机减速比 36:1
#define HOLES_PER_ROUND         7.0f     // 拨弹盘一圈 7 孔
// 单发子弹对应的电机轴角度增量 = 36 * 360 / 7 ≈ 1851.43度
#define ANGLE_PER_BULLET        (TRIGGER_GEAR_RATIO * 360.0f / HOLES_PER_ROUND)

// 判定电机已到达目标位置的死区 (度)
#define POS_DEADBAND            15.0f

// 摩擦轮目标转速 
#define FRIC_SPEED              6500.0f 

// ================= PID 实例化 =================
pid_type_def pid_fric_l_speed;
pid_type_def pid_fric_r_speed;
pid_type_def pid_trigger_angle;
pid_type_def pid_trigger_speed;

extern Gimbal_State_e gimbal_state;

static float trigger_target_pos = 0.0f;    
static uint8_t is_trigger_initialized = 0; 

/**
 * @brief 射击机构初始化
 */
void Shoot_Init(void) {
    // 摩擦轮 PID (纯速度环)
    PID_Init(&pid_fric_l_speed, 16384, 5000, 15.0f, 0.5f, 0.0f);
    PID_Init(&pid_fric_r_speed, 16384, 5000, 15.0f, 0.5f, 0.0f);
    
    // 拨弹盘 PID (串级双环：位置环 -> 速度环)
    PID_Init(&pid_trigger_angle, 60, 0, 0.5f, 0.0f, 0.0f);   
    PID_Init(&pid_trigger_speed, 5000, 500, 10.0f, 0.5f, 0.0f); 
}

/**
 * @brief 射击控制主任务
 */
void Shoot_Task(void) {
    float set_fric_l = 0;
    float set_fric_r = 0;
    float set_trigger_speed = 0;
    float out_trigger_current = 0;

    // 获取电机当前连续累计角度 (单位: 度)
    float current_trigger_pos = motor_trigger.total_angle; 

    // 1. 上电初始化：以当前静止位置为原点起点
    if (is_trigger_initialized == 0) {
        trigger_target_pos = current_trigger_pos;
        is_trigger_initialized = 1;
    }

    // 2. 热量安全检查
    uint8_t is_heat_safe = ((receiver_heat + HEAT_PER_BULLET + HEAT_SAFE_BUFFER) < SHOOTER_HEAT_LIMIT);

    // ================== 核心发射状态机 ==================
    if (gimbal_state == GIMBAL_STATE_CHECK_IN) {
        set_fric_l = FRIC_SPEED;       
        set_fric_r = -FRIC_SPEED;      
        
        if (fabsf(motor_yaw.ecd - YAW_CENTER_ECD) < 200.0f && 
            fabsf(motor_pitch.ecd - PITCH_CENTER_ECD) < 200.0f) {
            
            if (fabsf(trigger_target_pos - current_trigger_pos) < POS_DEADBAND && is_heat_safe) {
                // 【核心修正】：原速是 -1000，说明电机需要反转出弹！这里必须是 -=
                trigger_target_pos -= ANGLE_PER_BULLET; 
            }
        }
    } 
    else if (vision_recv_data.tracking == 1) {
        set_fric_l = FRIC_SPEED;       
        set_fric_r = -FRIC_SPEED;      
        
        if (can_shoot_flag == 1) {
            if (fabsf(trigger_target_pos - current_trigger_pos) < POS_DEADBAND && is_heat_safe) {
                // 【核心修正】：反向递减累加！
                trigger_target_pos -= ANGLE_PER_BULLET; 
            }
        } 
    } 
    else {
        set_fric_l = 0;
        set_fric_r = 0;
    }

    // ================== PID 计算 ==================
    float out_fric_l = PID_Calc(&pid_fric_l_speed, motor_fric_l.speed_rpm, set_fric_l);
    float out_fric_r = PID_Calc(&pid_fric_r_speed, motor_fric_r.speed_rpm, set_fric_r);
    
    // 拨弹盘串级计算
    set_trigger_speed = PID_Calc(&pid_trigger_angle, current_trigger_pos, trigger_target_pos);
    out_trigger_current = PID_Calc(&pid_trigger_speed, motor_trigger.speed_rpm, set_trigger_speed);

    // ================== 单向防卡弹自救保护 ==================
    static uint32_t block_start_time = 0;
    
    // 判断堵转：目标位置与当前位置差距大 (绝对值判断，不受正反转影响)，且电机转速极低
    if (fabsf(trigger_target_pos - current_trigger_pos) > 30.0f && fabsf(motor_trigger.speed_rpm) < 10.0f) {
        if (block_start_time == 0) {
            block_start_time = HAL_GetTick(); 
        }
        
        if (HAL_GetTick() - block_start_time > 400)  {  
            // 切断输出电流 500ms 尝试让弹丸落位
            out_trigger_current = 0; 
            
            if (HAL_GetTick() - block_start_time > 900) { 
                block_start_time = 0; 
            } 
    } else { 
        block_start_time = 0; 
    }

    // ================== 发送指令 ==================
    CAN_Cmd_Shooter((int16_t)out_fric_l, (int16_t)out_fric_r, (int16_t)out_trigger_current);
}
}