#include "gimbal.h"
#include "pid.h"
#include "bsp_can.h"
#include "vision.h"
#include "solve_trajectory.h"
#include "mahony.h"
#include <math.h>
#include "bsp_debug.h" 
#include "bsp_uart8.h"

// ==================== 常量与参数配置 ====================
#define RAD_TO_ECD           (8192.0f / 6.2831853f) 
#define YAW_CENTER_ECD       2750.0f                
#define PITCH_CENTER_ECD     1000.0f                
#define BULLET_SPEED         18.0f                  

// --- 滤波系数优化：提升基础响应速度 ---
#define VISION_FILTER_ALPHA  0.70f

// ==================== 全局变量 ====================
Gimbal_State_e gimbal_state = GIMBAL_STATE_INIT;
uint32_t state_timer = 0;

static float filtered_yaw_target = YAW_CENTER_ECD;
static float filtered_pitch_target = PITCH_CENTER_ECD;

float target_yaw_ecd = YAW_CENTER_ECD;
float target_pitch_ecd = PITCH_CENTER_ECD;

int8_t yaw_cruise_dir = 1;
const float YAW_CRUISE_STEP = 0.7f;

int8_t pitch_cruise_dir = 1;
const float PITCH_CRUISE_STEP = 2.5f; 

pid_type_def pid_yaw_angle, pid_yaw_speed;
pid_type_def pid_pitch_angle, pid_pitch_speed;

static uint8_t last_tracking_status = 0;

/**
 * @brief 带死区的 PID 角度计算
 */
float PID_Calc_Angle_With_Deadband(pid_type_def *pid, float get, float set, float deadband) {
    float err = set - get;
    if (err > 4096.0f) err -= 8192.0f;
    else if (err < -4096.0f) err += 8192.0f;
    
    if (fabsf(err) < deadband) return 0.0f;
    return PID_Calc(pid, get, set); 
}

/**
 * @brief 云台初始化
 */
void Gimbal_Init(void) {
    PID_Init(&pid_yaw_angle, 300, 0, 0.08f, 0, 0); 
    PID_Init(&pid_yaw_speed, 30000, 5000, 150.0f, 0.0f, 1.0f); 

    PID_Init(&pid_pitch_angle, 300, 0, 0.25f, 0, 0);
    PID_Init(&pid_pitch_speed, 30000, 3000, 130.0f, 0.0f, 1.5f); 
    
    gimbal_state = GIMBAL_STATE_INIT;
}

/**
 * @brief 云台控制主任务
 */
void Gimbal_Task(void) {
    static float smooth_target_yaw = YAW_CENTER_ECD;
    static float smooth_target_pitch = PITCH_CENTER_ECD;
    
    static float debug_raw_vision_yaw_ecd = YAW_CENTER_ECD;
    static float debug_raw_vision_pitch_ecd = PITCH_CENTER_ECD;

    // ================= 【核心修改：状态解析与通讯看门狗】 =================
    static uint16_t uart8_offline_timer = 0; // 通讯离线计数器

// 判断是否收到上位机新数据帧
    if (uart8_rx_update_flag == 1) {
        uart8_rx_update_flag = 0; 
        uart8_offline_timer = 0; // 喂狗

        // 直接判断我们刚加的全局变量
        // 1 就是你在主控板遥控器双下拨杆时发出的 tx_buf[4] = 1
        if (uart8_recv_mode == 1) {
            gimbal_state = GIMBAL_STATE_CHECK_IN;
        } else {
            // 防止打断开机初始化的归中过程
            if (gimbal_state != GIMBAL_STATE_INIT) {
                gimbal_state = GIMBAL_STATE_NORMAL;
            }
        }
    } else {
        // 通讯看门狗：如果连续 500 个任务周期没收到 UART8 数据 (假设任务频率1kHz，即500ms断联)
        if (++uart8_offline_timer > 500) {
            uart8_offline_timer = 500; // 防止变量溢出
            
            // 断联保护逻辑：如果正在检录时上位机掉线了，强制切回正常模式，防止电机发疯
            if (gimbal_state == GIMBAL_STATE_CHECK_IN) {
                gimbal_state = GIMBAL_STATE_NORMAL;
            }
        }
    }
    // =================================================================

    switch (gimbal_state) {
        case GIMBAL_STATE_INIT:
            target_yaw_ecd = YAW_CENTER_ECD;
            target_pitch_ecd = PITCH_CENTER_ECD;
            filtered_yaw_target = YAW_CENTER_ECD;
            filtered_pitch_target = PITCH_CENTER_ECD;
            smooth_target_yaw = YAW_CENTER_ECD;
            smooth_target_pitch = PITCH_CENTER_ECD;
            
            if (++state_timer > 1500) gimbal_state = GIMBAL_STATE_NORMAL;
            break;
                
        case GIMBAL_STATE_CHECK_IN:
        {   
            // 检录模式下，云台保持在中点
            float raw_target_yaw = YAW_CENTER_ECD;
            float raw_target_pitch = PITCH_CENTER_ECD;

            filtered_yaw_target = filtered_yaw_target * (1.0f - VISION_FILTER_ALPHA) + raw_target_yaw * VISION_FILTER_ALPHA;
            filtered_pitch_target = filtered_pitch_target * (1.0f - VISION_FILTER_ALPHA) + raw_target_pitch * VISION_FILTER_ALPHA;

            target_yaw_ecd = filtered_yaw_target;
            target_pitch_ecd = filtered_pitch_target;

            smooth_target_yaw = target_yaw_ecd;
            smooth_target_pitch = target_pitch_ecd;
            break;
        }
                
        case GIMBAL_STATE_NORMAL:
        { 
            static uint16_t lock_settle_timer = 0;

            if (vision_recv_data.tracking != last_tracking_status) {
                target_yaw_ecd = motor_yaw.ecd;
                target_pitch_ecd = motor_pitch.ecd;
                filtered_yaw_target = motor_yaw.ecd;
                filtered_pitch_target = motor_pitch.ecd;
                smooth_target_yaw = motor_yaw.ecd;
                smooth_target_pitch = motor_pitch.ecd;
                
                if (vision_recv_data.tracking == 1) {
                    lock_settle_timer = 300; 
                }
            }
            last_tracking_status = vision_recv_data.tracking;

            if (vision_recv_data.tracking == 0) {
                /* ================= 巡航模式 ================= */
                pid_yaw_angle.Kp = 0.08f;
                pid_pitch_angle.Kp = 0.25f;
                
target_yaw_ecd += yaw_cruise_dir * YAW_CRUISE_STEP;

// 累加目标角度
target_yaw_ecd += yaw_cruise_dir * YAW_CRUISE_STEP;


// 【修改这里】限制范围在 20度 (+455) 到 -60度 (-1365) 之间
if (target_yaw_ecd <= (YAW_CENTER_ECD - 1365.0f)) { 
    target_yaw_ecd = YAW_CENTER_ECD - 1365.0f; // 碰到 -60度边界，准备向正方向（往回）转
    yaw_cruise_dir = 1; 
}
else if (target_yaw_ecd >= (YAW_CENTER_ECD + 455.0f)) { 
    target_yaw_ecd = YAW_CENTER_ECD + 455.0f;  // 碰到 20度边界，准备向负方向转
    yaw_cruise_dir = -1; 
}
                target_pitch_ecd += pitch_cruise_dir * PITCH_CRUISE_STEP;
                if (target_pitch_ecd >= 1000.0f) { target_pitch_ecd = 1000.0f; pitch_cruise_dir = -1; }
                else if (target_pitch_ecd <= 600.0f) { target_pitch_ecd = 600.0f; pitch_cruise_dir = 1; }
                
                smooth_target_yaw = smooth_target_yaw * 0.90f + target_yaw_ecd * 0.10f;
                smooth_target_pitch = smooth_target_pitch * 0.90f + target_pitch_ecd * 0.10f;
                
                debug_raw_vision_yaw_ecd = target_yaw_ecd;
                debug_raw_vision_pitch_ecd = target_pitch_ecd;
            } 
else {
                /* ================= 自瞄模式 ================= */
                pid_yaw_angle.Kp = 0.8f;   
                pid_pitch_angle.Kp = 0.5f; 
                pid_yaw_speed.Kp = 150.0f;
                
                // 1. 公用计算：目标水平距离
                float dist_h = sqrtf(vision_recv_data.x * vision_recv_data.x + vision_recv_data.y * vision_recv_data.y);

                // ================= 【完美的 Yaw 轴逻辑】 =================
                // 获取目标在世界坐标系下的绝对目标角度 (单位：弧度)
                float target_yaw_rad = atan2f(vision_recv_data.y, vision_recv_data.x);
                // 计算【目标绝对角度】与【当前云台绝对角度(IMU)】的差值
                float yaw_err_rad = target_yaw_rad - imu_yaw;

                // 限制 Yaw 轴误差在 [-PI, PI] 之间，防止云台绕远路或越界
                while (yaw_err_rad > 3.14159f)  yaw_err_rad -= 6.28318f;
                while (yaw_err_rad < -3.14159f) yaw_err_rad += 6.28318f;

                // 将弧度误差转换为编码器(ECD)误差，并叠加到【当前电机位置】上
                float raw_target_yaw = motor_yaw.ecd + (yaw_err_rad * RAD_TO_ECD);

                // ================= 【完美的 Pitch 轴逻辑】 =================
                // 直接解算带弹道下坠补偿的绝对仰角 p_rad
                float p_rad = Solve_Pitch_Control(dist_h, vision_recv_data.z, BULLET_SPEED);
                
                // 恢复为原版基于机械中点计算的绝对位置
                float raw_target_pitch = PITCH_CENTER_ECD - (p_rad * RAD_TO_ECD);

                // ================= 【限位与滤波处理】 =================
                // 机械限位保护
                if (raw_target_pitch > 2000.0f) raw_target_pitch = 2000.0f;
                if (raw_target_pitch < 500.0f) raw_target_pitch = 500.0f;

                // 截取未滤波数值供调试
                debug_raw_vision_yaw_ecd = raw_target_yaw;
                debug_raw_vision_pitch_ecd = raw_target_pitch;

                // 正常的滤波处理
                filtered_yaw_target = filtered_yaw_target * (1.0f - VISION_FILTER_ALPHA) + raw_target_yaw * VISION_FILTER_ALPHA;
                filtered_pitch_target = filtered_pitch_target * (1.0f - VISION_FILTER_ALPHA) + raw_target_pitch * VISION_FILTER_ALPHA;

                // 缓冲区拦截逻辑
                if (lock_settle_timer > 0) {
                    lock_settle_timer--;
                } else {
                    smooth_target_yaw = filtered_yaw_target;
                    smooth_target_pitch = filtered_pitch_target;
                }
                
                target_yaw_ecd = smooth_target_yaw+200;//5mi +200
                target_pitch_ecd = smooth_target_pitch-700;//5mi -600
            }
        }
        break;

        default: break;
    }

    // --- PID 计算 ---
    float yaw_err = smooth_target_yaw - motor_yaw.ecd;
    if (yaw_err > 4096.0f) yaw_err -= 8192.0f;
    else if (yaw_err < -4096.0f) yaw_err += 8192.0f;
    
//    if (fabsf(yaw_err) < 8.0f) yaw_err = 0;
    float y_s = yaw_err * pid_yaw_angle.Kp;
    float y_c = PID_Calc(&pid_yaw_speed, motor_yaw.speed_rpm, y_s);
    
    float pitch_err = smooth_target_pitch - motor_pitch.ecd;
//    if (fabsf(pitch_err) < 5.0f) pitch_err = 0;
    float p_s = pitch_err * pid_pitch_angle.Kp;
    float p_c = PID_Calc(&pid_pitch_speed, motor_pitch.speed_rpm, p_s);
    // ================= 【新增：重力前馈补偿】 =================
    // 这里的 800.0f 是预估的前馈系数（你需要实测），代表水平时需要多少电流才能托住枪管
    // 如果 imu_pitch=0 是水平状态，用 cosf；如果抬头是负角，cosf(-x) 也是正的，逻辑通用。
    float gravity_comp = 1300.0f * cosf(imu_pitch); 
    
    // 补偿方向确认：
    // 如果你的云台是低头数值变小，抬头数值变大，给正电流是抬头，那就用 + 
    // 如果给正电流是低头，那就用 - 
    p_c += gravity_comp;
    CAN_Cmd_Gimbal((int16_t)y_c, (int16_t)p_c);

    // --- 串口发送波形数据 ---
    static uint8_t debug_cnt = 0;
// 临时替换你的 Debug_Send_Waveform 参数，专门看视觉原始数据
if (++debug_cnt >= 2) {
    Debug_Send_Waveform(
//        vision_recv_data.x,         // 通道 0: 看前方距离
//        vision_recv_data.y,         // 通道 1: 看左右偏置
//        vision_recv_data.z,         // 通道 2: 看绝对高度！(重点看这个)
//        (float)vision_recv_data.tracking, // 通道 3: 看是否有目标(0或1)
//        imu_pitch,                  // 通道 4: 看当前云台仰角
//        imu_yaw                     // 通道 5: 看当前云台偏航角
				            smooth_target_yaw, motor_yaw.ecd, 
            smooth_target_pitch, motor_pitch.ecd, 
            debug_raw_vision_yaw_ecd, debug_raw_vision_pitch_ecd
    );
    debug_cnt = 0;
}
    }