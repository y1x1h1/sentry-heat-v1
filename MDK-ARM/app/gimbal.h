#ifndef GIMBAL_H
#define GIMBAL_H

#include "stdint.h"

// 定义云台的运行状态枚举
typedef enum {
    GIMBAL_STATE_INIT = 0,    // 初始化归中状态
    GIMBAL_STATE_NORMAL,      // 正常运行状态 (包含巡航和自瞄)
    GIMBAL_STATE_ERROR_RESET,  // 异常发散后的回正状态
		 GIMBAL_STATE_CHECK_IN // 新增：检录状态
} Gimbal_State_e;

#define RAD_TO_ECD           (8192.0f / 6.2831853f) 
#define YAW_CENTER_ECD       2750.0f                
#define PITCH_CENTER_ECD     1000.0f                
#define BULLET_SPEED         18.0f                  

// 暴露云台状态供外部（如裁判系统/UI）读取
extern Gimbal_State_e gimbal_state;

void Gimbal_Init(void);
void Gimbal_Task(void);

#endif // GIMBAL_H
