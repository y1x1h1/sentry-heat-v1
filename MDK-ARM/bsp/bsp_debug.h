#ifndef BSP_DEBUG_H
#define BSP_DEBUG_H

#include "stdint.h"

// 定义 JustFloat 协议结构体 (扩展为 6 通道)
#pragma pack(push, 1)
typedef struct {
    float data[6];      // 通道数据
    uint8_t tail[4];    // 帧尾: 0x00 0x00 0x80 0x7F
} Debug_Data_t;
#pragma pack(pop)

void Debug_Send_Waveform(float d1, float d2, float d3, float d4, float d5, float d6);

#endif