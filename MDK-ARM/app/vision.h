#ifndef VISION_H
#define VISION_H

#include <stdint.h>

#pragma pack(push, 1) // 确保 28 和 48 字节对齐

typedef struct {
    uint8_t  header;          // 0x5A
    uint8_t  detect_color : 1; 
    uint8_t  reset_tracker : 1;
    uint8_t  reserved : 6;
    float    roll;            
    float    pitch;           
    float    yaw;             
    float    aim_x;
    float    aim_y;
    float    aim_z;
    uint16_t checksum;        
} Vision_Send_Packet_t;

typedef struct {
    uint8_t  header;          // 0xA5
    uint8_t  tracking : 1;    
    uint8_t  id : 3;          
    uint8_t  armors_num : 3;
    uint8_t  reserved : 1;
    float    x;               // 目标相对坐标 (m)
    float    y;
    float    z;
    float    yaw;             
    float    vx;              
    float    vy;
    float    vz;
    float    v_yaw;
    float    r1;
    float    r2;
    float    dz;
    uint16_t checksum;        
} Vision_Recv_Packet_t;

#pragma pack(pop)

extern Vision_Recv_Packet_t vision_recv_data;
extern uint8_t vision_rx_buf[128]; // 仅声明，不分配内存

void Vision_Decode_Callback(uint8_t *data, uint16_t len);
void Vision_Send_Task(void);

#endif
