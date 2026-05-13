#include "vision.h"
#include "usart.h"
#include "crc.h"
#include "string.h"
#include "mahony.h"

Vision_Recv_Packet_t vision_recv_data;
Vision_Send_Packet_t vision_send_data;
//float PITCH_OFFSET=0.10f;
void Vision_Decode_Callback(uint8_t *data, uint16_t len) {
    if (len == sizeof(Vision_Recv_Packet_t) && data[0] == 0xA5) {
        if (Verify_CRC16_Check_Sum(data, sizeof(Vision_Recv_Packet_t))) {
            memcpy(&vision_recv_data, data, sizeof(Vision_Recv_Packet_t));
        }
    }
}

void Vision_Send_Task(void) {
    vision_send_data.header = 0x5A;
    vision_send_data.detect_color = 1; 
    vision_send_data.reset_tracker = 0;
    vision_send_data.reserved = 0;
//float real_gun_pitch = imu_pitch - PITCH_OFFSET;
    // 发送 Mahony 解算出的绝对弧度角
    vision_send_data.roll  = 0;//imu_roll; 
    vision_send_data.pitch = imu_pitch; 
    vision_send_data.yaw   = imu_yaw;   

    vision_send_data.aim_x = 0.0f;
    vision_send_data.aim_y = 0.0f;
    vision_send_data.aim_z = 0.0f;

    // 结构体 1 字节对齐确保 sizeof(Vision_Send_Packet_t) 为 28
    Append_CRC16_Check_Sum((uint8_t*)&vision_send_data, sizeof(Vision_Send_Packet_t));

    HAL_UART_Transmit_DMA(&huart6, (uint8_t*)&vision_send_data, sizeof(Vision_Send_Packet_t));
}
