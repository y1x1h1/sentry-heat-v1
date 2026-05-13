#include "bsp_debug.h"
#include "usart.h"

Debug_Data_t debug_data;

/**
 * @brief 发送 6 通道波形数据到 VOFA+
 */
void Debug_Send_Waveform(float d1, float d2, float d3, float d4, float d5, float d6) {
    debug_data.data[0] = d1;
    debug_data.data[1] = d2;
    debug_data.data[2] = d3;
    debug_data.data[3] = d4;
    debug_data.data[4] = d5;
    debug_data.data[5] = d6;
    
    // JustFloat 帧尾
    debug_data.tail[0] = 0x00;
    debug_data.tail[1] = 0x00;
    debug_data.tail[2] = 0x80;
    debug_data.tail[3] = 0x7F;

    HAL_UART_Transmit_DMA(&huart7, (uint8_t *)&debug_data, sizeof(Debug_Data_t));
}