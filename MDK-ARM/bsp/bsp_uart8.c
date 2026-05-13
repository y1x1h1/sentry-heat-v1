#include "bsp_uart8.h"
#include "usart.h"

uint8_t uart8_rx_buf[16];
uint8_t uart8_rx_update_flag = 0; // 独立的接收标志位
uint8_t can_shoot_flag = 0;       // 默认不开火，等上位机指令
uint16_t receiver_heat = 0;       
uint8_t uart8_recv_mode = 0;      // 【新增】存放解析出来的云台模式 (例如检录模式)

void BSP_UART8_Init(void) {
    __HAL_UART_ENABLE_IT(&huart8, UART_IT_IDLE); 
    HAL_UART_Receive_DMA(&huart8, uart8_rx_buf, sizeof(uart8_rx_buf));
}

void UART8_Parse_Data(uint8_t *buf, uint16_t len) {
    // 【修改1】一帧数据变成了 7 个字节，所以最短长度判断改为 7
    if (len < 7) return; 

    // 【修改2】遍历边界改为 len - 7
    for (uint16_t i = 0; i <= len - 7; i++) {
        
        // 【修改3】帧尾现在是第 7 个字节，即 buf[i + 6]
        if (buf[i] == 0xA5 && buf[i + 6] == 0x5A) {
            
            // 【修改4】校验和计算，必须加上你的模式位 buf[i + 4]
            uint8_t sum = buf[i + 1] + buf[i + 2] + buf[i + 3] + buf[i + 4];
            
            // 【修改5】校验和现在存放在第 6 个字节，即 buf[i + 5]
            if (sum == buf[i + 5]) {
                
                can_shoot_flag = buf[i + 1]; 
                receiver_heat = (uint16_t)((buf[i + 2] << 8) | buf[i + 3]);
                
                // 【新增核心逻辑】把解包出来的 mode 保存下来，供 gimbal.c 使用
                uart8_recv_mode = buf[i + 4]; 
                
                uart8_rx_update_flag = 1; // 标记成功接收并解包了一帧数据
                break; 
            }
        }
    }
}