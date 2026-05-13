#ifndef BSP_UART8_H
#define BSP_UART8_H

#include "stdint.h"

extern uint8_t uart8_rx_update_flag; // 专门用于表示串口收到新数据
extern uint8_t can_shoot_flag;   
extern uint16_t receiver_heat;   
extern uint8_t uart8_recv_mode;      // 【新增】暴露给外部使用的模式变量
extern uint8_t uart8_rx_buf[16]; 

void BSP_UART8_Init(void);
void UART8_Parse_Data(uint8_t *buf, uint16_t len); 

#endif