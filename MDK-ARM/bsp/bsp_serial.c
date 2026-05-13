#include "bsp_serial.h"
#include "usart.h"
#include "vision.h"

#define VISION_RX_BUF_SIZE 128
uint8_t vision_rx_buf[VISION_RX_BUF_SIZE];

void BSP_Vision_Init(void) {
    // 开启 USART6 的空闲中断
    __HAL_UART_ENABLE_IT(&huart6, UART_IT_IDLE);
    // 开启第一次 DMA 接收
    HAL_UART_Receive_DMA(&huart6, vision_rx_buf, VISION_RX_BUF_SIZE);
}

// 拦截 USART6 的底层中断 (请去 stm32f4xx_it.c 中把 USART6_IRQHandler 里的内容注释掉，或在此利用 HAL 提供的钩子)
// 推荐做法是在 stm32f4xx_it.c 里的 USART6_IRQHandler 中添加如下代码：
/* void USART6_IRQHandler(void)
{
    // 如果发生了空闲中断
    if (__HAL_UART_GET_FLAG(&huart6, UART_FLAG_IDLE) != RESET) {
        __HAL_UART_CLEAR_IDLEFLAG(&huart6); // 清除标志位
        HAL_UART_DMAStop(&huart6);          // 停止本次 DMA

        // 计算接收到了多少字节
        uint16_t len = VISION_RX_BUF_SIZE - __HAL_DMA_GET_COUNTER(huart6.hdmarx);
        
        // 传递给应用层解码
        Vision_Decode_Callback(vision_rx_buf, len);
        
        // 重新开启 DMA 接收下一帧
        HAL_UART_Receive_DMA(&huart6, vision_rx_buf, VISION_RX_BUF_SIZE);
    }
    HAL_UART_IRQHandler(&huart6);
}
*/
