#include "bsp_can.h"
#include "can.h" // 必须包含 CubeMX 生成的 can.h

motor_measure_t motor_yaw;
motor_measure_t motor_pitch;

motor_measure_t motor_fric_l;
motor_measure_t motor_fric_r;
motor_measure_t motor_trigger;

void BSP_CAN_Init(void) {
    CAN_FilterTypeDef can_filter_st;
    can_filter_st.FilterActivation = ENABLE;
    can_filter_st.FilterMode = CAN_FILTERMODE_IDMASK;
    can_filter_st.FilterScale = CAN_FILTERSCALE_32BIT;
    can_filter_st.FilterIdHigh = 0x0000;
    can_filter_st.FilterIdLow = 0x0000;
    can_filter_st.FilterMaskIdHigh = 0x0000;
    can_filter_st.FilterMaskIdLow = 0x0000;
    can_filter_st.FilterBank = 0;
    can_filter_st.FilterFIFOAssignment = CAN_RX_FIFO0;
    
    HAL_CAN_ConfigFilter(&hcan1, &can_filter_st);
    HAL_CAN_Start(&hcan1);
    HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);
}

// ================== 【关键新增】 ==================
// 把解包和角度累计逻辑封装成一个通用函数
// ================== 【关键新增】 ==================
// 把解包和角度累计逻辑封装成一个通用函数
void decode_motor_measure(motor_measure_t *ptr, uint8_t *data) {
    ptr->last_ecd = ptr->ecd;
    ptr->ecd = (data[0] << 8) | data[1];
    ptr->speed_rpm = (data[2] << 8) | data[3];
    ptr->given_current = (data[4] << 8) | data[5];
    ptr->temperate = data[6];

    // 【必须新增的核心逻辑：处理过零点，计算真正圈数】
    // 大疆电机的 ecd 范围是 0~8191
    if (ptr->ecd - ptr->last_ecd > 4096) {
        ptr->round_cnt--;
    } else if (ptr->ecd - ptr->last_ecd < -4096) {
        ptr->round_cnt++;
    }
    
    // 计算电机的连续累计总角度 (转一圈是360度，大疆电机分辨率是8192)
    ptr->total_angle = ptr->round_cnt * 360.0f + (ptr->ecd * 360.0f / 8192.0f);
}
// ==================================================
// ==================================================

// CAN 接收中断回调函数
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan) {
    CAN_RxHeaderTypeDef rx_header;
    uint8_t rx_data[8];

    HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, rx_data);

    if (hcan->Instance == CAN1) {
        // 使用新写的 decode 函数直接进行解析！
        switch (rx_header.StdId) {
            case 0x201: decode_motor_measure(&motor_fric_l, rx_data); break;
            case 0x202: decode_motor_measure(&motor_fric_r, rx_data); break;
            case 0x203: decode_motor_measure(&motor_trigger, rx_data); break;
            case 0x205: decode_motor_measure(&motor_yaw, rx_data); break;
            case 0x206: decode_motor_measure(&motor_pitch, rx_data); break;
            default: break;
        }
    }
}
// （发送函数 CAN_Cmd_Gimbal 和 CAN_Cmd_Shooter 保持你原来的代码即可...）
void CAN_Cmd_Gimbal(int16_t yaw_v, int16_t pitch_v) {
    CAN_TxHeaderTypeDef tx_header;
    uint8_t tx_data[8] = {0};
    uint32_t send_mail_box;
    tx_header.StdId = 0x1FF;
    tx_header.IDE = CAN_ID_STD;
    tx_header.RTR = CAN_RTR_DATA;
    tx_header.DLC = 8;
    tx_data[0] = yaw_v >> 8;
    tx_data[1] = yaw_v;
    tx_data[2] = pitch_v >> 8;
    tx_data[3] = pitch_v;
    HAL_CAN_AddTxMessage(&hcan1, &tx_header, tx_data, &send_mail_box);
}

void CAN_Cmd_Shooter(int16_t motor1, int16_t motor2, int16_t motor3) {
    CAN_TxHeaderTypeDef TxHeader;
    uint8_t TxData[8];
    uint32_t TxMailbox;
    TxHeader.StdId = 0x200; 
    TxHeader.IDE = CAN_ID_STD;
    TxHeader.RTR = CAN_RTR_DATA;
    TxHeader.DLC = 8;
    TxData[0] = motor1 >> 8;
    TxData[1] = motor1;
    TxData[2] = motor2 >> 8;
    TxData[3] = motor2;
    TxData[4] = motor3 >> 8;
    TxData[5] = motor3;
    HAL_CAN_AddTxMessage(&hcan1, &TxHeader, TxData, &TxMailbox); 
}
// 【注意】：请在这一行下面敲一个回车，留一个空行！                                                   咩咩咩咩咩咩                         