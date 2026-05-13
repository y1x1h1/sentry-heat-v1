#ifndef CRC_H
#define CRC_H

#include <stdint.h>

uint16_t Get_CRC16_Check_Sum(uint8_t *pchMessage, uint32_t dwLength, uint16_t wCRC);
void Append_CRC16_Check_Sum(uint8_t *pchMessage, uint32_t dwLength);
uint8_t Verify_CRC16_Check_Sum(uint8_t *pchMessage, uint32_t dwLength);

#endif
