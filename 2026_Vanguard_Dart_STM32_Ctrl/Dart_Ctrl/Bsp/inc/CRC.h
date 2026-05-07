/**
 * @file crc.h
 * @author guatai (2508588132@qq.com)
 * @brief
 * @version 0.1
 * @date 2025-08-12
 *
 * @copyright Copyright (c) 2023
 *
 */

#ifndef __CRC_H__
#define __CRC_H__

#include <stdint.h>

// 注：此为裁判系统所用CRC，与USB-CDC所用算法不同
extern uint8_t Verify_CRC8_Check_Sum(uint8_t *pchMessage, uint32_t dwLength);

extern uint8_t Get_CRC8_Check_Sum(uint8_t *pchMessage,
                                  uint32_t dwLength,
                                  uint8_t ucCRC8);

extern void Append_CRC8_Check_Sum(uint8_t *pchMessage, uint32_t dwLength);

extern uint16_t Get_CRC16_Check_Sum(uint8_t *pchMessage,
                                    uint32_t dwLength,
                                    uint16_t wCRC);

extern uint32_t Verify_CRC16_Check_Sum(uint8_t *pchMessage,
                                       uint32_t dwLength);

extern void Append_CRC16_Check_Sum(uint8_t *pchMessage, uint32_t dwLength);

#endif /* __CRC_H__ */
