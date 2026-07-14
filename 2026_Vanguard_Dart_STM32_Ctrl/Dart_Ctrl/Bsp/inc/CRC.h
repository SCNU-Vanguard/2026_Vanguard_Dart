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

#ifndef __CRC_H__ /* 按 __CRC_H__ 选择编译分支。 */
#define __CRC_H__ /* 定义 __CRC_H__。 */

#include <stdint.h>

// 注：此为裁判系统所用CRC，与USB-CDC所用算法不同
extern uint8_t Verify_CRC8_Check_Sum(uint8_t *pchMessage, uint32_t dwLength); /* 声明 Verify_CRC8_Check_Sum 接口。 */

extern uint8_t Get_CRC8_Check_Sum(uint8_t *pchMessage, /* 传入下一项参数或数据。 */
                                  uint32_t dwLength, /* 传入下一项参数或数据。 */
                                  uint8_t ucCRC8); /* 完成本行操作。 */

extern void Append_CRC8_Check_Sum(uint8_t *pchMessage, uint32_t dwLength); /* 声明 Append_CRC8_Check_Sum 接口。 */

extern uint16_t Get_CRC16_Check_Sum(uint8_t *pchMessage, /* 传入下一项参数或数据。 */
                                    uint32_t dwLength, /* 传入下一项参数或数据。 */
                                    uint16_t wCRC); /* 完成本行操作。 */

extern uint32_t Verify_CRC16_Check_Sum(uint8_t *pchMessage, /* 传入下一项参数或数据。 */
                                       uint32_t dwLength); /* 完成本行操作。 */

extern void Append_CRC16_Check_Sum(uint8_t *pchMessage, uint32_t dwLength); /* 声明 Append_CRC16_Check_Sum 接口。 */

#endif /* __CRC_H__ */
