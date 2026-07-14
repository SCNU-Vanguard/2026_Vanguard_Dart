/**
******************************************************************************
* @file    crc.c
* @brief
* @author
******************************************************************************
* Copyright (c) 2023 Team
* All rights reserved.
******************************************************************************
*/

#include <string.h>
#include <stdlib.h>
#include "CRC.h"

const uint8_t CRC8_INIT = 0xff; /* 初始化 CRC8_INIT。 */
const uint8_t CRC8_TAB[256] = { /* 初始化 CRC8_TAB。 */
    0x00, /* 提供查表常量。 */
    0x5e, /* 提供查表常量。 */
    0xbc, /* 提供查表常量。 */
    0xe2, /* 提供查表常量。 */
    0x61, /* 提供查表常量。 */
    0x3f, /* 提供查表常量。 */
    0xdd, /* 提供查表常量。 */
    0x83, /* 提供查表常量。 */
    0xc2, /* 提供查表常量。 */
    0x9c, /* 提供查表常量。 */
    0x7e, /* 提供查表常量。 */
    0x20, /* 提供查表常量。 */
    0xa3, /* 提供查表常量。 */
    0xfd, /* 提供查表常量。 */
    0x1f, /* 提供查表常量。 */
    0x41, /* 提供查表常量。 */
    0x9d, /* 提供查表常量。 */
    0xc3, /* 提供查表常量。 */
    0x21, /* 提供查表常量。 */
    0x7f, /* 提供查表常量。 */
    0xfc, /* 提供查表常量。 */
    0xa2, /* 提供查表常量。 */
    0x40, /* 提供查表常量。 */
    0x1e, /* 提供查表常量。 */
    0x5f, /* 提供查表常量。 */
    0x01, /* 提供查表常量。 */
    0xe3, /* 提供查表常量。 */
    0xbd, /* 提供查表常量。 */
    0x3e, /* 提供查表常量。 */
    0x60, /* 提供查表常量。 */
    0x82, /* 提供查表常量。 */
    0xdc, /* 提供查表常量。 */
    0x23, /* 提供查表常量。 */
    0x7d, /* 提供查表常量。 */
    0x9f, /* 提供查表常量。 */
    0xc1, /* 提供查表常量。 */
    0x42, /* 提供查表常量。 */
    0x1c, /* 提供查表常量。 */
    0xfe, /* 提供查表常量。 */
    0xa0, /* 提供查表常量。 */
    0xe1, /* 提供查表常量。 */
    0xbf, /* 提供查表常量。 */
    0x5d, /* 提供查表常量。 */
    0x03, /* 提供查表常量。 */
    0x80, /* 提供查表常量。 */
    0xde, /* 提供查表常量。 */
    0x3c, /* 提供查表常量。 */
    0x62, /* 提供查表常量。 */
    0xbe, /* 提供查表常量。 */
    0xe0, /* 提供查表常量。 */
    0x02, /* 提供查表常量。 */
    0x5c, /* 提供查表常量。 */
    0xdf, /* 提供查表常量。 */
    0x81, /* 提供查表常量。 */
    0x63, /* 提供查表常量。 */
    0x3d, /* 提供查表常量。 */
    0x7c, /* 提供查表常量。 */
    0x22, /* 提供查表常量。 */
    0xc0, /* 提供查表常量。 */
    0x9e, /* 提供查表常量。 */
    0x1d, /* 提供查表常量。 */
    0x43, /* 提供查表常量。 */
    0xa1, /* 提供查表常量。 */
    0xff, /* 提供查表常量。 */
    0x46, /* 提供查表常量。 */
    0x18, /* 提供查表常量。 */
    0xfa, /* 提供查表常量。 */
    0xa4, /* 提供查表常量。 */
    0x27, /* 提供查表常量。 */
    0x79, /* 提供查表常量。 */
    0x9b, /* 提供查表常量。 */
    0xc5, /* 提供查表常量。 */
    0x84, /* 提供查表常量。 */
    0xda, /* 提供查表常量。 */
    0x38, /* 提供查表常量。 */
    0x66, /* 提供查表常量。 */
    0xe5, /* 提供查表常量。 */
    0xbb, /* 提供查表常量。 */
    0x59, /* 提供查表常量。 */
    0x07, /* 提供查表常量。 */
    0xdb, /* 提供查表常量。 */
    0x85, /* 提供查表常量。 */
    0x67, /* 提供查表常量。 */
    0x39, /* 提供查表常量。 */
    0xba, /* 提供查表常量。 */
    0xe4, /* 提供查表常量。 */
    0x06, /* 提供查表常量。 */
    0x58, /* 提供查表常量。 */
    0x19, /* 提供查表常量。 */
    0x47, /* 提供查表常量。 */
    0xa5, /* 提供查表常量。 */
    0xfb, /* 提供查表常量。 */
    0x78, /* 提供查表常量。 */
    0x26, /* 提供查表常量。 */
    0xc4, /* 提供查表常量。 */
    0x9a, /* 提供查表常量。 */
    0x65, /* 提供查表常量。 */
    0x3b, /* 提供查表常量。 */
    0xd9, /* 提供查表常量。 */
    0x87, /* 提供查表常量。 */
    0x04, /* 提供查表常量。 */
    0x5a, /* 提供查表常量。 */
    0xb8, /* 提供查表常量。 */
    0xe6, /* 提供查表常量。 */
    0xa7, /* 提供查表常量。 */
    0xf9, /* 提供查表常量。 */
    0x1b, /* 提供查表常量。 */
    0x45, /* 提供查表常量。 */
    0xc6, /* 提供查表常量。 */
    0x98, /* 提供查表常量。 */
    0x7a, /* 提供查表常量。 */
    0x24, /* 提供查表常量。 */
    0xf8, /* 提供查表常量。 */
    0xa6, /* 提供查表常量。 */
    0x44, /* 提供查表常量。 */
    0x1a, /* 提供查表常量。 */
    0x99, /* 提供查表常量。 */
    0xc7, /* 提供查表常量。 */
    0x25, /* 提供查表常量。 */
    0x7b, /* 提供查表常量。 */
    0x3a, /* 提供查表常量。 */
    0x64, /* 提供查表常量。 */
    0x86, /* 提供查表常量。 */
    0xd8, /* 提供查表常量。 */
    0x5b, /* 提供查表常量。 */
    0x05, /* 提供查表常量。 */
    0xe7, /* 提供查表常量。 */
    0xb9, /* 提供查表常量。 */
    0x8c, /* 提供查表常量。 */
    0xd2, /* 提供查表常量。 */
    0x30, /* 提供查表常量。 */
    0x6e, /* 提供查表常量。 */
    0xed, /* 提供查表常量。 */
    0xb3, /* 提供查表常量。 */
    0x51, /* 提供查表常量。 */
    0x0f, /* 提供查表常量。 */
    0x4e, /* 提供查表常量。 */
    0x10, /* 提供查表常量。 */
    0xf2, /* 提供查表常量。 */
    0xac, /* 提供查表常量。 */
    0x2f, /* 提供查表常量。 */
    0x71, /* 提供查表常量。 */
    0x93, /* 提供查表常量。 */
    0xcd, /* 提供查表常量。 */
    0x11, /* 提供查表常量。 */
    0x4f, /* 提供查表常量。 */
    0xad, /* 提供查表常量。 */
    0xf3, /* 提供查表常量。 */
    0x70, /* 提供查表常量。 */
    0x2e, /* 提供查表常量。 */
    0xcc, /* 提供查表常量。 */
    0x92, /* 提供查表常量。 */
    0xd3, /* 提供查表常量。 */
    0x8d, /* 提供查表常量。 */
    0x6f, /* 提供查表常量。 */
    0x31, /* 提供查表常量。 */
    0xb2, /* 提供查表常量。 */
    0xec, /* 提供查表常量。 */
    0x0e, /* 提供查表常量。 */
    0x50, /* 提供查表常量。 */
    0xaf, /* 提供查表常量。 */
    0xf1, /* 提供查表常量。 */
    0x13, /* 提供查表常量。 */
    0x4d, /* 提供查表常量。 */
    0xce, /* 提供查表常量。 */
    0x90, /* 提供查表常量。 */
    0x72, /* 提供查表常量。 */
    0x2c, /* 提供查表常量。 */
    0x6d, /* 提供查表常量。 */
    0x33, /* 提供查表常量。 */
    0xd1, /* 提供查表常量。 */
    0x8f, /* 提供查表常量。 */
    0x0c, /* 提供查表常量。 */
    0x52, /* 提供查表常量。 */
    0xb0, /* 提供查表常量。 */
    0xee, /* 提供查表常量。 */
    0x32, /* 提供查表常量。 */
    0x6c, /* 提供查表常量。 */
    0x8e, /* 提供查表常量。 */
    0xd0, /* 提供查表常量。 */
    0x53, /* 提供查表常量。 */
    0x0d, /* 提供查表常量。 */
    0xef, /* 提供查表常量。 */
    0xb1, /* 提供查表常量。 */
    0xf0, /* 提供查表常量。 */
    0xae, /* 提供查表常量。 */
    0x4c, /* 提供查表常量。 */
    0x12, /* 提供查表常量。 */
    0x91, /* 提供查表常量。 */
    0xcf, /* 提供查表常量。 */
    0x2d, /* 提供查表常量。 */
    0x73, /* 提供查表常量。 */
    0xca, /* 提供查表常量。 */
    0x94, /* 提供查表常量。 */
    0x76, /* 提供查表常量。 */
    0x28, /* 提供查表常量。 */
    0xab, /* 提供查表常量。 */
    0xf5, /* 提供查表常量。 */
    0x17, /* 提供查表常量。 */
    0x49, /* 提供查表常量。 */
    0x08, /* 提供查表常量。 */
    0x56, /* 提供查表常量。 */
    0xb4, /* 提供查表常量。 */
    0xea, /* 提供查表常量。 */
    0x69, /* 提供查表常量。 */
    0x37, /* 提供查表常量。 */
    0xd5, /* 提供查表常量。 */
    0x8b, /* 提供查表常量。 */
    0x57, /* 提供查表常量。 */
    0x09, /* 提供查表常量。 */
    0xeb, /* 提供查表常量。 */
    0xb5, /* 提供查表常量。 */
    0x36, /* 提供查表常量。 */
    0x68, /* 提供查表常量。 */
    0x8a, /* 提供查表常量。 */
    0xd4, /* 提供查表常量。 */
    0x95, /* 提供查表常量。 */
    0xcb, /* 提供查表常量。 */
    0x29, /* 提供查表常量。 */
    0x77, /* 提供查表常量。 */
    0xf4, /* 提供查表常量。 */
    0xaa, /* 提供查表常量。 */
    0x48, /* 提供查表常量。 */
    0x16, /* 提供查表常量。 */
    0xe9, /* 提供查表常量。 */
    0xb7, /* 提供查表常量。 */
    0x55, /* 提供查表常量。 */
    0x0b, /* 提供查表常量。 */
    0x88, /* 提供查表常量。 */
    0xd6, /* 提供查表常量。 */
    0x34, /* 提供查表常量。 */
    0x6a, /* 提供查表常量。 */
    0x2b, /* 提供查表常量。 */
    0x75, /* 提供查表常量。 */
    0x97, /* 提供查表常量。 */
    0xc9, /* 提供查表常量。 */
    0x4a, /* 提供查表常量。 */
    0x14, /* 提供查表常量。 */
    0xf6, /* 提供查表常量。 */
    0xa8, /* 提供查表常量。 */
    0x74, /* 提供查表常量。 */
    0x2a, /* 提供查表常量。 */
    0xc8, /* 提供查表常量。 */
    0x96, /* 提供查表常量。 */
    0x15, /* 提供查表常量。 */
    0x4b, /* 提供查表常量。 */
    0xa9, /* 提供查表常量。 */
    0xf7, /* 提供查表常量。 */
    0xb6, /* 提供查表常量。 */
    0xe8, /* 提供查表常量。 */
    0x0a, /* 提供查表常量。 */
    0x54, /* 提供查表常量。 */
    0xd7, /* 提供查表常量。 */
    0x89, /* 提供查表常量。 */
    0x6b, /* 提供查表常量。 */
    0x35, /* 提供查表常量。 */
};

uint16_t CRC_INIT = 0xffff; /* 初始化 CRC_INIT。 */
const uint16_t wCRC_Table[256] = { /* 初始化 wCRC_Table。 */
    0x0000, 0x1189, 0x2312, 0x329b, 0x4624, 0x57ad, 0x6536, 0x74bf, /* 继续配置下一项。 */
    0x8c48, 0x9dc1, 0xaf5a, 0xbed3, 0xca6c, 0xdbe5, 0xe97e, 0xf8f7, /* 继续配置下一项。 */
    0x1081, 0x0108, 0x3393, 0x221a, 0x56a5, 0x472c, 0x75b7, 0x643e, /* 继续配置下一项。 */
    0x9cc9, 0x8d40, 0xbfdb, 0xae52, 0xdaed, 0xcb64, 0xf9ff, 0xe876, /* 继续配置下一项。 */
    0x2102, 0x308b, 0x0210, 0x1399, 0x6726, 0x76af, 0x4434, 0x55bd, /* 继续配置下一项。 */
    0xad4a, 0xbcc3, 0x8e58, 0x9fd1, 0xeb6e, 0xfae7, 0xc87c, 0xd9f5, /* 继续配置下一项。 */
    0x3183, 0x200a, 0x1291, 0x0318, 0x77a7, 0x662e, 0x54b5, 0x453c, /* 继续配置下一项。 */
    0xbdcb, 0xac42, 0x9ed9, 0x8f50, 0xfbef, 0xea66, 0xd8fd, 0xc974, /* 继续配置下一项。 */
    0x4204, 0x538d, 0x6116, 0x709f, 0x0420, 0x15a9, 0x2732, 0x36bb, /* 继续配置下一项。 */
    0xce4c, 0xdfc5, 0xed5e, 0xfcd7, 0x8868, 0x99e1, 0xab7a, 0xbaf3, /* 继续配置下一项。 */
    0x5285, 0x430c, 0x7197, 0x601e, 0x14a1, 0x0528, 0x37b3, 0x263a, /* 继续配置下一项。 */
    0xdecd, 0xcf44, 0xfddf, 0xec56, 0x98e9, 0x8960, 0xbbfb, 0xaa72, /* 继续配置下一项。 */
    0x6306, 0x728f, 0x4014, 0x519d, 0x2522, 0x34ab, 0x0630, 0x17b9, /* 继续配置下一项。 */
    0xef4e, 0xfec7, 0xcc5c, 0xddd5, 0xa96a, 0xb8e3, 0x8a78, 0x9bf1, /* 继续配置下一项。 */
    0x7387, 0x620e, 0x5095, 0x411c, 0x35a3, 0x242a, 0x16b1, 0x0738, /* 继续配置下一项。 */
    0xffcf, 0xee46, 0xdcdd, 0xcd54, 0xb9eb, 0xa862, 0x9af9, 0x8b70, /* 继续配置下一项。 */
    0x8408, 0x9581, 0xa71a, 0xb693, 0xc22c, 0xd3a5, 0xe13e, 0xf0b7, /* 继续配置下一项。 */
    0x0840, 0x19c9, 0x2b52, 0x3adb, 0x4e64, 0x5fed, 0x6d76, 0x7cff, /* 继续配置下一项。 */
    0x9489, 0x8500, 0xb79b, 0xa612, 0xd2ad, 0xc324, 0xf1bf, 0xe036, /* 继续配置下一项。 */
    0x18c1, 0x0948, 0x3bd3, 0x2a5a, 0x5ee5, 0x4f6c, 0x7df7, 0x6c7e, /* 继续配置下一项。 */
    0xa50a, 0xb483, 0x8618, 0x9791, 0xe32e, 0xf2a7, 0xc03c, 0xd1b5, /* 继续配置下一项。 */
    0x2942, 0x38cb, 0x0a50, 0x1bd9, 0x6f66, 0x7eef, 0x4c74, 0x5dfd, /* 继续配置下一项。 */
    0xb58b, 0xa402, 0x9699, 0x8710, 0xf3af, 0xe226, 0xd0bd, 0xc134, /* 继续配置下一项。 */
    0x39c3, 0x284a, 0x1ad1, 0x0b58, 0x7fe7, 0x6e6e, 0x5cf5, 0x4d7c, /* 继续配置下一项。 */
    0xc60c, 0xd785, 0xe51e, 0xf497, 0x8028, 0x91a1, 0xa33a, 0xb2b3, /* 继续配置下一项。 */
    0x4a44, 0x5bcd, 0x6956, 0x78df, 0x0c60, 0x1de9, 0x2f72, 0x3efb, /* 继续配置下一项。 */
    0xd68d, 0xc704, 0xf59f, 0xe416, 0x90a9, 0x8120, 0xb3bb, 0xa232, /* 继续配置下一项。 */
    0x5ac5, 0x4b4c, 0x79d7, 0x685e, 0x1ce1, 0x0d68, 0x3ff3, 0x2e7a, /* 继续配置下一项。 */
    0xe70e, 0xf687, 0xc41c, 0xd595, 0xa12a, 0xb0a3, 0x8238, 0x93b1, /* 继续配置下一项。 */
    0x6b46, 0x7acf, 0x4854, 0x59dd, 0x2d62, 0x3ceb, 0x0e70, 0x1ff9, /* 继续配置下一项。 */
    0xf78f, 0xe606, 0xd49d, 0xc514, 0xb1ab, 0xa022, 0x92b9, 0x8330, /* 继续配置下一项。 */
    0x7bc7, 0x6a4e, 0x58d5, 0x495c, 0x3de3, 0x2c6a, 0x1ef1, 0x0f78}; /* 完成本行操作。 */

uint8_t Get_CRC8_Check_Sum(uint8_t *pchMessage, /* 传入下一项参数或数据。 */
                           uint32_t dwLength, /* 传入下一项参数或数据。 */
                           uint8_t ucCRC8) /* 继续当前语句。 */
{
    uint8_t ucIndex; /* 保存 ucIndex。 */
    while (dwLength--) /* 条件满足时继续执行。 */
    {
        ucIndex = ucCRC8 ^ (*pchMessage++); /* 更新 ucIndex。 */
        ucCRC8 = CRC8_TAB[ucIndex]; /* 更新 ucCRC8。 */
    }
    return (ucCRC8); /* 返回当前计算结果。 */
}

/*
 ** Descriptions: CRC8 Verify function
 ** Input: Data to Verify,Stream length = Data + checksum
 ** Output: True or False (CRC Verify Result)
 */
uint8_t Verify_CRC8_Check_Sum(uint8_t *pchMessage, uint32_t dwLength) /* 实现 Verify_CRC8_Check_Sum。 */
{
    uint8_t ucExpected = 0; /* 初始化 ucExpected。 */
    if ((pchMessage == 0) || (dwLength <= 2)) /* 检查当前执行条件。 */
        return 0; /* 返回状态值 0。 */
    ucExpected = Get_CRC8_Check_Sum(pchMessage, dwLength - 1, CRC8_INIT); /* 更新 ucExpected。 */
    return (ucExpected == pchMessage[dwLength - 1]); /* 返回当前计算结果。 */
}

/*
 ** Descriptions: append CRC8 to the end of data
 ** Input: Data to CRC and append,Stream length = Data + checksum
 ** Output: True or False (CRC Verify Result)
 */
void Append_CRC8_Check_Sum(uint8_t *pchMessage, uint32_t dwLength) /* 实现 Append_CRC8_Check_Sum。 */
{
    uint8_t ucCRC = 0; /* 初始化 ucCRC。 */
    if ((pchMessage == 0) || (dwLength <= 2)) /* 检查当前执行条件。 */
        return; /* 结束当前函数。 */
    ucCRC = Get_CRC8_Check_Sum((uint8_t *)pchMessage, dwLength - 1, CRC8_INIT); /* 更新 ucCRC。 */
    pchMessage[dwLength - 1] = ucCRC; /* 更新 pchMessage。 */
}

/*
 ** Descriptions: CRC16 checksum function
 ** Input: Data to check,Stream length, initialized checksum
 ** Output: CRC checksum
 */
uint16_t Get_CRC16_Check_Sum(uint8_t *pchMessage, /* 传入下一项参数或数据。 */
                             uint32_t dwLength, /* 传入下一项参数或数据。 */
                             uint16_t wCRC) /* 继续当前语句。 */
{
    uint8_t chData; /* 保存 chData。 */
    if (pchMessage == NULL) /* 检查当前执行条件。 */
    {
        return 0xFFFF; /* 返回当前计算结果。 */
    }
    while (dwLength--) /* 条件满足时继续执行。 */
    {
        chData = *pchMessage++; /* 更新 chData。 */
        (wCRC) = ((uint16_t)(wCRC) >> 8) ^ wCRC_Table[((uint16_t)(wCRC) ^ (uint16_t)(chData)) & 0x00ff]; /* 更新 wCRC。 */
    }
    return wCRC; /* 返回当前计算结果。 */
}

/*
 ** Descriptions: CRC16 Verify function
 ** Input: Data to Verify,Stream length = Data + checksum
 ** Output: True or False (CRC Verify Result)
 */
uint32_t Verify_CRC16_Check_Sum(uint8_t *pchMessage, uint32_t dwLength) /* 实现 Verify_CRC16_Check_Sum。 */
{
    uint16_t wExpected = 0; /* 初始化 wExpected。 */
    if ((pchMessage == NULL) || (dwLength <= 2)) /* 检查当前执行条件。 */
    {
        return 0; /* 返回状态值 0。 */
    }
    wExpected = Get_CRC16_Check_Sum(pchMessage, dwLength - 2, CRC_INIT); /* 更新 wExpected。 */
    return ((wExpected & 0xff) == pchMessage[dwLength - 2] && ((wExpected >> 8) & 0xff) == pchMessage[dwLength - 1]); /* 返回当前计算结果。 */
}

/*
 ** Descriptions: append CRC16 to the end of data
 ** Input: Data to CRC and append,Stream length = Data + checksum
 ** Output: True or False (CRC Verify Result)
 */
void Append_CRC16_Check_Sum(uint8_t *pchMessage, uint32_t dwLength) /* 实现 Append_CRC16_Check_Sum。 */
{
    uint16_t wCRC = 0; /* 初始化 wCRC。 */
    if ((pchMessage == NULL) || (dwLength <= 2)) /* 检查当前执行条件。 */
    {
        return; /* 结束当前函数。 */
    }
    wCRC = Get_CRC16_Check_Sum((uint8_t *)pchMessage, dwLength - 2, CRC_INIT); /* 更新 wCRC。 */
    pchMessage[dwLength - 2] = (uint8_t)(wCRC & 0x00ff); /* 更新 pchMessage。 */
    pchMessage[dwLength - 1] = (uint8_t)((wCRC >> 8) & 0x00ff); /* 更新 pchMessage。 */
}
