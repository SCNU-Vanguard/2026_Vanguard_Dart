/*********************************************
 * 文件名：IA6B.c
 * 作用：接收FS-I6的数据
 * 创建时间：2025-12-24
 * 创建人：bale
 * notice：有16位校验码，通信速率115200（8N1）
 * //TODO：将数据读取到这里并返回位移后的数据，同时解算一定的数据并存放好
 ********************************************/
#include "IA6B.h"

int8_t Channel[13] = {0};
int16_t RawChannel[13] = {0};

void IA6B_Init(void)
{
    // 每次上电其实不用初始化什么特别的东西
}

void IA6B_HandleData2Channel(uint8_t *data)
{
    if (data == NULL)
    {
        return;
    }

    // 通道数据（每通道2字节，小端）
    for (uint8_t i = 0; i < 13; i++)
    {
        RawChannel[i] = (int16_t)(data[2 * i] | (data[2 * i + 1] << 8));
    }

    // 4个摇杆方向离散化（-1/0/1）
    for (uint8_t i = 0; i < 4; i++)
    {
        if ((RawChannel[i] >= 1400) && (RawChannel[i] <= 1600))
        {
            Channel[i] = 0;
        }
        else if (RawChannel[i] < 1400)
        {
            Channel[i] = -1;
        }
        else
        {
            Channel[i] = 1;
        }
    }

    // SWB 两段（1=默认位，0=调试位）
    if (RawChannel[4] >= 900 && RawChannel[4] <= 2100)
    {
        Channel[4] = (RawChannel[4] < 1500) ? 1 : 0;
    }

    // SWC 三段（1/0/-1）
    if (RawChannel[5] >= 900 && RawChannel[5] <= 2100)
    {
        if (RawChannel[5] < 1250)
        {
            Channel[5] = 1;
        }
        else if (RawChannel[5] < 1750)
        {
            Channel[5] = 0;
        }
        else
        {
            Channel[5] = -1;
        }
    }
}

int16_t IA6B_ReadChannel(uint8_t ChannelNum)
{
    if (ChannelNum < 1 || ChannelNum > 13)
    {
        return 0;
    }

    return Channel[ChannelNum - 1];
}

bool IA6B_ProcessIbusPacket(BSP_UART_NUM_e uart_num)
{
    IbusPacket_t packet;
    if (UART_GetIbusPacket(uart_num, &packet))
    {
        // 跳过帧头 data[0]=0x20, data[1]=0x40，后续28字节为通道区
        IA6B_HandleData2Channel(&packet.data[2]);
        return true;
    }

    return false;
}
