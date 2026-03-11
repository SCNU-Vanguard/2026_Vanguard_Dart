/*********************************************
 * 文件名：IA6B.c
 * 作用：接收FS-I6的数据
 * 创建时间：2025-12-24
 * 创建人：bale
 * notice：有16位校验码，通信速率115200（8N1）
 * //TODO：将数据读取到这里并返回位移后的数据，同时解算一定的数据并存放好
 * NOTE:
 * 右边摇杆
 *
 * （左右）
 * RawChannel 0 : 属于 1000左 -> 1500中 -> 2000右
 * （上下）
 * RawChannel 1 : 属于 2000上 -> 1500中 -> 1000下
 *
 *  左边摇杆
 *
 * （上下）
 * RawChannel 2 : 属于 2000上 -> 1500中 -> 1000下
 * （左右）
 * RawChannel 3 : 属于 1000左 -> 1500中 -> 2000右
 *
 * SWB
 * RawChannel 4 : 属于1对应1000，2对应2000
 *
 * RAWChannel 5 : 属于1对应1000，2对应1500，3对应2000
 ********************************************/
#include "config.h"
#include "IA6B.h"

int8_t Channel[13] = {0};
int16_t RawChannel[13] = {0};
void IA6B_HandleData2Channel(uint8_t *data)
{
    // 传进来的channel数据是符合对应信道格式的,除去帧头和校验帧尾一共28字节
    // 前Channel1-14获取
    for (uint8_t i = 0; i < 14; i++)
    {
        // 两字节循环,直到26
        RawChannel[i] = data[2 * i] | (data[2 * i + 1] << 8);
    }
    for (uint8_t temp = 0; temp < 4; temp++)
    {
        // 一般中间为0
        if ((RawChannel[temp] >= 1400) && (RawChannel[temp] <= 1600))
        {
            Channel[temp] = 0;
        }
        // 下和左为-1
        else if (RawChannel[temp] < 1400)
        {
            Channel[temp] = -1;
        }
        else
        {
            Channel[temp] = 1;
        }
    }

    // 对应SWB,1为默认状态为真,2为手动调节为0
    if (RawChannel[4] == 1000)
    {
        Channel[4] = 1;
    }
    else
    {
        Channel[4] = 0;
    }

    if (RawChannel[5] == 1000)
    {
        Channel[5] = 1;
    }
    else if (RawChannel[5] == 1500)
    {
        Channel[5] = 0;
    }
    else
    {
        Channel[5] = -1;
    }
}

// 初始化队列接受任务,同时使用DMA加缓冲区接收,但是这个固定很麻烦
int16_t IA6B_ReadChannel(uint8_t ChannelNum)
{
    return Channel[ChannelNum - 1];
}

/**
 * @brief 处理IBUS数据包并解析通道
 * @param uart_num UART编号 (通常为BSP_UART6)
 * @return true-成功获取并解析, false-无数据包
 * @note 在主循环或任务中周期性调用
 */
bool IA6B_ProcessIbusPacket(BSP_UART_NUM_e uart_num)
{
    IbusPacket_t packet;
    if (UART_GetIbusPacket(uart_num, &packet))
    {
        // 跳过帧头(data[0]=0x20, data[1]=0x40)，传入28字节通道数据
        IA6B_HandleData2Channel(&packet.data[2]);
        return true;
    }
    return false;
}
