#include "rm_referee_protocol.h"
#include "referee.h"
#include "CRC.h"
#include "stdbool.h"
#include "usart.h"
#include <string.h>

#define RE_RX_BUFFER_SIZE 511u // 裁判系统接收缓冲区大小

static referee_info_t referee_info; // 裁判系统数据
static uint32_t g_game_state_update_seq = 0U; /* 初始化 g_game_state_update_seq。 */
static uint32_t g_dart_info_update_seq = 0U; /* 初始化 g_dart_info_update_seq。 */
static uint32_t g_dart_launch_status_update_seq = 0U; /* 初始化 g_dart_launch_status_update_seq。 */

uint16_t RefereeDebug_CmdID = 0U; /* 初始化 RefereeDebug_CmdID。 */
uint32_t RefereeDebug_GameStateSeq = 0U; /* 初始化 RefereeDebug_GameStateSeq。 */
uint32_t RefereeDebug_DartInfoSeq = 0U; /* 初始化 RefereeDebug_DartInfoSeq。 */
uint32_t RefereeDebug_DartLaunchSeq = 0U; /* 初始化 RefereeDebug_DartLaunchSeq。 */
uint16_t RefereeDebug_StageRemainTime = 0U; /* 初始化 RefereeDebug_StageRemainTime。 */
uint8_t RefereeDebug_DartRemainingTime = 0U; /* 初始化 RefereeDebug_DartRemainingTime。 */
uint8_t RefereeDebug_DartOpeningStatus = 0U; /* 初始化 RefereeDebug_DartOpeningStatus。 */
uint16_t RefereeDebug_TargetChangeTime = 0U; /* 初始化 RefereeDebug_TargetChangeTime。 */
uint16_t RefereeDebug_LatestLaunchCmdTime = 0U; /* 初始化 RefereeDebug_LatestLaunchCmdTime。 */
ext_dart_launch_status_t RefereeDebug_DartLaunchStatus = {0U}; /* 初始化 RefereeDebug_DartLaunchStatus。 */
uint8_t RefereeDebug_DartLaunchRaw[LEN_dart_launch_status] = {0U}; /* 初始化 RefereeDebug_DartLaunchRaw。 */
uint8_t RefereeDebug_SelectedTarget = 0U; /* 初始化 RefereeDebug_SelectedTarget。 */
uint32_t RefereeDebug_LastParseTick = 0U; /* 初始化 RefereeDebug_LastParseTick。 */
uint32_t RefereeDebug_PollCount = 0U; /* 初始化 RefereeDebug_PollCount。 */
uint32_t RefereeDebug_PacketPopCount = 0U; /* 初始化 RefereeDebug_PacketPopCount。 */
uint32_t RefereeDebug_PacketValidCount = 0U; /* 初始化 RefereeDebug_PacketValidCount。 */
uint32_t RefereeDebug_JudgeFrameOkCount = 0U; /* 初始化 RefereeDebug_JudgeFrameOkCount。 */
uint32_t RefereeDebug_JudgeHeaderCrcFailCount = 0U; /* 初始化 RefereeDebug_JudgeHeaderCrcFailCount。 */
uint32_t RefereeDebug_JudgeFrameCrcFailCount = 0U; /* 初始化 RefereeDebug_JudgeFrameCrcFailCount。 */
uint32_t RefereeDebug_JudgeInvalidSofCount = 0U; /* 初始化 RefereeDebug_JudgeInvalidSofCount。 */
uint32_t RefereeDebug_JudgeIncompleteCount = 0U; /* 初始化 RefereeDebug_JudgeIncompleteCount。 */
uint16_t RefereeDebug_LastPacketLen = 0U; /* 初始化 RefereeDebug_LastPacketLen。 */
uint16_t RefereeDebug_LastJudgeLen = 0U; /* 初始化 RefereeDebug_LastJudgeLen。 */

/**
 * @brief  读取裁判数据,中断中读取保证速度
 * @param  buff: 读取到的裁判系统原始数据
 * @param  buff_len: 当前缓冲中的总长度
 * @attention  在此判断帧头和CRC校验,无误再写入数据
 */
static void JudgeReadData(const uint8_t *buff, uint16_t buff_len) /* 实现 JudgeReadData。 */
{
    uint16_t offset = 0; /* 初始化 offset。 */

    if (buff == NULL) // 空数据包，则不作任何处理
        return; /* 结束当前函数。 */

    while ((buff_len - offset) >= (LEN_HEADER + LEN_CMDID + LEN_TAIL)) /* 条件满足时继续执行。 */
    {
        const uint8_t *frame = buff + offset; /* 初始化 frame。 */
        uint16_t data_len; /* 保存 data_len。 */
        uint16_t judge_length; /* 保存 judge_length。 */

        if (frame[SOF] != REFEREE_SOF) /* 检查当前执行条件。 */
        {
            RefereeDebug_JudgeInvalidSofCount++; /* 递增 RefereeDebug_JudgeInvalidSofCount。 */
            offset++; /* 递增 offset。 */
            continue; /* 跳过本轮剩余处理。 */
        }

        if (!Verify_CRC8_Check_Sum((uint8_t *)frame, LEN_HEADER)) /* 检查当前执行条件。 */
        {
            RefereeDebug_JudgeHeaderCrcFailCount++; /* 递增 RefereeDebug_JudgeHeaderCrcFailCount。 */
            offset++; /* 递增 offset。 */
            continue; /* 跳过本轮剩余处理。 */
        }

        data_len = (uint16_t)frame[DATA_LENGTH] | ((uint16_t)frame[DATA_LENGTH + 1] << 8); /* 更新 data_len。 */
        judge_length = (uint16_t)(data_len + LEN_HEADER + LEN_CMDID + LEN_TAIL); /* 更新 judge_length。 */
        RefereeDebug_LastJudgeLen = judge_length; /* 定义 RefereeDebug_LastJudgeLen 枚举项。 */
        if (judge_length > (buff_len - offset) || judge_length < (LEN_HEADER + LEN_CMDID + LEN_TAIL)) /* 检查当前执行条件。 */
        {
            RefereeDebug_JudgeIncompleteCount++; /* 递增 RefereeDebug_JudgeIncompleteCount。 */
            break; /* 结束当前循环或分支。 */
        }

        if (!Verify_CRC16_Check_Sum((uint8_t *)frame, judge_length)) /* 检查当前执行条件。 */
        {
            RefereeDebug_JudgeFrameCrcFailCount++; /* 递增 RefereeDebug_JudgeFrameCrcFailCount。 */
            offset++; /* 递增 offset。 */
            continue; /* 跳过本轮剩余处理。 */
        }

        memcpy(&referee_info.FrameHeader, frame, LEN_HEADER); /* 调用 memcpy。 */
        referee_info.CmdID = (uint16_t)frame[5] | ((uint16_t)frame[6] << 8); /* 更新 CmdID。 */
        RefereeDebug_CmdID = referee_info.CmdID; /* 定义 RefereeDebug_CmdID 枚举项。 */
        RefereeDebug_LastParseTick = HAL_GetTick(); /* 定义 RefereeDebug_LastParseTick 枚举项。 */
        RefereeDebug_JudgeFrameOkCount++; /* 递增 RefereeDebug_JudgeFrameOkCount。 */
        switch (referee_info.CmdID) /* 按当前状态选择处理分支。 */
        {
        case ID_game_state: /* 处理 ID_game_state 分支。 */
            memcpy(&referee_info.GameState, (frame + DATA_Offset), LEN_game_state); /* 调用 memcpy。 */
            g_game_state_update_seq++; /* 递增 g_game_state_update_seq。 */
            RefereeDebug_GameStateSeq = g_game_state_update_seq; /* 定义 RefereeDebug_GameStateSeq 枚举项。 */
            RefereeDebug_StageRemainTime = referee_info.GameState.stage_remain_time; /* 定义 RefereeDebug_StageRemainTime 枚举项。 */
            break; /* 结束当前循环或分支。 */
        case ID_game_result: /* 处理 ID_game_result 分支。 */
            memcpy(&referee_info.GameResult, (frame + DATA_Offset), LEN_game_result); /* 调用 memcpy。 */
            break; /* 结束当前循环或分支。 */
        case ID_game_robot_HP: /* 处理 ID_game_robot_HP 分支。 */
            memcpy(&referee_info.GameRobotHP, (frame + DATA_Offset), LEN_game_robot_HP); /* 调用 memcpy。 */
            break; /* 结束当前循环或分支。 */
        case ID_event_data: /* 处理 ID_event_data 分支。 */
            memcpy(&referee_info.EventData, (frame + DATA_Offset), LEN_event_data); /* 调用 memcpy。 */
            break; /* 结束当前循环或分支。 */
        case ID_referee_warning: /* 处理 ID_referee_warning 分支。 */
            memcpy(&referee_info.RefereeWarning, (frame + DATA_Offset), LEN_referee_warning); /* 调用 memcpy。 */
            break; /* 结束当前循环或分支。 */
        case ID_dart_info: /* 处理 ID_dart_info 分支。 */
            memcpy(&referee_info.DartInfo, (frame + DATA_Offset), LEN_dart_info); /* 调用 memcpy。 */
            g_dart_info_update_seq++; /* 递增 g_dart_info_update_seq。 */
            RefereeDebug_DartInfoSeq = g_dart_info_update_seq; /* 定义 RefereeDebug_DartInfoSeq 枚举项。 */
            RefereeDebug_DartRemainingTime = referee_info.DartInfo.dart_remaining_time; /* 定义 RefereeDebug_DartRemainingTime 枚举项。 */
            RefereeDebug_SelectedTarget = referee_info.DartInfo.referee_dart_info.dart_info_bia_bits.selected_target; /* 定义 RefereeDebug_SelectedTarget 枚举项。 */
            break; /* 结束当前循环或分支。 */
        case ID_robot_performance: /* 处理 ID_robot_performance 分支。 */
            memcpy(&referee_info.RobotPerformance, (frame + DATA_Offset), LEN_robot_performance); /* 调用 memcpy。 */
            break; /* 结束当前循环或分支。 */
        case ID_power_heat_data: /* 处理 ID_power_heat_data 分支。 */
            memcpy(&referee_info.PowerHeatData, (frame + DATA_Offset), LEN_power_heat_data); /* 调用 memcpy。 */
            break; /* 结束当前循环或分支。 */
        case ID_game_robot_pos: /* 处理 ID_game_robot_pos 分支。 */
            memcpy(&referee_info.GameRobotPos, (frame + DATA_Offset), LEN_game_robot_pos); /* 调用 memcpy。 */
            break; /* 结束当前循环或分支。 */
        case ID_buff_musk: /* 处理 ID_buff_musk 分支。 */
            memcpy(&referee_info.BuffMusk, (frame + DATA_Offset), LEN_buff_musk); /* 调用 memcpy。 */
            break; /* 结束当前循环或分支。 */
        case ID_robot_hurt: /* 处理 ID_robot_hurt 分支。 */
            memcpy(&referee_info.RobotHurt, (frame + DATA_Offset), LEN_robot_hurt); /* 调用 memcpy。 */
            break; /* 结束当前循环或分支。 */
        case ID_shoot_data: /* 处理 ID_shoot_data 分支。 */
            memcpy(&referee_info.ShootData, (frame + DATA_Offset), LEN_shoot_data); /* 调用 memcpy。 */
            break; /* 结束当前循环或分支。 */
        case ID_projectile_allowance: /* 处理 ID_projectile_allowance 分支。 */
            memcpy(&referee_info.ProjectileAllowance, (frame + DATA_Offset), LEN_projectile_allowance); /* 调用 memcpy。 */
            break; /* 结束当前循环或分支。 */
        case ID_rfid_status: /* 处理 ID_rfid_status 分支。 */
            memcpy(&referee_info.RFIDStatus, (frame + DATA_Offset), LEN_rfid_status); /* 调用 memcpy。 */
            break; /* 结束当前循环或分支。 */
        case ID_dart_launch_status: /* 处理 ID_dart_launch_status 分支。 */
            memcpy(&referee_info.DartLaunchStatus, (frame + DATA_Offset), LEN_dart_launch_status); /* 调用 memcpy。 */
            g_dart_launch_status_update_seq++; /* 递增 g_dart_launch_status_update_seq。 */
            RefereeDebug_DartLaunchSeq = g_dart_launch_status_update_seq; /* 定义 RefereeDebug_DartLaunchSeq 枚举项。 */
            memcpy(&RefereeDebug_DartLaunchStatus, &referee_info.DartLaunchStatus, sizeof(RefereeDebug_DartLaunchStatus)); /* 调用 memcpy。 */
            memcpy(RefereeDebug_DartLaunchRaw, (frame + DATA_Offset), sizeof(RefereeDebug_DartLaunchRaw)); /* 调用 memcpy。 */
            RefereeDebug_DartOpeningStatus = referee_info.DartLaunchStatus.dart_launch_opening_status; /* 定义 RefereeDebug_DartOpeningStatus 枚举项。 */
            RefereeDebug_TargetChangeTime = referee_info.DartLaunchStatus.target_change_time; /* 定义 RefereeDebug_TargetChangeTime 枚举项。 */
            RefereeDebug_LatestLaunchCmdTime = referee_info.DartLaunchStatus.latest_launch_cmd_time; /* 定义 RefereeDebug_LatestLaunchCmdTime 枚举项。 */
            break; /* 结束当前循环或分支。 */
        case ID_ground_robot_position: /* 处理 ID_ground_robot_position 分支。 */
            memcpy(&referee_info.GroundRobotPosition, (frame + DATA_Offset), LEN_ground_robot_position); /* 调用 memcpy。 */
            break; /* 结束当前循环或分支。 */
        case ID_radar_mark_data: /* 处理 ID_radar_mark_data 分支。 */
            memcpy(&referee_info.RadarMarkData, (frame + DATA_Offset), LEN_radar_mark_data); /* 调用 memcpy。 */
            break; /* 结束当前循环或分支。 */
        case ID_sentry_info: /* 处理 ID_sentry_info 分支。 */
            memcpy(&referee_info.SentryInfo, (frame + DATA_Offset), LEN_sentry_info); /* 调用 memcpy。 */
            break; /* 结束当前循环或分支。 */
        case ID_radar_info: /* 处理 ID_radar_info 分支。 */
            memcpy(&referee_info.RadarInfo, (frame + DATA_Offset), LEN_radar_info); /* 调用 memcpy。 */
            break; /* 结束当前循环或分支。 */
        case ID_student_interactive: /* 处理 ID_student_interactive 分支。 */
            memcpy(&referee_info.ReceiveData, (frame + DATA_Offset), LEN_receive_data); /* 调用 memcpy。 */
            break; /* 结束当前循环或分支。 */
        default: /* 处理默认分支。 */
            break; /* 结束当前循环或分支。 */
        }

        offset += judge_length; /* 更新 offset。 */
    }
}

referee_info_t *Referee_Register(UART_HandleTypeDef *referee_usart_handle) /* 实现 Referee_Register。 */
{
    if (referee_usart_handle != NULL && referee_usart_handle != &huart8) /* 检查当前执行条件。 */
    {
        return NULL; /* 返回当前计算结果。 */
    }

    memset(&referee_info, 0, sizeof(referee_info)); /* 调用 memset。 */
    g_game_state_update_seq = 0U; /* 更新 g_game_state_update_seq。 */
    g_dart_info_update_seq = 0U; /* 更新 g_dart_info_update_seq。 */
    g_dart_launch_status_update_seq = 0U; /* 更新 g_dart_launch_status_update_seq。 */
    RefereeDebug_CmdID = 0U; /* 定义 RefereeDebug_CmdID 枚举项。 */
    RefereeDebug_GameStateSeq = 0U; /* 定义 RefereeDebug_GameStateSeq 枚举项。 */
    RefereeDebug_DartInfoSeq = 0U; /* 定义 RefereeDebug_DartInfoSeq 枚举项。 */
    RefereeDebug_DartLaunchSeq = 0U; /* 定义 RefereeDebug_DartLaunchSeq 枚举项。 */
    RefereeDebug_StageRemainTime = 0U; /* 定义 RefereeDebug_StageRemainTime 枚举项。 */
    RefereeDebug_DartRemainingTime = 0U; /* 定义 RefereeDebug_DartRemainingTime 枚举项。 */
    RefereeDebug_DartOpeningStatus = 0U; /* 定义 RefereeDebug_DartOpeningStatus 枚举项。 */
    RefereeDebug_TargetChangeTime = 0U; /* 定义 RefereeDebug_TargetChangeTime 枚举项。 */
    RefereeDebug_LatestLaunchCmdTime = 0U; /* 定义 RefereeDebug_LatestLaunchCmdTime 枚举项。 */
    memset(&RefereeDebug_DartLaunchStatus, 0, sizeof(RefereeDebug_DartLaunchStatus)); /* 调用 memset。 */
    memset(RefereeDebug_DartLaunchRaw, 0, sizeof(RefereeDebug_DartLaunchRaw)); /* 调用 memset。 */
    RefereeDebug_SelectedTarget = 0U; /* 定义 RefereeDebug_SelectedTarget 枚举项。 */
    RefereeDebug_LastParseTick = 0U; /* 定义 RefereeDebug_LastParseTick 枚举项。 */
    RefereeDebug_PollCount = 0U; /* 定义 RefereeDebug_PollCount 枚举项。 */
    RefereeDebug_PacketPopCount = 0U; /* 定义 RefereeDebug_PacketPopCount 枚举项。 */
    RefereeDebug_PacketValidCount = 0U; /* 定义 RefereeDebug_PacketValidCount 枚举项。 */
    RefereeDebug_JudgeFrameOkCount = 0U; /* 定义 RefereeDebug_JudgeFrameOkCount 枚举项。 */
    RefereeDebug_JudgeHeaderCrcFailCount = 0U; /* 定义 RefereeDebug_JudgeHeaderCrcFailCount 枚举项。 */
    RefereeDebug_JudgeFrameCrcFailCount = 0U; /* 定义 RefereeDebug_JudgeFrameCrcFailCount 枚举项。 */
    RefereeDebug_JudgeInvalidSofCount = 0U; /* 定义 RefereeDebug_JudgeInvalidSofCount 枚举项。 */
    RefereeDebug_JudgeIncompleteCount = 0U; /* 定义 RefereeDebug_JudgeIncompleteCount 枚举项。 */
    RefereeDebug_LastPacketLen = 0U; /* 定义 RefereeDebug_LastPacketLen 枚举项。 */
    RefereeDebug_LastJudgeLen = 0U; /* 定义 RefereeDebug_LastJudgeLen 枚举项。 */
    referee_info.init_flag = 1U; /* 更新 init_flag。 */
    // UART8 在 BSP_UART_Init() 中已经切到裁判协议并启动接收，这里只清软件状态。
    UART_ClearRefereePacket(BSP_UART8); /* 调用 UART_ClearRefereePacket。 */
    return &referee_info; /* 返回当前计算结果。 */
}

bool Referee_Poll(void) /* 实现 Referee_Poll。 */
{
    RefereePacket_t packet; /* 保存 packet。 */
    bool parsed = false; /* 初始化 parsed。 */

    RefereeDebug_PollCount++; /* 递增 RefereeDebug_PollCount。 */
    while (UART_GetRefereePacket(BSP_UART8, &packet)) /* 条件满足时继续执行。 */
    {
        RefereeDebug_PacketPopCount++; /* 递增 RefereeDebug_PacketPopCount。 */
        RefereeDebug_LastPacketLen = packet.length; /* 定义 RefereeDebug_LastPacketLen 枚举项。 */
        if (packet.is_valid && packet.length > 0U) /* 检查当前执行条件。 */
        {
            RefereeDebug_PacketValidCount++; /* 递增 RefereeDebug_PacketValidCount。 */
            JudgeReadData(packet.data, packet.length); /* 调用 JudgeReadData。 */
            parsed = true; /* 更新 parsed。 */
        }
    }

    return parsed; /* 返回当前计算结果。 */
}

uint32_t Referee_GetCmdUpdateSeq(uint16_t cmd_id) /* 实现 Referee_GetCmdUpdateSeq。 */
{
    switch (cmd_id) /* 按当前状态选择处理分支。 */
    {
    case ID_game_state: /* 处理 ID_game_state 分支。 */
        return g_game_state_update_seq; /* 返回当前计算结果。 */
    case ID_dart_info: /* 处理 ID_dart_info 分支。 */
        return g_dart_info_update_seq; /* 返回当前计算结果。 */
    case ID_dart_launch_status: /* 处理 ID_dart_launch_status 分支。 */
        return g_dart_launch_status_update_seq; /* 返回当前计算结果。 */
    default: /* 处理默认分支。 */
        return 0U; /* 返回状态值 0U。 */
    }
}

const void *Referee_GetDataByCmdID(uint16_t cmd_id) /* 实现 Referee_GetDataByCmdID。 */
{
    switch (cmd_id) /* 按当前状态选择处理分支。 */
    {
    case ID_game_state: /* 处理 ID_game_state 分支。 */
        return &referee_info.GameState; /* 返回当前计算结果。 */
    case ID_game_result: /* 处理 ID_game_result 分支。 */
        return &referee_info.GameResult; /* 返回当前计算结果。 */
    case ID_game_robot_HP: /* 处理 ID_game_robot_HP 分支。 */
        return &referee_info.GameRobotHP; /* 返回当前计算结果。 */
    case ID_event_data: /* 处理 ID_event_data 分支。 */
        return &referee_info.EventData; /* 返回当前计算结果。 */
    case ID_referee_warning: /* 处理 ID_referee_warning 分支。 */
        return &referee_info.RefereeWarning; /* 返回当前计算结果。 */
    case ID_dart_info: /* 处理 ID_dart_info 分支。 */
        return &referee_info.DartInfo; /* 返回当前计算结果。 */
    case ID_robot_performance: /* 处理 ID_robot_performance 分支。 */
        return &referee_info.RobotPerformance; /* 返回当前计算结果。 */
    case ID_power_heat_data: /* 处理 ID_power_heat_data 分支。 */
        return &referee_info.PowerHeatData; /* 返回当前计算结果。 */
    case ID_game_robot_pos: /* 处理 ID_game_robot_pos 分支。 */
        return &referee_info.GameRobotPos; /* 返回当前计算结果。 */
    case ID_buff_musk: /* 处理 ID_buff_musk 分支。 */
        return &referee_info.BuffMusk; /* 返回当前计算结果。 */
    case ID_robot_hurt: /* 处理 ID_robot_hurt 分支。 */
        return &referee_info.RobotHurt; /* 返回当前计算结果。 */
    case ID_shoot_data: /* 处理 ID_shoot_data 分支。 */
        return &referee_info.ShootData; /* 返回当前计算结果。 */
    case ID_projectile_allowance: /* 处理 ID_projectile_allowance 分支。 */
        return &referee_info.ProjectileAllowance; /* 返回当前计算结果。 */
    case ID_rfid_status: /* 处理 ID_rfid_status 分支。 */
        return &referee_info.RFIDStatus; /* 返回当前计算结果。 */
    case ID_dart_launch_status: /* 处理 ID_dart_launch_status 分支。 */
        return &referee_info.DartLaunchStatus; /* 返回当前计算结果。 */
    case ID_ground_robot_position: /* 处理 ID_ground_robot_position 分支。 */
        return &referee_info.GroundRobotPosition; /* 返回当前计算结果。 */
    case ID_radar_mark_data: /* 处理 ID_radar_mark_data 分支。 */
        return &referee_info.RadarMarkData; /* 返回当前计算结果。 */
    case ID_sentry_info: /* 处理 ID_sentry_info 分支。 */
        return &referee_info.SentryInfo; /* 返回当前计算结果。 */
    case ID_radar_info: /* 处理 ID_radar_info 分支。 */
        return &referee_info.RadarInfo; /* 返回当前计算结果。 */
    case ID_student_interactive: /* 处理 ID_student_interactive 分支。 */
        return &referee_info.ReceiveData; /* 返回当前计算结果。 */
    default: /* 处理默认分支。 */
        return NULL; /* 返回当前计算结果。 */
    }
}

/**
 * @brief 裁判系统数据发送函数
 * @param
 */
void Referee_Send(uint8_t *send, uint16_t tx_len) /* 实现 Referee_Send。 */
{
    UART_Send(BSP_UART8, send, tx_len); /* 调用 UART_Send。 */
    osDelay(40); /* 调用 osDelay。 */
}
