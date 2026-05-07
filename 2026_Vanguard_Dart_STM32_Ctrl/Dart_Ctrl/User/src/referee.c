#include "rm_referee_protocol.h"
#include "referee.h"
#include "CRC.h"
#include "stdbool.h"
#include "usart.h"
#include <string.h>

#define RE_RX_BUFFER_SIZE 511u // 裁判系统接收缓冲区大小

static referee_info_t referee_info; // 裁判系统数据
static uint32_t g_game_state_update_seq = 0U;
static uint32_t g_dart_info_update_seq = 0U;
static uint32_t g_dart_launch_status_update_seq = 0U;

uint16_t RefereeDebug_CmdID = 0U;
uint32_t RefereeDebug_GameStateSeq = 0U;
uint32_t RefereeDebug_DartInfoSeq = 0U;
uint32_t RefereeDebug_DartLaunchSeq = 0U;
uint16_t RefereeDebug_StageRemainTime = 0U;
uint8_t RefereeDebug_DartRemainingTime = 0U;
uint8_t RefereeDebug_DartOpeningStatus = 0U;
uint16_t RefereeDebug_TargetChangeTime = 0U;
uint16_t RefereeDebug_LatestLaunchCmdTime = 0U;
ext_dart_launch_status_t RefereeDebug_DartLaunchStatus = {0U};
uint8_t RefereeDebug_DartLaunchRaw[LEN_dart_launch_status] = {0U};
uint8_t RefereeDebug_SelectedTarget = 0U;
uint32_t RefereeDebug_LastParseTick = 0U;
uint32_t RefereeDebug_PollCount = 0U;
uint32_t RefereeDebug_PacketPopCount = 0U;
uint32_t RefereeDebug_PacketValidCount = 0U;
uint32_t RefereeDebug_JudgeFrameOkCount = 0U;
uint32_t RefereeDebug_JudgeHeaderCrcFailCount = 0U;
uint32_t RefereeDebug_JudgeFrameCrcFailCount = 0U;
uint32_t RefereeDebug_JudgeInvalidSofCount = 0U;
uint32_t RefereeDebug_JudgeIncompleteCount = 0U;
uint16_t RefereeDebug_LastPacketLen = 0U;
uint16_t RefereeDebug_LastJudgeLen = 0U;

/**
 * @brief  读取裁判数据,中断中读取保证速度
 * @param  buff: 读取到的裁判系统原始数据
 * @param  buff_len: 当前缓冲中的总长度
 * @attention  在此判断帧头和CRC校验,无误再写入数据
 */
static void JudgeReadData(const uint8_t *buff, uint16_t buff_len)
{
    uint16_t offset = 0;

    if (buff == NULL) // 空数据包，则不作任何处理
        return;

    while ((buff_len - offset) >= (LEN_HEADER + LEN_CMDID + LEN_TAIL))
    {
        const uint8_t *frame = buff + offset;
        uint16_t data_len;
        uint16_t judge_length;

        if (frame[SOF] != REFEREE_SOF)
        {
            RefereeDebug_JudgeInvalidSofCount++;
            offset++;
            continue;
        }

        if (!Verify_CRC8_Check_Sum((uint8_t *)frame, LEN_HEADER))
        {
            RefereeDebug_JudgeHeaderCrcFailCount++;
            offset++;
            continue;
        }

        data_len = (uint16_t)frame[DATA_LENGTH] | ((uint16_t)frame[DATA_LENGTH + 1] << 8);
        judge_length = (uint16_t)(data_len + LEN_HEADER + LEN_CMDID + LEN_TAIL);
        RefereeDebug_LastJudgeLen = judge_length;
        if (judge_length > (buff_len - offset) || judge_length < (LEN_HEADER + LEN_CMDID + LEN_TAIL))
        {
            RefereeDebug_JudgeIncompleteCount++;
            break;
        }

        if (!Verify_CRC16_Check_Sum((uint8_t *)frame, judge_length))
        {
            RefereeDebug_JudgeFrameCrcFailCount++;
            offset++;
            continue;
        }

        memcpy(&referee_info.FrameHeader, frame, LEN_HEADER);
        referee_info.CmdID = (uint16_t)frame[5] | ((uint16_t)frame[6] << 8);
        RefereeDebug_CmdID = referee_info.CmdID;
        RefereeDebug_LastParseTick = HAL_GetTick();
        RefereeDebug_JudgeFrameOkCount++;
        switch (referee_info.CmdID)
        {
        case ID_game_state:
            memcpy(&referee_info.GameState, (frame + DATA_Offset), LEN_game_state);
            g_game_state_update_seq++;
            RefereeDebug_GameStateSeq = g_game_state_update_seq;
            RefereeDebug_StageRemainTime = referee_info.GameState.stage_remain_time;
            break;
        case ID_game_result:
            memcpy(&referee_info.GameResult, (frame + DATA_Offset), LEN_game_result);
            break;
        case ID_game_robot_HP:
            memcpy(&referee_info.GameRobotHP, (frame + DATA_Offset), LEN_game_robot_HP);
            break;
        case ID_event_data:
            memcpy(&referee_info.EventData, (frame + DATA_Offset), LEN_event_data);
            break;
        case ID_referee_warning:
            memcpy(&referee_info.RefereeWarning, (frame + DATA_Offset), LEN_referee_warning);
            break;
        case ID_dart_info:
            memcpy(&referee_info.DartInfo, (frame + DATA_Offset), LEN_dart_info);
            g_dart_info_update_seq++;
            RefereeDebug_DartInfoSeq = g_dart_info_update_seq;
            RefereeDebug_DartRemainingTime = referee_info.DartInfo.dart_remaining_time;
            RefereeDebug_SelectedTarget = referee_info.DartInfo.referee_dart_info.dart_info_bia_bits.selected_target;
            break;
        case ID_robot_performance:
            memcpy(&referee_info.RobotPerformance, (frame + DATA_Offset), LEN_robot_performance);
            break;
        case ID_power_heat_data:
            memcpy(&referee_info.PowerHeatData, (frame + DATA_Offset), LEN_power_heat_data);
            break;
        case ID_game_robot_pos:
            memcpy(&referee_info.GameRobotPos, (frame + DATA_Offset), LEN_game_robot_pos);
            break;
        case ID_buff_musk:
            memcpy(&referee_info.BuffMusk, (frame + DATA_Offset), LEN_buff_musk);
            break;
        case ID_robot_hurt:
            memcpy(&referee_info.RobotHurt, (frame + DATA_Offset), LEN_robot_hurt);
            break;
        case ID_shoot_data:
            memcpy(&referee_info.ShootData, (frame + DATA_Offset), LEN_shoot_data);
            break;
        case ID_projectile_allowance:
            memcpy(&referee_info.ProjectileAllowance, (frame + DATA_Offset), LEN_projectile_allowance);
            break;
        case ID_rfid_status:
            memcpy(&referee_info.RFIDStatus, (frame + DATA_Offset), LEN_rfid_status);
            break;
        case ID_dart_launch_status:
            memcpy(&referee_info.DartLaunchStatus, (frame + DATA_Offset), LEN_dart_launch_status);
            g_dart_launch_status_update_seq++;
            RefereeDebug_DartLaunchSeq = g_dart_launch_status_update_seq;
            memcpy(&RefereeDebug_DartLaunchStatus, &referee_info.DartLaunchStatus, sizeof(RefereeDebug_DartLaunchStatus));
            memcpy(RefereeDebug_DartLaunchRaw, (frame + DATA_Offset), sizeof(RefereeDebug_DartLaunchRaw));
            RefereeDebug_DartOpeningStatus = referee_info.DartLaunchStatus.dart_launch_opening_status;
            RefereeDebug_TargetChangeTime = referee_info.DartLaunchStatus.target_change_time;
            RefereeDebug_LatestLaunchCmdTime = referee_info.DartLaunchStatus.latest_launch_cmd_time;
            break;
        case ID_ground_robot_position:
            memcpy(&referee_info.GroundRobotPosition, (frame + DATA_Offset), LEN_ground_robot_position);
            break;
        case ID_radar_mark_data:
            memcpy(&referee_info.RadarMarkData, (frame + DATA_Offset), LEN_radar_mark_data);
            break;
        case ID_sentry_info:
            memcpy(&referee_info.SentryInfo, (frame + DATA_Offset), LEN_sentry_info);
            break;
        case ID_radar_info:
            memcpy(&referee_info.RadarInfo, (frame + DATA_Offset), LEN_radar_info);
            break;
        case ID_student_interactive:
            memcpy(&referee_info.ReceiveData, (frame + DATA_Offset), LEN_receive_data);
            break;
        default:
            break;
        }

        offset += judge_length;
    }
}

referee_info_t *Referee_Register(UART_HandleTypeDef *referee_usart_handle)
{
    if (referee_usart_handle != NULL && referee_usart_handle != &huart8)
    {
        return NULL;
    }

    memset(&referee_info, 0, sizeof(referee_info));
    g_game_state_update_seq = 0U;
    g_dart_info_update_seq = 0U;
    g_dart_launch_status_update_seq = 0U;
    RefereeDebug_CmdID = 0U;
    RefereeDebug_GameStateSeq = 0U;
    RefereeDebug_DartInfoSeq = 0U;
    RefereeDebug_DartLaunchSeq = 0U;
    RefereeDebug_StageRemainTime = 0U;
    RefereeDebug_DartRemainingTime = 0U;
    RefereeDebug_DartOpeningStatus = 0U;
    RefereeDebug_TargetChangeTime = 0U;
    RefereeDebug_LatestLaunchCmdTime = 0U;
    memset(&RefereeDebug_DartLaunchStatus, 0, sizeof(RefereeDebug_DartLaunchStatus));
    memset(RefereeDebug_DartLaunchRaw, 0, sizeof(RefereeDebug_DartLaunchRaw));
    RefereeDebug_SelectedTarget = 0U;
    RefereeDebug_LastParseTick = 0U;
    RefereeDebug_PollCount = 0U;
    RefereeDebug_PacketPopCount = 0U;
    RefereeDebug_PacketValidCount = 0U;
    RefereeDebug_JudgeFrameOkCount = 0U;
    RefereeDebug_JudgeHeaderCrcFailCount = 0U;
    RefereeDebug_JudgeFrameCrcFailCount = 0U;
    RefereeDebug_JudgeInvalidSofCount = 0U;
    RefereeDebug_JudgeIncompleteCount = 0U;
    RefereeDebug_LastPacketLen = 0U;
    RefereeDebug_LastJudgeLen = 0U;
    referee_info.init_flag = 1U;
    // UART8 在 BSP_UART_Init() 中已经切到裁判协议并启动接收，这里只清软件状态。
    UART_ClearRefereePacket(BSP_UART8);
    return &referee_info;
}

bool Referee_Poll(void)
{
    RefereePacket_t packet;
    bool parsed = false;

    RefereeDebug_PollCount++;
    while (UART_GetRefereePacket(BSP_UART8, &packet))
    {
        RefereeDebug_PacketPopCount++;
        RefereeDebug_LastPacketLen = packet.length;
        if (packet.is_valid && packet.length > 0U)
        {
            RefereeDebug_PacketValidCount++;
            JudgeReadData(packet.data, packet.length);
            parsed = true;
        }
    }

    return parsed;
}

uint32_t Referee_GetCmdUpdateSeq(uint16_t cmd_id)
{
    switch (cmd_id)
    {
    case ID_game_state:
        return g_game_state_update_seq;
    case ID_dart_info:
        return g_dart_info_update_seq;
    case ID_dart_launch_status:
        return g_dart_launch_status_update_seq;
    default:
        return 0U;
    }
}

const void *Referee_GetDataByCmdID(uint16_t cmd_id)
{
    switch (cmd_id)
    {
    case ID_game_state:
        return &referee_info.GameState;
    case ID_game_result:
        return &referee_info.GameResult;
    case ID_game_robot_HP:
        return &referee_info.GameRobotHP;
    case ID_event_data:
        return &referee_info.EventData;
    case ID_referee_warning:
        return &referee_info.RefereeWarning;
    case ID_dart_info:
        return &referee_info.DartInfo;
    case ID_robot_performance:
        return &referee_info.RobotPerformance;
    case ID_power_heat_data:
        return &referee_info.PowerHeatData;
    case ID_game_robot_pos:
        return &referee_info.GameRobotPos;
    case ID_buff_musk:
        return &referee_info.BuffMusk;
    case ID_robot_hurt:
        return &referee_info.RobotHurt;
    case ID_shoot_data:
        return &referee_info.ShootData;
    case ID_projectile_allowance:
        return &referee_info.ProjectileAllowance;
    case ID_rfid_status:
        return &referee_info.RFIDStatus;
    case ID_dart_launch_status:
        return &referee_info.DartLaunchStatus;
    case ID_ground_robot_position:
        return &referee_info.GroundRobotPosition;
    case ID_radar_mark_data:
        return &referee_info.RadarMarkData;
    case ID_sentry_info:
        return &referee_info.SentryInfo;
    case ID_radar_info:
        return &referee_info.RadarInfo;
    case ID_student_interactive:
        return &referee_info.ReceiveData;
    default:
        return NULL;
    }
}

/**
 * @brief 裁判系统数据发送函数
 * @param
 */
void Referee_Send(uint8_t *send, uint16_t tx_len)
{
    UART_Send(BSP_UART8, send, tx_len);
    osDelay(40);
}
