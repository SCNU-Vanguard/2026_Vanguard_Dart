#include "RefereeTaskFunc.h"

volatile uint8_t g_ucActiveDartNum = 4U;

extern bool StateSet_BuildRequest(uint8_t dart_num, StateSetRequest_t *request);
extern bool StateSet_SubmitRequest(const StateSetRequest_t *request);

void RefereeTaskMainLoopFunc(void)
{
    uint32_t last_dart_seq = 0U;
    uint32_t last_game_seq = 0U;
    uint32_t last_dart_info_seq = 0U;
    while (1)
    {
        bool parsed = Referee_Poll();

        if (parsed)
        {
            // 轮询获取裁判系统信息
            uint32_t dart_seq = Referee_GetCmdUpdateSeq(ID_dart_launch_status);
            uint32_t game_seq = Referee_GetCmdUpdateSeq(ID_game_state);
            uint32_t dart_info_seq = Referee_GetCmdUpdateSeq(ID_dart_info);

            bool gate_changed = false;
            FirePermission_t permission = {0};

            // 选手端对于飞镖操作
            if (dart_seq != last_dart_seq)
            {
                const ext_dart_launch_status_t *dart_status = Referee_GetDataByCmdID(ID_dart_launch_status);
                gate_changed = FireControl_UpdateRefereeGate(dart_status);
                last_dart_seq = dart_seq;
            }

            // 全局时间信息
            if (game_seq != last_game_seq)
            {
                const ext_game_state_t *game_state = Referee_GetDataByCmdID(ID_game_state);
                if (game_state != NULL)
                {
                    (void)FireControl_UpdateRefereeRemainTime(game_state->stage_remain_time);
                }
                last_game_seq = game_seq;
            }

            // 飞镖发射站相关目标选定和剩余时间
            if (dart_info_seq != last_dart_info_seq)
            {
                const ext_dart_info_t *dart_info = Referee_GetDataByCmdID(ID_dart_info);
                (void)FireControl_UpdateDartInfo(dart_info);
                last_dart_info_seq = dart_info_seq;
            }

            // 确认最后发射数据
            FireControl_GetPermission(&permission);
            if (permission.need_reconfigure || gate_changed)
            {
                StateSetRequest_t request = {0};
                if (StateSet_BuildRequest(g_ucActiveDartNum, &request))
                {
                    (void)StateSet_SubmitRequest(&request);
                }
            }
        }

        vTaskDelay(pdMS_TO_TICKS(100));
    }
}
