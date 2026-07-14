#include "RefereeTaskFunc.h"

volatile uint8_t g_ucActiveDartNum = 4U; /* 初始化 g_ucActiveDartNum。 */

extern bool StateSet_BuildRequest(uint8_t dart_num, StateSetRequest_t *request); /* 声明 StateSet_BuildRequest 接口。 */
extern bool StateSet_SubmitRequest(const StateSetRequest_t *request); /* 声明 StateSet_SubmitRequest 接口。 */

/// @note 当前没有使用遥控器控制数据
void RefereeTaskMainLoopFunc(void) /* 实现 RefereeTaskMainLoopFunc。 */
{
    uint32_t last_dart_seq = 0U; /* 初始化 last_dart_seq。 */
    uint32_t last_game_seq = 0U; /* 初始化 last_game_seq。 */
    uint32_t last_dart_info_seq = 0U; /* 初始化 last_dart_info_seq。 */
    while (1) /* 持续执行当前任务。 */
    {
        bool parsed = Referee_Poll(); /* 初始化 parsed。 */

        if (parsed) /* 检查当前执行条件。 */
        {
            // 轮询获取裁判系统信息
            uint32_t dart_seq = Referee_GetCmdUpdateSeq(ID_dart_launch_status); /* 初始化 dart_seq。 */
            uint32_t game_seq = Referee_GetCmdUpdateSeq(ID_game_state); /* 初始化 game_seq。 */
            uint32_t dart_info_seq = Referee_GetCmdUpdateSeq(ID_dart_info); /* 初始化 dart_info_seq。 */

            bool gate_changed = false; /* 初始化 gate_changed。 */
            FirePermission_t permission = {0}; /* 初始化 permission。 */

            // 选手端对于飞镖操作
            if (dart_seq != last_dart_seq) /* 检查当前执行条件。 */
            {
                const ext_dart_launch_status_t *dart_status = Referee_GetDataByCmdID(ID_dart_launch_status); /* 初始化 dart_status。 */
                gate_changed = FireControl_UpdateRefereeGate(dart_status); /* 更新 gate_changed。 */
                last_dart_seq = dart_seq; /* 更新 last_dart_seq。 */
            }

            // 全局时间信息
            if (game_seq != last_game_seq) /* 检查当前执行条件。 */
            {
                const ext_game_state_t *game_state = Referee_GetDataByCmdID(ID_game_state); /* 初始化 game_state。 */
                if (game_state != NULL) /* 检查当前执行条件。 */
                {
                    (void)FireControl_UpdateRefereeRemainTime(game_state->stage_remain_time); /* 调用 FireControl_UpdateRefereeRemainTime。 */
                }
                last_game_seq = game_seq; /* 更新 last_game_seq。 */
            }

            // 飞镖发射站相关目标选定和剩余时间
            if (dart_info_seq != last_dart_info_seq) /* 检查当前执行条件。 */
            {
                const ext_dart_info_t *dart_info = Referee_GetDataByCmdID(ID_dart_info); /* 初始化 dart_info。 */
                (void)FireControl_UpdateDartInfo(dart_info); /* 调用 FireControl_UpdateDartInfo。 */
                last_dart_info_seq = dart_info_seq; /* 更新 last_dart_info_seq。 */
            }

            // 确认最后发射数据
            FireControl_GetPermission(&permission); /* 调用 FireControl_GetPermission。 */
            if (permission.need_reconfigure || gate_changed) /* 检查当前执行条件。 */
            {
                StateSetRequest_t request = {0}; /* 初始化 request。 */
                if (StateSet_BuildRequest(g_ucActiveDartNum, &request)) /* 检查当前执行条件。 */
                {
                    (void)StateSet_SubmitRequest(&request); /* 调用 StateSet_SubmitRequest。 */
                }
            }
        }

        vTaskDelay(pdMS_TO_TICKS(100)); /* 调用 vTaskDelay。 */
    }
}
