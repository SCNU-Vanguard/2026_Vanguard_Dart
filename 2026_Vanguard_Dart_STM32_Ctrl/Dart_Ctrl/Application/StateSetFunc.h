#ifndef __STATE_SET_FUNC_H_ /* 按 __STATE_SET_FUNC_H_ 选择编译分支。 */
#define __STATE_SET_FUNC_H_ /* 定义 __STATE_SET_FUNC_H_。 */

#include "FreeRTOS.h"
#include <stdbool.h>
#include <stdint.h>

typedef struct /* 开始定义数据类型。 */
{
    uint32_t seq; /* 保存 seq。 */
    uint32_t ref_seq; /* 保存 ref_seq。 */
    uint32_t intent_seq; /* 保存 intent_seq。 */
    uint8_t dart_num; /* 保存 dart_num。 */
    float yaw_target; /* 保存 yaw_target。 */
    float trigger_target; /* 保存 trigger_target。 */
} StateSetRequest_t; /* 结束 StateSetRequest_t 类型定义。 */

bool StateSet_BuildRequest(uint8_t dart_num, StateSetRequest_t *request); /* 声明 StateSet_BuildRequest 接口。 */
bool StateSet_ApplyYawPreset(float target_yaw); /* 声明 StateSet_ApplyYawPreset 接口。 */
bool StateSet_SubmitRequest(const StateSetRequest_t *request); /* 声明 StateSet_SubmitRequest 接口。 */
bool StateSet_RequestAndWait(uint8_t dart_num, TickType_t timeout_ticks); /* 声明 StateSet_RequestAndWait 接口。 */
void StateSetTaskMainLoopFunc(void); /* 声明 StateSetTaskMainLoopFunc 接口。 */

#endif /* 结束条件编译。 */
