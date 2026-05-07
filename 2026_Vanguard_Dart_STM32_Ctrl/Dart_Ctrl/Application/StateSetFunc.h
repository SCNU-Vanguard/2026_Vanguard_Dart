#ifndef __STATE_SET_FUNC_H_
#define __STATE_SET_FUNC_H_

#include "FreeRTOS.h"
#include <stdbool.h>
#include <stdint.h>

typedef struct
{
    uint32_t seq;
    uint32_t ref_seq;
    uint32_t intent_seq;
    uint8_t dart_num;
    float yaw_target;
    float trigger_target;
} StateSetRequest_t;

bool StateSet_BuildRequest(uint8_t dart_num, StateSetRequest_t *request);
bool StateSet_ApplyYawPreset(float target_yaw);
bool StateSet_ApplyRequest(const StateSetRequest_t *request);
bool StateSet_ApplyPreset(uint8_t dart_num);
bool StateSet_SubmitRequest(const StateSetRequest_t *request);
bool StateSet_RequestAndWait(uint8_t dart_num, TickType_t timeout_ticks);
void StateSetTaskMainLoopFunc(void);

#endif
