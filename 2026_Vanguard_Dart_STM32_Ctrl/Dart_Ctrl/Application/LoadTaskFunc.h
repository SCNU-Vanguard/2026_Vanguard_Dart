#ifndef __LOAD_TASK_FUNC_H_
#define __LOAD_TASK_FUNC_H_

#include "UserTask.h"
#include "FreeRTOS.h"
#include "cmsis_os.h"
#include "task.h"
#include "semphr.h"
#include "event_groups.h"
#include "stream_buffer.h"

void LoadServoTaskMainLoopFunc(void);
void LoadMotorTaskMainLoopFunc(void);

#endif