#ifndef __LOAD_TASK_FUNC_H_ /* 按 __LOAD_TASK_FUNC_H_ 选择编译分支。 */
#define __LOAD_TASK_FUNC_H_ /* 定义 __LOAD_TASK_FUNC_H_。 */

#include "UserTask.h"
#include "FreeRTOS.h"
#include "cmsis_os.h"
#include "task.h"
#include "semphr.h"
#include "event_groups.h"
#include "stream_buffer.h"

void LoadServoTaskMainLoopFunc(void); /* 声明 LoadServoTaskMainLoopFunc 接口。 */
void LoadMotorTaskMainLoopFunc(void); /* 声明 LoadMotorTaskMainLoopFunc 接口。 */

#endif /* 结束条件编译。 */