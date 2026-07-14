/*****************************************************
 * 通用 CAN 电机管理
 ****************************************************/

#include "CanMotor.h"
#include "RM_Motor.h"
#include "DM_Motor.h"
#include "angle_motor.h"
#include "config.h"
#include <stdbool.h>
#include <string.h>
#include "FreeRTOS.h"
#include "task.h"

MotorManager_t MotorManager = {0}; /* 初始化 MotorManager。 */

static uint8_t Motor_DefaultCanSend(CAN_TxHeaderTypeDef *hdr, uint8_t *data) /* 实现 Motor_DefaultCanSend。 */
{
    return CAN_SendData(&hcan1, hdr, data); /* 返回当前计算结果。 */
}

static void Motor_DefaultDelayMs(uint32_t ms) /* 实现 Motor_DefaultDelayMs。 */
{
    vTaskDelay(ms); /* 调用 vTaskDelay。 */
}

static const MotorHAL_t g_DefaultMotorHAL = { /* 初始化 g_DefaultMotorHAL。 */
    .can_send = Motor_DefaultCanSend, /* 配置 can_send。 */
    .delay_ms = Motor_DefaultDelayMs, /* 配置 delay_ms。 */
};

const MotorHAL_t *g_pMotorHAL = &g_DefaultMotorHAL; /* 初始化 g_pMotorHAL。 */

const MotorHAL_t *Motor_GetHAL(void) /* 实现 Motor_GetHAL。 */
{
    return g_pMotorHAL; /* 返回当前计算结果。 */
}

void Motor_SetHAL(const MotorHAL_t *hal) /* 实现 Motor_SetHAL。 */
{
    g_pMotorHAL = (hal != NULL) ? hal : &g_DefaultMotorHAL; /* 更新 g_pMotorHAL。 */
}

void CanRegisterMotorCfg(MotorTypeDef *ptr) /* 实现 CanRegisterMotorCfg。 */
{
    if (ptr == NULL) /* 检查当前执行条件。 */
    {
        return; /* 结束当前函数。 */
    }

    ptr->band = ptr->MotorInf.band; /* 更新 band。 */
    ptr->model = ptr->MotorInf.model; /* 更新 model。 */

    if (ptr->band == RM_MOTOR_BAND) /* 检查当前执行条件。 */
    {
        /* RM 电调没有独立使能协议，注册完成后即可接收电流指令。 */
        ptr->drive_enabled = 1U; /* 更新 drive_enabled。 */
        if (ptr->motor_class.rm_motor_class != NULL) /* 检查当前执行条件。 */
        {
            ptr->gear_ratio = ptr->motor_class.rm_motor_class->gear_ratio; /* 更新 gear_ratio。 */
            ptr->current_ratio = ptr->motor_class.rm_motor_class->current_ratio; /* 更新 current_ratio。 */
            ptr->max_current = ptr->motor_class.rm_motor_class->max_current; /* 更新 max_current。 */
            ptr->calculate = ptr->motor_class.rm_motor_class->calculate; /* 更新 calculate。 */
        }

        if (ptr->model == RmM3508) /* 检查当前执行条件。 */
        {
            ptr->g_TxHeader.StdId = g_RM_MOTOR_BIAS_ADDR_3508; /* 更新 StdId。 */
        }
        else if (ptr->model == RmM2006) /* 继续判断下一条件。 */
        {
            ptr->g_TxHeader.StdId = g_RM_MOTOR_BIAS_ADDR_2006; /* 更新 StdId。 */
        }
        else if (ptr->model == RmGM6020) /* 继续判断下一条件。 */
        {
            ptr->g_TxHeader.StdId = g_RM_MOTOR_BIAS_ADDR_6020; /* 更新 StdId。 */
        }
    }
    else if (ptr->band == DM_MOTOR_BAND) /* 继续判断下一条件。 */
    {
        /* DM 必须成功发送协议使能帧后才能接收控制指令。 */
        ptr->drive_enabled = 0U; /* 更新 drive_enabled。 */
        if (ptr->motor_class.dm_motor_class != NULL) /* 检查当前执行条件。 */
        {
            ptr->max_current = (int16_t)ptr->motor_class.dm_motor_class->torque_max; /* 更新 max_current。 */
            ptr->calculate = ptr->motor_class.dm_motor_class->calculate; /* 更新 calculate。 */
            uint8_t idx = (uint8_t)(ptr - MotorManager.MotorList); /* 初始化 idx。 */
            can_motor_cfg motor_cfg = (can_motor_cfg)(idx + 1U); /* 初始化 motor_cfg。 */
            const DM_MotorIdConfig_t *id_cfg = DM_GetIdConfig(motor_cfg); /* 初始化 id_cfg。 */
            if (id_cfg != NULL) /* 检查当前执行条件。 */
            {
                ptr->g_TxHeader.StdId = id_cfg->tx_id; /* 更新 StdId。 */
            }
        }
    }

    ptr->can_id_tx = ptr->g_TxHeader.StdId; /* 更新 can_id_tx。 */
    ptr->can_id_rx = ptr->CAN_Rid; /* 更新 can_id_rx。 */
    ptr->g_TxHeader.IDE = CAN_ID_STD; /* 更新 IDE。 */
    ptr->g_TxHeader.RTR = CAN_RTR_DATA; /* 更新 RTR。 */
    ptr->g_TxHeader.DLC = CtrlMotorLen; /* 更新 DLC。 */
}

void MotorRegister(void) /* 实现 MotorRegister。 */
{
    // 换弹夹爪 M3508：目标坐标统一使用“相对零点角度”，因此限位也直接注册到 RM 电机配置里。
    RM_MotorConfig_t gripper_config = { /* 初始化 gripper_config。 */
        .direction_bias = 0.0f, /* 配置 direction_bias。 */
        .position_min = LOAD3508_MIN, /* 配置 position_min。 */
        .position_max = LOAD3508_MAX, /* 配置 position_max。 */
        .position_tolerance = MOTOR_DEAD_ZONE, /* 配置 position_tolerance。 */
        .reverse = 0U, /* 配置 reverse。 */
    };

    // 左右储能 M3508：各自注册独立的位置限位，后续 PID/保护都按各自配置生效。
    // TODO确认角度正负号
    RM_MotorConfig_t left_store_config = { /* 初始化 left_store_config。 */
        .direction_bias = 0.0f, /* 配置 direction_bias。 */
        .position_min = 0.0f, /* 配置 position_min。 */
        .position_max = LimitStore, /* 配置 position_max。 */
        .position_tolerance = 0.0f, /* 配置 position_tolerance。 */
        .reverse = 0U, /* 配置 reverse。 */
    };
    RM_MotorConfig_t right_store_config = { /* 初始化 right_store_config。 */
        .direction_bias = 0.0f, /* 配置 direction_bias。 */
        .position_min = 0.0f, /* 配置 position_min。 */
        .position_max = LimitStore, /* 配置 position_max。 */
        .position_tolerance = 0.0f, /* 配置 position_tolerance。 */
        .reverse = 1U, /* 配置 reverse。 */
    };

    // 左右储能 M3508 共用同一组 S 型规划；
    // 如果后面要左右分别调参，可以直接拆成 left/right 两份注册结构。
    MotorTrapConfig_t store_trap_config = { /* 初始化 store_trap_config。 */
        .registered = 1U, /* 配置 registered。 */
        .resync_on_target_change = 1U, /* 配置 resync_on_target_change。 */
        .vmax_deg_s = STORE3508_TRAP_VMAX_DEG_S, /* 配置 vmax_deg_s。 */
        .amax_deg_s2 = STORE3508_TRAP_AMAX_DEG_S2, /* 配置 amax_deg_s2。 */
#if STORE3508_TRAP_DISABLE_JERK /* 按 STORE3508_TRAP_DISABLE_JERK 选择编译分支。 */
        .jmax_deg_s3 = 0.0f, /* 配置 jmax_deg_s3。 */
#else /* 切换到备用编译分支。 */
        .jmax_deg_s3 = STORE3508_TRAP_AMAX_DEG_S2 * STORE3508_TRAP_JERK_FACTOR, /* 配置 jmax_deg_s3。 */
#endif /* 结束条件编译。 */
        .brake_gain = STORE3508_TRAP_BRAKE_GAIN, /* 配置 brake_gain。 */
        .arrive_zone = STORE3508_TRAP_ARRIVE_ZONE, /* 配置 arrive_zone。 */
        .decel_zone = STORE3508_TRAP_DECEL_ZONE, /* 配置 decel_zone。 */
    };

    // 夹爪 M3508 的 S 型规划参数
    MotorTrapConfig_t gripper_trap_config = { /* 初始化 gripper_trap_config。 */
        .registered = 1U, /* 配置 registered。 */
        .resync_on_target_change = LOAD_TASK_TRAP_RESET_ON_TARGET_CHANGE, /* 配置 resync_on_target_change。 */
        .vmax_deg_s = LOAD_TASK_TRAP_VMAX_DEG_S, /* 配置 vmax_deg_s。 */
        .amax_deg_s2 = LOAD_TASK_TRAP_AMAX_DEG_S2, /* 配置 amax_deg_s2。 */
#if LOAD_TASK_TRAP_DISABLE_JERK /* 按 LOAD_TASK_TRAP_DISABLE_JERK 选择编译分支。 */
        .jmax_deg_s3 = 0.0f, /* 配置 jmax_deg_s3。 */
#else /* 切换到备用编译分支。 */
        .jmax_deg_s3 = LOAD_TASK_TRAP_AMAX_DEG_S2 * LOAD_TASK_TRAP_JERK_FACTOR, /* 配置 jmax_deg_s3。 */
#endif /* 结束条件编译。 */
        .brake_gain = LOAD_TASK_TRAP_BRAKE_GAIN, /* 配置 brake_gain。 */
        .arrive_zone = LOAD_TASK_TRAP_ARRIVE_ZONE, /* 配置 arrive_zone。 */
        .decel_zone = LOAD_TASK_TRAP_DECEL_ZONE, /* 配置 decel_zone。 */
    };

    RM_M3508_Create(&MotorManager.MotorList[RM_3508_GRIPPER - 1], 1U); /* 调用 RM_M3508_Create。 */
    RM_Motor_SetConfig(&MotorManager.MotorList[RM_3508_GRIPPER - 1], &gripper_config); /* 调用 RM_Motor_SetConfig。 */
    RM_Motor_SetCascadePID(&MotorManager.MotorList[RM_3508_GRIPPER - 1], /* 传入下一项参数或数据。 */
                           14.00f, 0.1f, 0.05f, 10.0f, /* 继续配置下一项。 */
                           0.225f, 1.00f, 0.05f, 93.0f, /* 继续配置下一项。 */
                           20000.0f, 0.0f, 500.0f, /* 继续配置下一项。 */
                           8000.0f, 0.0f, 1500.0f); /* 完成本行操作。 */
    MotorManager.MotorList[RM_3508_GRIPPER - 1].trap_config = gripper_trap_config; /* 更新 trap_config。 */
    MotorManager.MotorList[RM_3508_GRIPPER - 1].CAN_Rid = 0x201; /* 更新 CAN_Rid。 */

    /* 0x1FF controls feedback IDs 0x205..0x208; 0x205 is local slot 1. */
    RM_M2006_Create(&MotorManager.MotorList[RM_2006_TRIGGER - 1], 1U); /* 调用 RM_M2006_Create。 */
    RM_Motor_SetCascadePID(&MotorManager.MotorList[RM_2006_TRIGGER - 1], /* 传入下一项参数或数据。 */
                           0.3f, 0.0f, 0.0f, 0.000f, /* 继续配置下一项。 */
                           15.91f, 0.0f, 0.0f, 0.0f, /* 继续配置下一项。 */
                           9000.0f, 0.0f, 0.0f, /* 继续配置下一项。 */
                           6000.0f, 0.0f, 0.0f); /* 完成本行操作。 */
    MotorManager.MotorList[RM_2006_TRIGGER - 1].CAN_Rid = 0x205; /* 更新 CAN_Rid。 */

#if USE_RM_STORE /* 按 USE_RM_STORE 选择编译分支。 */
    RM_M3508_Create(&MotorManager.MotorList[RM_3508_STORE_RIGHT - 1], 2U); /* 调用 RM_M3508_Create。 */
    RM_Motor_SetConfig(&MotorManager.MotorList[RM_3508_STORE_RIGHT - 1], &right_store_config); /* 调用 RM_Motor_SetConfig。 */
    RM_Motor_SetCascadePID(&MotorManager.MotorList[RM_3508_STORE_RIGHT - 1], /* 传入下一项参数或数据。 */
                           10.50f, 1.50f, 100.0f, 0.0f, /* 继续配置下一项。 */
                           11.20f, 3.50f, 0.0f, 0.0f, /* 继续配置下一项。 */
                           400.0f, 0.0f, 50.0f, /* 继续配置下一项。 */
                           30000.0f, 0.0f, 10000.0f); /* 完成本行操作。 */
    MotorManager.MotorList[RM_3508_STORE_RIGHT - 1].trap_config = store_trap_config; /* 更新 trap_config。 */
    MotorManager.MotorList[RM_3508_STORE_RIGHT - 1].CAN_Rid = 0x202; /* 更新 CAN_Rid。 */

    RM_M3508_Create(&MotorManager.MotorList[RM_3508_STORE_LEFT - 1], 3U); /* 调用 RM_M3508_Create。 */
    RM_Motor_SetConfig(&MotorManager.MotorList[RM_3508_STORE_LEFT - 1], &left_store_config); /* 调用 RM_Motor_SetConfig。 */
    RM_Motor_SetCascadePID(&MotorManager.MotorList[RM_3508_STORE_LEFT - 1], /* 传入下一项参数或数据。 */
                           10.50f, 1.50f, 100.00f, 0.0f, /* 继续配置下一项。 */
                           11.20f, 3.50f, 0.0f, 0.0f, /* 继续配置下一项。 */
                           400.0f, 0.0f, 50.0f, /* 继续配置下一项。 */
                           30000.0f, 0.0f, 10000.0f); /* 完成本行操作。 */
    MotorManager.MotorList[RM_3508_STORE_LEFT - 1].trap_config = store_trap_config; /* 更新 trap_config。 */
    MotorManager.MotorList[RM_3508_STORE_LEFT - 1].CAN_Rid = 0x203; /* 更新 CAN_Rid。 */
#else /* 切换到备用编译分支。 */
    {
        MotorTypeDef *right_store = &MotorManager.MotorList[DM_3519_STRENTH_RIGHT - 1]; /* 初始化 right_store。 */
        MotorTypeDef *left_store = &MotorManager.MotorList[DM_3519_STRENTH_LEFT - 1]; /* 初始化 left_store。 */
        const DM_MotorIdConfig_t *right_id = DM_GetIdConfig(DM_3519_STRENTH_RIGHT); /* 初始化 right_id。 */
        const DM_MotorIdConfig_t *left_id = DM_GetIdConfig(DM_3519_STRENTH_LEFT); /* 初始化 left_id。 */

        DM_J3519_Init(right_store, right_id->motor_id); /* 调用 DM_J3519_Init。 */
        right_store->config.direction_bias = right_store_config.direction_bias; /* 更新 direction_bias。 */
        right_store->config.position_min = right_store_config.position_min; /* 更新 position_min。 */
        right_store->config.position_max = right_store_config.position_max; /* 更新 position_max。 */
        right_store->config.position_tolerance = right_store_config.position_tolerance; /* 更新 position_tolerance。 */
        right_store->config.reverse = right_store_config.reverse; /* 更新 reverse。 */
        right_store->trap_config = store_trap_config; /* 更新 trap_config。 */
        right_store->CAN_Rid = right_id->rx_id; /* 更新 CAN_Rid。 */
        DM_Motor_SetPosLimits(right_store, DegreeToRad(-LimitStore), 0.0f); /* 调用 DM_Motor_SetPosLimits。 */

        DM_J3519_Init(left_store, left_id->motor_id); /* 调用 DM_J3519_Init。 */
        left_store->config.direction_bias = left_store_config.direction_bias; /* 更新 direction_bias。 */
        left_store->config.position_min = left_store_config.position_min; /* 更新 position_min。 */
        left_store->config.position_max = left_store_config.position_max; /* 更新 position_max。 */
        left_store->config.position_tolerance = left_store_config.position_tolerance; /* 更新 position_tolerance。 */
        left_store->config.reverse = left_store_config.reverse; /* 更新 reverse。 */
        left_store->trap_config = store_trap_config; /* 更新 trap_config。 */
        left_store->CAN_Rid = left_id->rx_id; /* 更新 CAN_Rid。 */
        DM_Motor_SetPosLimits(left_store, 0.0f, DegreeToRad(LimitStore)); /* 调用 DM_Motor_SetPosLimits。 */

        DM_Motor_Clear3519StallProtection(); /* 调用 DM_Motor_Clear3519StallProtection。 */
    }
#endif /* 结束条件编译。 */

    /* 双侧蓄力 3508 位置同步 PID：对 (left_pos - right_pos) 做 PID，
     * 上层按 LeftFinal=LeftTarget+correction、RightFinal=RightTarget-correction 分发。*/
    AngleMotor_InitStoreSyncPid(STORE3508_SYNC_PID_KP, STORE3508_SYNC_PID_KI, /* 传入下一项参数或数据。 */
                                STORE3508_SYNC_PID_KD, STORE3508_SYNC_PID_KF, /* 传入下一项参数或数据。 */
                                STORE3508_SYNC_PID_MAX_OUT, STORE3508_SYNC_PID_MIN_OUT, /* 传入下一项参数或数据。 */
                                STORE3508_SYNC_PID_MAX_IOUT); /* 完成本行操作。 */

    // Yaw轴电机 - J4310
    DM_J4310_Init(&MotorManager.MotorList[DM_4310_YAW - 1], /* 传入下一项参数或数据。 */
                  DM_GetIdConfig(DM_4310_YAW)->motor_id); /* 调用 DM_GetIdConfig。 */
    DM_Motor_SetVelLimits(&MotorManager.MotorList[DM_4310_YAW - 1], -30.0f, 30.0f); /* 调用 DM_Motor_SetVelLimits。 */
    DM_Motor_SetPosLimits(&MotorManager.MotorList[DM_4310_YAW - 1], -160.0f, 160.0f); /* 调用 DM_Motor_SetPosLimits。 */
    DM_Motor_SetMITParams(&MotorManager.MotorList[DM_4310_YAW - 1], 1.4591f, 1.0f, 0.0f); /* 调用 DM_Motor_SetMITParams。 */
    MotorManager.MotorList[DM_4310_YAW - 1].CAN_Rid = DM_GetIdConfig(DM_4310_YAW)->rx_id; /* 更新 CAN_Rid。 */

    MotorManager.registered_count = 5U; /* 更新 registered_count。 */
    for (uint8_t i = 0; i < MotorManager.registered_count; i++) /* 遍历当前数据集合。 */
    {
        CanRegisterMotorCfg(&MotorManager.MotorList[i]); /* 调用 CanRegisterMotorCfg。 */
    }
}

void MotorInit(void) /* 实现 MotorInit。 */
{
#if TestUse /* 按 TestUse 选择编译分支。 */
#if DM_TestUse /* 按 DM_TestUse 选择编译分支。 */
    DmTestMotorSingleRegister(); /* 调用 DmTestMotorSingleRegister。 */
#elif RM_TestUse /* 继续检查 RM_TestUse。 */
    RmTestMotorSingleRegister(); /* 调用 RmTestMotorSingleRegister。 */
#endif /* 结束条件编译。 */
    for (uint8_t i = 0; i < MotorManager.registered_count; i++) /* 遍历当前数据集合。 */
    {
        CanRegisterMotorCfg(&MotorManager.MotorList[i]); /* 调用 CanRegisterMotorCfg。 */
    }
#else /* 切换到备用编译分支。 */
    MotorRegister(); /* 调用 MotorRegister。 */
#endif /* 结束条件编译。 */

    CAN_Init(&hcan1, fifo0, 0, 0); /* 调用 CAN_Init。 */
    CAN_Init(&hcan1, fifo1, 10, 0); /* 调用 CAN_Init。 */
    HAL_Delay(5); /* 调用 HAL_Delay。 */
}

void CanMotor_WatchdogInit(void) /* 实现 CanMotor_WatchdogInit。 */
{
    uint8_t i; /* 电机表索引。 */

    /* 为每个已注册电机分配独立看门狗。 */
    for (i = 0U; i < MotorManager.registered_count; i++) /* 遍历当前数据集合。 */
    {
        SoftwareWatchdogId_e id = SoftwareWatchdog_MotorId((uint8_t)(i + 1U)); /* 映射看门狗编号。 */
        MotorManager.MotorList[i].last_rx_tick = 0U;             /* 尚无反馈时间。 */
        MotorManager.MotorList[i].online = 0U;                   /* 上电默认离线。 */
        MotorManager.MotorList[i].watchdog_lost_pending = 0U;    /* 清任务待处理标志。 */
        MotorManager.MotorList[i].dm_reply_pending = 0U;         /* 清 DM 回复等待。 */
        /* DM 使用一次性回复阈值，RM 使用连续反馈阈值。 */
        (void)SoftwareWatchdog_Register(id, /* 开始调用 SoftwareWatchdog_Register。 */
                                        (MotorManager.MotorList[i].MotorInf.band == DM_MOTOR_BAND) /* 继续更新 目标值。 */
                                            ? DM_MOTOR_REPLY_TIMEOUT_MS /* 继续当前语句。 */
                                            : RM_MOTOR_FEEDBACK_TIMEOUT_MS, /* 继续配置下一项。 */
                                        CanMotor_WatchdogLostCallback); /* 完成本行操作。 */
        if (MotorManager.MotorList[i].MotorInf.band == DM_MOTOR_BAND) /* 检查当前执行条件。 */
        {
            SoftwareWatchdog_Disarm(id); /* DM 未发送时不计时。 */
        }
    }
}

bool CanMotor_DmReplyWaitBegin(MotorTypeDef *motor) /* 实现 CanMotor_DmReplyWaitBegin。 */
{
    uint32_t primask;          /* 保存进入函数前的中断状态。 */
    uint32_t index;            /* 电机表索引。 */
    SoftwareWatchdogId_e id;   /* 对应看门狗编号。 */
    bool started = false;      /* 本次是否成功占用回复窗口。 */

    /* 该接口只接受已注册的 DM 电机。 */
    if (motor == NULL || motor->MotorInf.band != DM_MOTOR_BAND) /* 检查当前执行条件。 */
    {
        return false; /* 指针或品牌无效。 */
    }

    index = (uint32_t)(motor - MotorManager.MotorList); /* 由句柄反查索引。 */
    if (index >= MotorManager.registered_count) /* 检查当前执行条件。 */
    {
        return false; /* 电机不在注册表中。 */
    }
    id = SoftwareWatchdog_MotorId((uint8_t)(index + 1U)); /* 映射看门狗编号。 */

    primask = __get_PRIMASK(); /* 记录原中断开关。 */
    __disable_irq();           /* 防止反馈中断抢在 Arm 之前清标志。 */
    if (motor->dm_reply_pending == 0U) /* 检查当前执行条件。 */
    {
        motor->dm_reply_pending = 1U;         /* 先声明正在等待回复。 */
        started = SoftwareWatchdog_Arm(id);   /* 再启动一次性计时。 */
        if (!started) /* 检查当前执行条件。 */
        {
            motor->dm_reply_pending = 0U; /* Arm 失败时立即回滚。 */
        }
    }
    if (primask == 0U) /* 检查当前执行条件。 */
    {
        __enable_irq(); /* 仅恢复原本开启的中断。 */
    }
    return started; /* false 同时表示已有未完成回复。 */
}

void CanMotor_DmReplyWaitCancel(MotorTypeDef *motor) /* 实现 CanMotor_DmReplyWaitCancel。 */
{
    uint32_t primask;        /* 保存原中断状态。 */
    uint32_t index;          /* 电机表索引。 */
    SoftwareWatchdogId_e id; /* 对应看门狗编号。 */

    /* 只处理有效 DM 电机。 */
    if (motor == NULL || motor->MotorInf.band != DM_MOTOR_BAND) /* 检查当前执行条件。 */
    {
        return; /* 指针或品牌无效。 */
    }

    index = (uint32_t)(motor - MotorManager.MotorList); /* 由句柄反查索引。 */
    if (index >= MotorManager.registered_count) /* 检查当前执行条件。 */
    {
        return; /* 电机不在注册表中。 */
    }
    id = SoftwareWatchdog_MotorId((uint8_t)(index + 1U)); /* 映射看门狗编号。 */

    primask = __get_PRIMASK();       /* 记录原中断开关。 */
    __disable_irq();                 /* 与反馈中断原子互斥。 */
    motor->dm_reply_pending = 0U;    /* 清发送等待标志。 */
    SoftwareWatchdog_Disarm(id);     /* 停止本次回复计时。 */
    if (primask == 0U) /* 检查当前执行条件。 */
    {
        __enable_irq(); /* 恢复原中断状态。 */
    }
}

void CanMotor_WatchdogLostCallback(SoftwareWatchdogId_e id) /* 实现 CanMotor_WatchdogLostCallback。 */
{
    uint32_t index; /* 超时通道对应的电机索引。 */

    /* 忽略非 CAN 电机通道。 */
    if (id < SOFTWARE_WATCHDOG_CAN_MOTOR_1 || id > SOFTWARE_WATCHDOG_CAN_MOTOR_5) /* 检查当前执行条件。 */
    {
        return; /* 通道类型不匹配。 */
    }

    index = (uint32_t)id - (uint32_t)SOFTWARE_WATCHDOG_CAN_MOTOR_1; /* 还原电机索引。 */
    if (index < MotorManager.registered_count) /* 检查当前执行条件。 */
    {
        /* TIM4 中断内只发布状态，不发送 CAN 或操作 RTOS。 */
        if (MotorManager.MotorList[index].MotorInf.band == DM_MOTOR_BAND) /* 检查当前执行条件。 */
        {
            if (MotorManager.MotorList[index].dm_reply_pending == 0U) /* 检查当前执行条件。 */
            {
                return; /* 已及时收到反馈，不再报丢失。 */
            }
            MotorManager.MotorList[index].dm_reply_pending = 0U; /* 结束本次回复等待。 */
        }
        MotorManager.MotorList[index].online = 0U;                  /* 立即发布离线状态。 */
        MotorManager.MotorList[index].watchdog_lost_pending = 1U;  /* 通知任务执行安全清理。 */
    }
}

bool Motor_IsOnline(can_motor_cfg motor_id) /* 实现 Motor_IsOnline。 */
{
    SoftwareWatchdogId_e id; /* RM 连续反馈看门狗编号。 */
    MotorTypeDef *motor;      /* 被查询电机。 */

    /* 越界编号按离线处理。 */
    if (motor_id < 1 || motor_id > MotorManager.registered_count) /* 检查当前执行条件。 */
    {
        return false; /* 电机不存在。 */
    }

    motor = &MotorManager.MotorList[motor_id - 1]; /* 取得电机状态。 */
    if (motor->MotorInf.band == DM_MOTOR_BAND) /* 检查当前执行条件。 */
    {
        return (motor->online != 0U); /* DM 在线状态由请求回复结果锁存。 */
    }
    id = SoftwareWatchdog_MotorId((uint8_t)motor_id); /* 映射 RM 连续看门狗。 */
    /* RM 必须在线标志和连续看门狗同时正常。 */
    return (motor->online != 0U && SoftwareWatchdog_IsHealthy(id)); /* 返回当前计算结果。 */
}

void CanFilterCfg(void) /* 实现 CanFilterCfg。 */
{
    HAL_CAN_Stop(&hcan1); /* 调用 HAL_CAN_Stop。 */
    uint16_t id_mask_arr[2] = {0x0000, 0x0000}; /* 初始化 id_mask_arr。 */
    uint16_t id_arr[2] = {0x0200, 0x0010}; /* 初始化 id_arr。 */
    FliterIdCfg_Init(&hcan1, id_arr, id_mask_arr, 0, fifo0); /* 调用 FliterIdCfg_Init。 */
    HAL_Delay(5); /* 调用 HAL_Delay。 */
    FliterIdCfg_Init(&hcan1, id_arr, id_mask_arr, 10, fifo1); /* 调用 FliterIdCfg_Init。 */
    HAL_CAN_Start(&hcan1); /* 调用 HAL_CAN_Start。 */
    HAL_Delay(10); /* 调用 HAL_Delay。 */
}

void CAN_FIFO_CBKHANDLER(uint32_t fifo_num, uint8_t FIFOmessageNum) /* 实现 CAN_FIFO_CBKHANDLER。 */
{
    static uint8_t rx_data[8]; /* 单帧接收缓冲区。 */
    CAN_RxHeaderTypeDef rx_header; /* 当前报文头。 */

    /* 依次取出本次 FIFO 中的全部报文。 */
    for (uint8_t a = 0; a < FIFOmessageNum; a++) /* 遍历当前数据集合。 */
    {
        /* 只接受读取成功的 8 字节标准数据帧。 */
        if (HAL_CAN_GetRxMessage(&hcan1, fifo_num, &rx_header, rx_data) != HAL_OK || /* 检查当前执行条件。 */
            rx_header.IDE != CAN_ID_STD || rx_header.RTR != CAN_RTR_DATA || rx_header.DLC != 8U) /* 继续更新 目标值。 */
        {
            continue; /* 丢弃格式异常报文。 */
        }

        /* 用反馈 ID 匹配注册电机。 */
        for (uint8_t i = 0; i < MotorManager.registered_count; i++) /* 遍历当前数据集合。 */
        {
            if (rx_header.StdId == MotorManager.MotorList[i].can_id_rx) /* 检查当前执行条件。 */
            {
                memcpy(MotorManager.MotorList[i].ReceiveMotorData, rx_data, 8); /* 保存原始反馈。 */
                if (MotorManager.MotorList[i].calculate != NULL) /* 检查当前执行条件。 */
                {
                    MotorManager.MotorList[i].calculate(&MotorManager.MotorList[i]); /* 立即解算反馈。 */
                }
                MotorManager.MotorList[i].last_rx_tick = HAL_GetTick(); /* 记录最近反馈时间。 */
                MotorManager.MotorList[i].online = 1U;                 /* 有效反馈确认在线。 */
                if (MotorManager.MotorList[i].MotorInf.band == DM_MOTOR_BAND) /* 检查当前执行条件。 */
                {
                    MotorManager.MotorList[i].dm_reply_pending = 0U; /* DM 一收后立即清发送标志。 */
                    SoftwareWatchdog_Disarm(SoftwareWatchdog_MotorId((uint8_t)(i + 1U))); /* 停止回复计时。 */
                }
                else /* 处理其余情况。 */
                {
                    SoftwareWatchdog_Feed(SoftwareWatchdog_MotorId((uint8_t)(i + 1U))); /* RM 连续喂狗。 */
                }
                break; /* 每帧只属于一台电机。 */
            }
        }
    }
}

void Motor_SetRegisteredCount(uint8_t count) /* 实现 Motor_SetRegisteredCount。 */
{
    if (count <= g_CanMotorNum) /* 检查当前执行条件。 */
    {
        MotorManager.registered_count = count; /* 更新 registered_count。 */
    }
}

float Motor_GetTotalAngle(can_motor_cfg motor_id) /* 实现 Motor_GetTotalAngle。 */
{
    if (motor_id < 1 || motor_id > MotorManager.registered_count || !Motor_IsOnline(motor_id)) /* 检查当前执行条件。 */
    {
        return 0.0f; /* 返回当前计算结果。 */
    }

    MotorTypeDef *motor = &MotorManager.MotorList[motor_id - 1]; /* 初始化 motor。 */
    if (motor->MotorInf.band == RM_MOTOR_BAND) /* 检查当前执行条件。 */
    {
        if ((motor_id == RM_3508_STORE_LEFT) || (motor_id == RM_3508_STORE_RIGHT)) /* 检查当前执行条件。 */
        {
            return motor->motor_data.solved_data[3]; /* 返回当前计算结果。 */
        }
        return motor->motor_data.solved_data[3] - motor->motor_data.offset_ecd_angle; /* 返回当前计算结果。 */
    }
    if (motor->MotorInf.band == DM_MOTOR_BAND) /* 检查当前执行条件。 */
    {
        /* S3519 储能适配：业务层继续使用与 RM3508 相同的度制逻辑坐标。
         * DM4310 YAW 保持原有 rad 语义，不受此分支影响。 */
        if ((motor_id == DM_3519_STRENTH_LEFT) || /* 检查当前执行条件。 */
            (motor_id == DM_3519_STRENTH_RIGHT)) /* 继续更新 目标值。 */
        {
            return RadToDegree(motor->motor_data.solved_data[0]); /* 返回当前计算结果。 */
        }
        return motor->motor_data.solved_data[0]; /* 返回当前计算结果。 */
    }
    return 0.0f; /* 返回当前计算结果。 */
}

float Motor_GetTotalAngleRad(can_motor_cfg motor_id) /* 实现 Motor_GetTotalAngleRad。 */
{
    if (motor_id < 1 || motor_id > MotorManager.registered_count || !Motor_IsOnline(motor_id)) /* 检查当前执行条件。 */
    {
        return 0.0f; /* 返回当前计算结果。 */
    }

    MotorTypeDef *motor = &MotorManager.MotorList[motor_id - 1]; /* 初始化 motor。 */
    if (motor->MotorInf.band == RM_MOTOR_BAND) /* 检查当前执行条件。 */
    {
        return DegreeToRad(motor->motor_data.solved_data[3]); /* 返回当前计算结果。 */
    }
    if (motor->MotorInf.band == DM_MOTOR_BAND) /* 检查当前执行条件。 */
    {
        return motor->motor_data.solved_data[0]; /* 返回当前计算结果。 */
    }
    return 0.0f; /* 返回当前计算结果。 */
}

float Motor_GetSpeedRPM(can_motor_cfg motor_id) /* 实现 Motor_GetSpeedRPM。 */
{
    if (motor_id < 1 || motor_id > MotorManager.registered_count || !Motor_IsOnline(motor_id)) /* 检查当前执行条件。 */
    {
        return 0.0f; /* 返回当前计算结果。 */
    }

    MotorTypeDef *motor = &MotorManager.MotorList[motor_id - 1]; /* 初始化 motor。 */
    if (motor->MotorInf.band == RM_MOTOR_BAND) /* 检查当前执行条件。 */
    {
        return motor->motor_data.solved_data[1]; /* 返回当前计算结果。 */
    }
    if (motor->MotorInf.band == DM_MOTOR_BAND) /* 检查当前执行条件。 */
    {
        return motor->motor_data.solved_data[1] * 9.5493f; /* 返回当前计算结果。 */
    }
    return 0.0f; /* 返回当前计算结果。 */
}

float Motor_GetSpeedRadS(can_motor_cfg motor_id) /* 实现 Motor_GetSpeedRadS。 */
{
    if (motor_id < 1 || motor_id > MotorManager.registered_count || !Motor_IsOnline(motor_id)) /* 检查当前执行条件。 */
    {
        return 0.0f; /* 返回当前计算结果。 */
    }

    MotorTypeDef *motor = &MotorManager.MotorList[motor_id - 1]; /* 初始化 motor。 */
    if (motor->MotorInf.band == RM_MOTOR_BAND) /* 检查当前执行条件。 */
    {
        return motor->motor_data.solved_data[4]; /* 返回当前计算结果。 */
    }
    if (motor->MotorInf.band == DM_MOTOR_BAND) /* 检查当前执行条件。 */
    {
        return motor->motor_data.solved_data[1]; /* 返回当前计算结果。 */
    }
    return 0.0f; /* 返回当前计算结果。 */
}

float Motor_GetSingleAngle(can_motor_cfg motor_id) /* 实现 Motor_GetSingleAngle。 */
{
    if (motor_id < 1 || motor_id > MotorManager.registered_count || !Motor_IsOnline(motor_id)) /* 检查当前执行条件。 */
    {
        return 0.0f; /* 返回当前计算结果。 */
    }

    MotorTypeDef *motor = &MotorManager.MotorList[motor_id - 1]; /* 初始化 motor。 */
    if (motor->MotorInf.band == RM_MOTOR_BAND) /* 检查当前执行条件。 */
    {
        return motor->motor_data.solved_data[0]; /* 返回当前计算结果。 */
    }
    if (motor->MotorInf.band == DM_MOTOR_BAND) /* 检查当前执行条件。 */
    {
        float deg = RadToDegree(motor->motor_data.solved_data[0]); /* 初始化 deg。 */
        while (deg < 0.0f) /* 条件满足时继续执行。 */
        {
            deg += 360.0f; /* 更新 deg。 */
        }
        while (deg >= 360.0f) /* 条件满足时继续执行。 */
        {
            deg -= 360.0f; /* 更新 deg。 */
        }
        return deg; /* 返回当前计算结果。 */
    }
    return 0.0f; /* 返回当前计算结果。 */
}

float Motor_GetCurrent(can_motor_cfg motor_id) /* 实现 Motor_GetCurrent。 */
{
    if (motor_id < 1 || motor_id > MotorManager.registered_count || !Motor_IsOnline(motor_id)) /* 检查当前执行条件。 */
    {
        return 0.0f; /* 返回当前计算结果。 */
    }

    MotorTypeDef *motor = &MotorManager.MotorList[motor_id - 1]; /* 初始化 motor。 */
    return motor->motor_data.solved_data[2]; /* 返回当前计算结果。 */
}

bool Motor_GetAllData(can_motor_cfg motor_id, float *data) /* 实现 Motor_GetAllData。 */
{
    if (motor_id < 1 || motor_id > MotorManager.registered_count || data == NULL || !Motor_IsOnline(motor_id)) /* 检查当前执行条件。 */
    {
        return false; /* 返回 false。 */
    }

    memcpy(data, MotorManager.MotorList[motor_id - 1].motor_data.solved_data, /* 传入下一项参数或数据。 */
           sizeof(float) * MOTOR_SOLVED_DATA_NUM); /* 调用 sizeof。 */
    return true; /* 返回 true。 */
}
