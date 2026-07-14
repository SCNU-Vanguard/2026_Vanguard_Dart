/*****************************************************
 * 通用 CAN 电机管理
 ****************************************************/

#include "CanMotor.h"
#include "RM_Motor.h"
#include "DM_Motor.h"
#include "config.h"
#include <stdbool.h>
#include <string.h>
#include "FreeRTOS.h"
#include "task.h"

MotorManager_t MotorManager = {0};

static uint8_t Motor_DefaultCanSend(CAN_TxHeaderTypeDef *hdr, uint8_t *data)
{
    return CAN_SendData(&hcan1, hdr, data);
}

static void Motor_DefaultDelayMs(uint32_t ms)
{
    vTaskDelay(ms);
}

static const MotorHAL_t g_DefaultMotorHAL = {
    .can_send = Motor_DefaultCanSend,
    .delay_ms = Motor_DefaultDelayMs,
};

const MotorHAL_t *g_pMotorHAL = &g_DefaultMotorHAL;

const MotorHAL_t *Motor_GetHAL(void)
{
    return g_pMotorHAL;
}

void Motor_SetHAL(const MotorHAL_t *hal)
{
    g_pMotorHAL = (hal != NULL) ? hal : &g_DefaultMotorHAL;
}

void CanRegisterMotorCfg(MotorTypeDef *ptr)
{
    if (ptr == NULL)
    {
        return;
    }

    ptr->band = ptr->MotorInf.band;
    ptr->model = ptr->MotorInf.model;

    if (ptr->band == RM_MOTOR_BAND)
    {
        if (ptr->motor_class.rm_motor_class != NULL)
        {
            ptr->gear_ratio = ptr->motor_class.rm_motor_class->gear_ratio;
            ptr->current_ratio = ptr->motor_class.rm_motor_class->current_ratio;
            ptr->max_current = ptr->motor_class.rm_motor_class->max_current;
            ptr->calculate = ptr->motor_class.rm_motor_class->calculate;
        }

        if (ptr->model == RmM3508)
        {
            ptr->g_TxHeader.StdId = g_RM_MOTOR_BIAS_ADDR_3508;
        }
        else if (ptr->model == RmM2006)
        {
            ptr->g_TxHeader.StdId = g_RM_MOTOR_BIAS_ADDR_2006;
        }
        else if (ptr->model == RmGM6020)
        {
            ptr->g_TxHeader.StdId = g_RM_MOTOR_BIAS_ADDR_6020;
        }
    }
    else if (ptr->band == DM_MOTOR_BAND)
    {
        if (ptr->motor_class.dm_motor_class != NULL)
        {
            ptr->max_current = (int16_t)ptr->motor_class.dm_motor_class->torque_max;
            ptr->calculate = ptr->motor_class.dm_motor_class->calculate;
            uint8_t idx = (uint8_t)(ptr - MotorManager.MotorList);
            can_motor_cfg motor_cfg = (can_motor_cfg)(idx + 1U);
            const DM_MotorIdConfig_t *id_cfg = DM_GetIdConfig(motor_cfg);
            if (id_cfg != NULL)
            {
                ptr->g_TxHeader.StdId = id_cfg->tx_id;
            }
        }
    }

    ptr->can_id_tx = ptr->g_TxHeader.StdId;
    ptr->can_id_rx = ptr->CAN_Rid;
    ptr->g_TxHeader.IDE = CAN_ID_STD;
    ptr->g_TxHeader.RTR = CAN_RTR_DATA;
    ptr->g_TxHeader.DLC = CtrlMotorLen;
}

void MotorRegister(void)
{
    // 换弹夹爪 M3508：目标坐标统一使用“相对零点角度”，因此限位也直接注册到 RM 电机配置里。
    RM_MotorConfig_t gripper_config = {
        .direction_bias = 0.0f,
        .position_min = LOAD3508_MIN,
        .position_max = LOAD3508_MAX,
        .position_tolerance = MOTOR_DEAD_ZONE,
        .reverse = 0U,
    };

    // 左右储能 M3508：各自注册独立的位置限位，后续 PID/保护都按各自配置生效。
    // TODO确认角度正负号
    RM_MotorConfig_t left_store_config = {
        .direction_bias = 0.0f,
        .position_min = STORE3508_LEFT_POS_MIN_DEG, // STORE3508_LEFT_POS_MIN_DEG,
        .position_max = LimitStore,                 // STORE3508_LEFT_POS_MAX_DEG,
        .position_tolerance = 0.0f,
        .reverse = 0U,
    };
    RM_MotorConfig_t right_store_config = {
        .direction_bias = 0.0f,
        .position_min = STORE3508_RIGHT_POS_MIN_DEG, // STORE3508_RIGHT_POS_MIN_DEG,
        .position_max = LimitStore,                  // STORE3508_RIGHT_POS_MAX_DEG,
        .position_tolerance = 0.0f,
        .reverse = 1U,
    };

    // 夹爪 M3508 的 S 型规划参数
    MotorTrapConfig_t gripper_trap_config = {
        .registered = 1U,
        .resync_on_target_change = LOAD_TASK_TRAP_RESET_ON_TARGET_CHANGE,
        .vmax_deg_s = LOAD_TASK_TRAP_VMAX_DEG_S,
        .amax_deg_s2 = LOAD_TASK_TRAP_AMAX_DEG_S2,
#if LOAD_TASK_TRAP_DISABLE_JERK
        .jmax_deg_s3 = 0.0f,
#else
        .jmax_deg_s3 = LOAD_TASK_TRAP_AMAX_DEG_S2 * LOAD_TASK_TRAP_JERK_FACTOR,
#endif
        .brake_gain = LOAD_TASK_TRAP_BRAKE_GAIN,
        .arrive_zone = LOAD_TASK_TRAP_ARRIVE_ZONE,
        .decel_zone = LOAD_TASK_TRAP_DECEL_ZONE,
    };

    // 左右储能 M3508 共用同一组 S 型规划；
    // 如果后面要左右分别调参，可以直接拆成 left/right 两份注册结构。
    MotorTrapConfig_t store_trap_config = {
        .registered = 1U,
        .resync_on_target_change = 1U,
        .vmax_deg_s = STORE3508_TRAP_VMAX_DEG_S,
        .amax_deg_s2 = STORE3508_TRAP_AMAX_DEG_S2,
#if STORE3508_TRAP_DISABLE_JERK
        .jmax_deg_s3 = 0.0f,
#else
        .jmax_deg_s3 = STORE3508_TRAP_AMAX_DEG_S2 * STORE3508_TRAP_JERK_FACTOR,
#endif
        .brake_gain = STORE3508_TRAP_BRAKE_GAIN,
        .arrive_zone = STORE3508_TRAP_ARRIVE_ZONE,
        .decel_zone = STORE3508_TRAP_DECEL_ZONE,
    };

    RM_M3508_Create(&MotorManager.MotorList[RM_3508_GRIPPER - 1], 1U);
    RM_Motor_SetConfig(&MotorManager.MotorList[RM_3508_GRIPPER - 1], &gripper_config);
    RM_Motor_SetCascadePID(&MotorManager.MotorList[RM_3508_GRIPPER - 1],
                           14.00f, 0.1f, 0.05f, 10.0f,
                           0.225f, 1.00f, 0.0f, 93.0f,
                           20000.0f, 0.0f, 500.0f,
                           8000.0f, 0.0f, 1500.0f);
    MotorManager.MotorList[RM_3508_GRIPPER - 1].trap_config = gripper_trap_config;
    MotorManager.MotorList[RM_3508_GRIPPER - 1].CAN_Rid = 0x201;

    RM_M2006_Create(&MotorManager.MotorList[RM_2006_TRIGGER - 1], RM_2006_TRIGGER);
    RM_Motor_SetCascadePID(&MotorManager.MotorList[RM_2006_TRIGGER - 1],
                           0.3f, 0.0f, 0.0f, 0.000f,
                           15.91f, 0.0f, 0.0f, 0.0f,
                           9000.0f, 0.0f, 0.0f,
                           6000.0f, 0.0f, 0.0f);
    MotorManager.MotorList[RM_2006_TRIGGER - 1].CAN_Rid = 0x205;

    RM_M3508_Create(&MotorManager.MotorList[RM_3508_STORE_RIGHT - 1], 2U);
    RM_Motor_SetConfig(&MotorManager.MotorList[RM_3508_STORE_RIGHT - 1], &right_store_config);
    RM_Motor_SetCascadePID(&MotorManager.MotorList[RM_3508_STORE_RIGHT - 1],
                           0.1f, 0.00f, 20.0f, 0.0f,
                           4.00f, 2.00f, 0.0f, 0.0f,
                           350.0f, 0.0f, 50.0f,
                           15000.0f, 0.0f, 4000.0f);
    // RM_Motor_SetSpeedPID(&MotorManager.MotorList[RM_3508_STORE_RIGHT - 1], PID_POSITION, 4.00f, 2.00f, 0.0f, 0.0f, 15000.0f, 0.0f, 4000.0f); // 速度2300,速度环kf为0
    MotorManager.MotorList[RM_3508_STORE_RIGHT - 1].trap_config = store_trap_config;
    MotorManager.MotorList[RM_3508_STORE_RIGHT - 1].CAN_Rid = 0x202;

    RM_M3508_Create(&MotorManager.MotorList[RM_3508_STORE_LEFT - 1], 3U);
    RM_Motor_SetConfig(&MotorManager.MotorList[RM_3508_STORE_LEFT - 1], &left_store_config);
    RM_Motor_SetCascadePID(&MotorManager.MotorList[RM_3508_STORE_LEFT - 1],
                           0.1f, 0.00f, 0.00f, 0.0f,
                           5.00f, 2.30f, 0.0f, 0.0f,
                           350.0f, 0.0f, 50.0f,
                           15000.0f, 0.0f, 4000.0f);
    // RM_Motor_SetSpeedPID(&MotorManager.MotorList[RM_3508_STORE_LEFT - 1], PID_POSITION, 5.00f, 2.30f, 0.0f, 0.0f, 15000.0f, 0.0f, 4000.0f); // 速度2300,速度环kf为0
    MotorManager.MotorList[RM_3508_STORE_LEFT - 1].trap_config = store_trap_config;
    MotorManager.MotorList[RM_3508_STORE_LEFT - 1].CAN_Rid = 0x203; // 左边是203，方向负向

    /* 双侧蓄力 3508 位置同步 PID：对 (left_pos - right_pos) 做 PID，
     * 上层以 ±correction/2 分发给左右目标，把累计角差推向 0。*/
    RM_Motor_InitStoreSyncPid(STORE3508_SYNC_PID_KP, STORE3508_SYNC_PID_KI,
                              STORE3508_SYNC_PID_KD, STORE3508_SYNC_PID_KF,
                              STORE3508_SYNC_PID_MAX_OUT, STORE3508_SYNC_PID_MIN_OUT,
                              STORE3508_SYNC_PID_MAX_IOUT);

    // Yaw轴电机 - J4310
    DM_J4310_Init(&MotorManager.MotorList[DM_4310_YAW - 1],
                  DM_GetIdConfig(DM_4310_YAW)->motor_id);
    DM_Motor_SetVelLimits(&MotorManager.MotorList[DM_4310_YAW - 1], -30.0f, 30.0f);
    DM_Motor_SetPosLimits(&MotorManager.MotorList[DM_4310_YAW - 1], -160.0f, 160.0f);
    DM_Motor_SetMITParams(&MotorManager.MotorList[DM_4310_YAW - 1], 1.4591f, 1.0f, 0.0f);
    MotorManager.MotorList[DM_4310_YAW - 1].CAN_Rid = DM_GetIdConfig(DM_4310_YAW)->rx_id;

    MotorManager.registered_count = 5U;
    for (uint8_t i = 0; i < MotorManager.registered_count; i++)
    {
        CanRegisterMotorCfg(&MotorManager.MotorList[i]);
    }
}

MotorManager_t *GetPtrMotorManager(void)
{
    return &MotorManager;
}

void MotorInit(void)
{
#if TestUse
#if DM_TestUse
    DmTestMotorSingleRegister();
#elif RM_TestUse
    RmTestMotorSingleRegister();
#endif
    for (uint8_t i = 0; i < MotorManager.registered_count; i++)
    {
        CanRegisterMotorCfg(&MotorManager.MotorList[i]);
    }
#else
    MotorRegister();
#endif

    CAN_Init(&hcan1, fifo0, 0, 0);
    CAN_Init(&hcan1, fifo1, 10, 0);
    HAL_Delay(5);
}

void CanFilterCfg(void)
{
    HAL_CAN_Stop(&hcan1);
    uint16_t id_mask_arr[2] = {0x0000, 0x0000};
    uint16_t id_arr[2] = {0x0200, 0x0010};
    FliterIdCfg_Init(&hcan1, id_arr, id_mask_arr, 0, fifo0);
    HAL_Delay(5);
    FliterIdCfg_Init(&hcan1, id_arr, id_mask_arr, 10, fifo1);
    HAL_CAN_Start(&hcan1);
    HAL_Delay(10);
}

void CAN_FIFO_CBKHANDLER(uint32_t fifo_num, uint8_t FIFOmessageNum)
{
    static uint8_t rx_data[8];
    CAN_RxHeaderTypeDef rx_header;

    for (uint8_t a = 0; a < FIFOmessageNum; a++)
    {
        HAL_CAN_GetRxMessage(&hcan1, fifo_num, &rx_header, rx_data);

        for (uint8_t i = 0; i < MotorManager.registered_count; i++)
        {
            if (rx_header.StdId == MotorManager.MotorList[i].can_id_rx)
            {
                memcpy(MotorManager.MotorList[i].ReceiveMotorData, rx_data, 8);
                if (MotorManager.MotorList[i].calculate != NULL)
                {
                    MotorManager.MotorList[i].calculate(&MotorManager.MotorList[i]);
                }
                break;
            }
        }
    }
}

void Motor_SetRegisteredCount(uint8_t count)
{
    if (count <= g_CanMotorNum)
    {
        MotorManager.registered_count = count;
    }
}

float Motor_GetTotalAngle(can_motor_cfg motor_id)
{
    if (motor_id < 1 || motor_id > MotorManager.registered_count)
    {
        return 0.0f;
    }

    MotorTypeDef *motor = &MotorManager.MotorList[motor_id - 1];
    if (motor->MotorInf.band == RM_MOTOR_BAND)
    {
        if ((motor_id == RM_3508_STORE_LEFT) || (motor_id == RM_3508_STORE_RIGHT))
        {
            return motor->motor_data.solved_data[3];
        }
        return motor->motor_data.solved_data[3] - motor->motor_data.offset_ecd_angle;
    }
    if (motor->MotorInf.band == DM_MOTOR_BAND)
    {
        return motor->motor_data.solved_data[0];
    }
    return 0.0f;
}

float Motor_GetTotalAngleRad(can_motor_cfg motor_id)
{
    if (motor_id < 1 || motor_id > MotorManager.registered_count)
    {
        return 0.0f;
    }

    MotorTypeDef *motor = &MotorManager.MotorList[motor_id - 1];
    if (motor->MotorInf.band == RM_MOTOR_BAND)
    {
        return DegreeToRad(motor->motor_data.solved_data[3]);
    }
    if (motor->MotorInf.band == DM_MOTOR_BAND)
    {
        return motor->motor_data.solved_data[0];
    }
    return 0.0f;
}

float Motor_GetSpeedRPM(can_motor_cfg motor_id)
{
    if (motor_id < 1 || motor_id > MotorManager.registered_count)
    {
        return 0.0f;
    }

    MotorTypeDef *motor = &MotorManager.MotorList[motor_id - 1];
    if (motor->MotorInf.band == RM_MOTOR_BAND)
    {
        return motor->motor_data.solved_data[1];
    }
    if (motor->MotorInf.band == DM_MOTOR_BAND)
    {
        return motor->motor_data.solved_data[1] * 9.5493f;
    }
    return 0.0f;
}

float Motor_GetSpeedRadS(can_motor_cfg motor_id)
{
    if (motor_id < 1 || motor_id > MotorManager.registered_count)
    {
        return 0.0f;
    }

    MotorTypeDef *motor = &MotorManager.MotorList[motor_id - 1];
    if (motor->MotorInf.band == RM_MOTOR_BAND)
    {
        return motor->motor_data.solved_data[4];
    }
    if (motor->MotorInf.band == DM_MOTOR_BAND)
    {
        return motor->motor_data.solved_data[1];
    }
    return 0.0f;
}

float Motor_GetSingleAngle(can_motor_cfg motor_id)
{
    if (motor_id < 1 || motor_id > MotorManager.registered_count)
    {
        return 0.0f;
    }

    MotorTypeDef *motor = &MotorManager.MotorList[motor_id - 1];
    if (motor->MotorInf.band == RM_MOTOR_BAND)
    {
        return motor->motor_data.solved_data[0];
    }
    if (motor->MotorInf.band == DM_MOTOR_BAND)
    {
        float deg = RadToDegree(motor->motor_data.solved_data[0]);
        while (deg < 0.0f)
        {
            deg += 360.0f;
        }
        while (deg >= 360.0f)
        {
            deg -= 360.0f;
        }
        return deg;
    }
    return 0.0f;
}

float Motor_GetCurrent(can_motor_cfg motor_id)
{
    if (motor_id < 1 || motor_id > MotorManager.registered_count)
    {
        return 0.0f;
    }

    MotorTypeDef *motor = &MotorManager.MotorList[motor_id - 1];
    return motor->motor_data.solved_data[2];
}

bool Motor_GetAllData(can_motor_cfg motor_id, float *data)
{
    if (motor_id < 1 || motor_id > MotorManager.registered_count || data == NULL)
    {
        return false;
    }

    memcpy(data, MotorManager.MotorList[motor_id - 1].motor_data.solved_data,
           sizeof(float) * MOTOR_SOLVED_DATA_NUM);
    return true;
}
