#ifndef MOTOR_ALGROTHIM_H
#define MOTOR_ALGROTHIM_H

#include "config.h"
#include "UserLib.h"

#ifdef __cplusplus
extern "C"
{
#endif

    /* ------------------------------------------------------------------ */
    /*  梯形(斜坡)加减速位置规划器                                                */
    /* ------------------------------------------------------------------ */

    /**
     * @brief 梯形速度曲线位置规划器
     *
     * 使用方式：
     *   1. Motor_TrapPos_Init(&profile, init_pos, vmax, amax);
     *   2. profile.target_pos = new_target;          // 设置新目标
     *   3. float cmd = Motor_TrapPos_Update(&profile, dt); // 每周期调用
     *   4. 将 cmd 作为位置环的设定值输入 PID
     */
    typedef struct
    {
        /* 用户设置 */
        float target_pos;  /**< 目标位置 */
        float vmax;        /**< 最大速度（绝对值） */
        float amax;        /**< 最大加速度（绝对值） */
        float jmax;        /**< 最大加加速度（暂留，供扩展 S 曲线用） */
        float brake_gain;  /**< 制动增益 (0,1]，越小越早减速，默认1.0 */
        float arrive_zone; /**< 到达死区(°)，误差小于此值强制零速锁定 */
        float decel_zone;  /**< 线性减速区(°)，进入此范围后速度上限线性压低 */

        /* 规划器输出（只读） */
        float cmd_pos; /**< 当前规划位置 */
        float cmd_vel; /**< 当前规划速度 */
        float cmd_acc; /**< 当前规划加速度 */

        /* 内部状态 */
        uint8_t initialized; /**< 是否已初始化 */
    } MotorTrapPosProfile_t;

    /* 调试用全局变量 */
    extern volatile float g_MotorTrapPosLastOutput;

    /* ------------------------------------------------------------------ */
    /*  API                                                                 */
    /* ------------------------------------------------------------------ */

    /**
     * @brief 估算梯形/三角速度曲线从起点走到目标点所需的超时时间
     * @param start_pos   起始位置
     * @param target_pos  目标位置
     * @param vmax        最大速度，函数内部按绝对值处理
     * @param amax        最大加速度，函数内部按绝对值处理
     * @param min_ms      允许返回的最小超时值
     * @param max_ms      允许返回的最大超时值
     * @return 估算后的超时时间（毫秒）
     */
    uint32_t CalcTrapMoveTimeoutMs(float start_pos, float target_pos,
                                   float vmax, float amax,
                                   uint32_t min_ms, uint32_t max_ms);

    /**
     * @brief 初始化规划器
     * @param profile       规划器实例指针
     * @param initial_pos   初始位置（与电机当前实际位置一致）
     * @param vmax          最大速度
     * @param amax          最大加速度
     */
    void Motor_TrapPos_Init(MotorTrapPosProfile_t *profile,
                            float initial_pos, float vmax, float amax);

    /**
     * @brief 设置最大加加速度（jerk），用于未来 S 曲线扩展
     */
    void Motor_TrapPos_SetJerk(MotorTrapPosProfile_t *profile, float jmax);

    /**
     * @brief 重置规划器到指定位置（速度清零）
     */
    void Motor_TrapPos_Reset(MotorTrapPosProfile_t *profile, float pos);

    /**
     * @brief 重新同步规划位置到实际位置，保留当前速度
     * @note  用于运动中切换目标，避免 Reset 清零速度导致的位置跳变
     */
    void Motor_TrapPos_Resync(MotorTrapPosProfile_t *profile, float actual_pos);

    /**
     * @brief 每控制周期调用，更新规划位置
     * @param profile  规划器实例指针
     * @param dt       控制周期 (s)，例如 0.001f
     * @return         本周期的期望位置 cmd_pos
     */
    float Motor_TrapPos_Update(MotorTrapPosProfile_t *profile, float dt);

#ifdef __cplusplus
}
#endif

#endif /* MOTOR_ALGROTHIM_H */
