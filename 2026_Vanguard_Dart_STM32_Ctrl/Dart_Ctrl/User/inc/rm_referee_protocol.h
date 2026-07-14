/**
 * @file referee_protocol.h
 * @author kidneygood (you@domain.com)
 * @version 0.1
 * @date 2022-12-02
 *
 * @copyright Copyright (c) HNU YueLu EC 2022 all rights reserved
 *
 */

#ifndef referee_protocol_H /* 按 referee_protocol_H 选择编译分支。 */
#define referee_protocol_H /* 定义 referee_protocol_H。 */

#include <string.h>
#include <stdarg.h>
#include <stdint.h>
#include <stdio.h>
#include "main.h"

/****************************宏定义部分****************************/

#if REFEREE_RAW /* 按 REFEREE_RAW 选择编译分支。 */
#else /* 切换到备用编译分支。 */
#define REFEREE_SOF 0xA5 // 起始字节,协议固定为0xA5
#define Robot_Red 0 /* 定义 Robot_Red。 */
#define Robot_Blue 1 /* 定义 Robot_Blue。 */
#define Communicate_Data_LEN 5 // 自定义交互数据长度，该长度决定了我方发送和他方接收，自定义交互数据协议更改时只需要更改此宏定义即可

#pragma pack(1) /* 配置编译选项 pack(1)。 */

/****************************通信协议格式****************************/

/* 通信协议格式偏移，枚举类型,代替#define声明 */
typedef enum /* 开始定义数据类型。 */
{
    FRAME_HEADER_Offset = 0, /* 定义 FRAME_HEADER_Offset 枚举项。 */
    CMD_ID_Offset = 5, /* 定义 CMD_ID_Offset 枚举项。 */
    DATA_Offset = 7, /* 定义 DATA_Offset 枚举项。 */
    bf /* 继续当前语句。 */
} JudgeFrameOffset_e; /* 结束 JudgeFrameOffset_e 类型定义。 */

/* 通信协议长度 */
typedef enum /* 开始定义数据类型。 */
{
    LEN_HEADER = 5, // 帧头长
    LEN_CMDID = 2,  // 命令码长度
    LEN_TAIL = 2,   // 帧尾CRC16

    LEN_CRC8 = 4, // 帧头CRC8校验长度=帧头+数据长+包序号
} JudgeFrameLength_e; /* 结束 JudgeFrameLength_e 类型定义。 */

/****************************帧头****************************/
/****************************帧头****************************/

/* 帧头偏移 */
typedef enum /* 开始定义数据类型。 */
{
    SOF = 0,         // 起始位
    DATA_LENGTH = 1, // 帧内数据长度,根据这个来获取数据长度
    SEQ = 3,         // 包序号
    CRC8 = 4         // CRC8
} FrameHeaderOffset_e; /* 结束 FrameHeaderOffset_e 类型定义。 */

/* 帧头定义 */
typedef struct /* 开始定义数据类型。 */
{
    uint8_t SOF; /* 保存 SOF。 */
    uint16_t DataLength; /* 保存 DataLength。 */
    uint8_t Seq; /* 保存 Seq。 */
    uint8_t CRC8; /* 保存 CRC8。 */
} xFrameHeader; /* 结束 xFrameHeader 类型定义。 */

/****************************cmd_id命令码说明****************************/
/****************************cmd_id命令码说明****************************/

/* 命令码ID,用来判断接收的是什么数据 */
typedef enum /* 开始定义数据类型。 */
{
    ID_game_state = 0x0001,    // 比赛状态数据
    ID_game_result = 0x0002,   // 比赛结果数据
    ID_game_robot_HP = 0x0003, // 机器人血量数据（仅己方）
    ID_event_data = 0x0101,    // 场地事件数据
    // ID_supply_projectile_action = 0x0102,  // 场地补给站动作标识数据
    // ID_supply_projectile_booking = 0x0103, // 场地补给站预约子弹数据
    ID_referee_warning = 0x0104,         // 裁判警告数据
    ID_dart_info = 0x0105,               // 飞镖发射相关数据
    ID_robot_performance = 0x0201,       // 机器人性能体系数据
    ID_power_heat_data = 0x0202,         // 实时功率热量数据
    ID_game_robot_pos = 0x0203,          // 机器人位置数据
    ID_buff_musk = 0x0204,               // 机器人增益和底盘能量数据
    ID_robot_hurt = 0x0206,              // 伤害状态数据
    ID_shoot_data = 0x0207,              // 实时射击数据
    ID_projectile_allowance = 0x0208,    // 允许发弹量
    ID_rfid_status = 0x0209,             // RFID状态
    ID_dart_launch_status = 0x020A,      // 飞镖发射站状态
    ID_ground_robot_position = 0x020B,   // 地面机器人位置数据
    ID_radar_mark_data = 0x020C,         // 雷达标记进度
    ID_sentry_info = 0x020D,             // 哨兵信息
    ID_radar_info = 0x020E,              // 雷达信息
    ID_student_interactive = 0x0301,     // 机器人间交互数据
    ID_custom_controller_robot = 0x0302, // 自定义控制器与机器人交互
    ID_map_command = 0x0303,             // 小地图交互数据（选手端下发）
    ID_custom_client_robot = 0x0310,     // 机器人->自定义客户端
    ID_robot_custom_client = 0x0311,     // 自定义客户端->机器人
} CmdID_e; /* 结束 CmdID_e 类型定义。 */

/* 命令码数据段长,根据官方协议来定义长度，还有自定义数据长度 */
typedef enum /* 开始定义数据类型。 */
{
    LEN_game_state = 11,    // 0x0001
    LEN_game_result = 1,    // 0x0002
    LEN_game_robot_HP = 16, // 0x0003 己方机器人血量
    LEN_event_data = 4,     // 0x0101
    // LEN_supply_projectile_action = 4,			 // 0x0102
    LEN_referee_warning = 3, // 0x0104
    LEN_dart_info = 3,       // 0x0105
    // LEN_game_robot_state = 13,					    // 0x0201
    LEN_robot_performance = 13, // 0x0201
    LEN_power_heat_data = 14,   // 0x0202
    LEN_game_robot_pos = 16,    // 0x0203
    LEN_buff_musk = 8,          // 0x0204
    // LEN_aerial_robot_energy = 2,				    // 0x0205
    LEN_robot_hurt = 1,                          // 0x0206
    LEN_shoot_data = 7,                          // 0x0207
    LEN_projectile_allowance = 6,                // 0x0208
    LEN_rfid_status = 5,                         // 0x0209 (4+1)
    LEN_dart_launch_status = 6,                  // 0x020A
    LEN_ground_robot_position = 40,              // 0x020B
    LEN_radar_mark_data = 2,                     // 0x020C
    LEN_sentry_info = 6,                         // 0x020D (4+2)
    LEN_radar_info = 1,                          // 0x020E
    LEN_receive_data = 6 + Communicate_Data_LEN, // 0x0301

} JudgeDataLength_e; /* 结束 JudgeDataLength_e 类型定义。 */

/****************************接收数据的详细说明****************************/
/****************************接收数据的详细说明****************************/

/* ID: 0x0001  Byte:  11    比赛状态数据 */
typedef struct /* 开始定义数据类型。 */
{
    uint8_t game_type : 4; /* 完成本行操作。 */
    uint8_t game_progress : 4; /* 完成本行操作。 */
    uint16_t stage_remain_time; /* 保存 stage_remain_time。 */
    uint64_t SyncTimeStamp; /* 保存 SyncTimeStamp。 */
} ext_game_state_t; /* 结束 ext_game_state_t 类型定义。 */

/* ID: 0x0002  Byte:  1    比赛结果数据 */
typedef struct /* 开始定义数据类型。 */
{
    uint8_t winner; /* 保存 winner。 */
} ext_game_result_t; /* 结束 ext_game_result_t 类型定义。 */

/* ID: 0x0003  Byte:  16    比赛机器人血量数据 */
typedef struct /* 开始定义数据类型。 */
{
    uint16_t ally_1_robot_HP; // 英雄
    uint16_t ally_2_robot_HP; // 工程
    uint16_t ally_3_robot_HP; // 步兵3
    uint16_t ally_4_robot_HP; // 步兵4
    uint16_t reserved;        // 保留
    uint16_t ally_7_robot_HP; // 哨兵
    uint16_t ally_outpost_HP; // 前哨站
    uint16_t ally_base_HP;    // 基地
                              // uint16_t red_1_robot_HP;
                              // uint16_t red_2_robot_HP;
                              // uint16_t red_3_robot_HP;
                              // uint16_t red_4_robot_HP;
                              // uint16_t red_5_robot_HP;
                              // uint16_t red_7_robot_HP;
                              // uint16_t red_outpost_HP;
                              // uint16_t red_base_HP;
                              // uint16_t blue_1_robot_HP;
                              // uint16_t blue_2_robot_HP;
                              // uint16_t blue_3_robot_HP;
                              // uint16_t blue_4_robot_HP;
                              // uint16_t blue_5_robot_HP;
                              // uint16_t blue_7_robot_HP;
                              // uint16_t blue_outpost_HP;
                              // uint16_t blue_base_HP;
} ext_game_robot_HP_t; /* 结束 ext_game_robot_HP_t 类型定义。 */

/* ID: 0x0101  Byte:  4    场地事件数据 */
typedef struct /* 开始定义数据类型。 */
{
    uint32_t event_type; /* 保存 event_type。 */
} ext_event_data_t; /* 结束 ext_event_data_t 类型定义。 */

// /* ID: 0x0102  Byte:  3    场地补给站动作标识数据 */
// typedef struct
// {
//   uint8_t supply_projectile_id;
//   uint8_t supply_robot_id;
//   uint8_t supply_projectile_step;
//   uint8_t supply_projectile_num;
// } ext_supply_projectile_action_t;

/* ID: 0x0104 裁判警告数据 */
typedef struct /* 开始定义数据类型。 */
{
    uint8_t level;              // 判罚等级
    uint8_t offending_robot_id; // 违规机器人ID
    uint8_t offense_count;      // 违规次数
} ext_referee_warning_t; /* 结束 ext_referee_warning_t 类型定义。 */

/* ID: 0x0105 飞镖发射相关数据 */
typedef struct /* 开始定义数据类型。 */
{
    uint8_t dart_remaining_time; // 剩余时间
    union /* 继续当前语句。 */
    {
        uint16_t raw_dart_info; // 原始16位协议数据
        struct /* 继续当前语句。 */
        {
            uint16_t last_hit_target : 3; // bit0-2 最近命中的目标，0=无/开局默认，1=前哨站，2~5=不同基地目标
            uint16_t hit_count : 3;       // bit3-5 对方本局被击中次数，范围0~4
            uint16_t selected_target : 3; // bit6-8 当前选定击打目标，0=前哨站，1~4=不同基地目标
            uint16_t reserved : 7;        // bit9-15 保留
        } dart_info_bia_bits; /* 结束 dart_info_bia_bits 类型定义。 */
    } referee_dart_info; /* 结束 referee_dart_info 类型定义。 */
} ext_dart_info_t; /* 结束 ext_dart_info_t 类型定义。 */

// /* ID: 0X0201  Byte: 13    机器人状态数据 */
// typedef struct
// {
//   uint8_t robot_id;
//   uint8_t robot_level;
//   uint16_t current_HP;
//   uint16_t maximum_HP;
//   uint16_t shooter_barrel_cooling_value;
//   uint16_t shooter_barrel_heat_limit;
//   uint16_t chassis_power_limit;
//   uint8_t power_management_gimbal_output :1;
//   uint8_t power_management_chassis_output :1;
//   uint8_t power_management_shooter_output :1;
// } ext_game_robot_state_t;

/* ID: 0x0201 机器人性能体系数据 */
typedef struct /* 开始定义数据类型。 */
{
    uint8_t robot_id; /* 保存 robot_id。 */
    uint8_t robot_level; /* 保存 robot_level。 */
    uint16_t current_HP; /* 保存 current_HP。 */
    uint16_t maximum_HP; /* 保存 maximum_HP。 */
    uint16_t shooter_barrel_cooling_value; // 机器人枪口热量每秒冷却值

    uint16_t shooter_barrel_heat_limit; // 枪口热量上限
    uint16_t chassis_power_limit;       // 底盘功率上限

    uint8_t power_management_gimbal_output : 1; /* 完成本行操作。 */
    uint8_t power_management_chassis_output : 1; /* 完成本行操作。 */
    uint8_t power_management_shooter_output : 1; // 可用上
} ext_robot_performance_t; /* 结束 ext_robot_performance_t 类型定义。 */

/* ID: 0X0202  Byte: 16    实时功率热量数据 */
typedef struct /* 开始定义数据类型。 */
{
    // uint16_t chassis_voltage;
    // uint16_t chassis_current;
    // float chassis_power;
    uint16_t reserved0; /* 保存 reserved0。 */
    uint16_t reserved1; /* 保存 reserved1。 */
    float reserved2; /* 保存 reserved2。 */
    uint16_t buffer_energy; /* 保存 buffer_energy。 */
    uint16_t shooter_17mm_barrel_heat; /* 保存 shooter_17mm_barrel_heat。 */
    uint16_t shooter_42mm_barrel_heat; /* 保存 shooter_42mm_barrel_heat。 */
} ext_power_heat_data_t; /* 结束 ext_power_heat_data_t 类型定义。 */

/* ID: 0x0203  Byte: 16    机器人位置数据 */
typedef struct /* 开始定义数据类型。 */
{
    float x; /* 保存 x。 */
    float y; /* 保存 y。 */
    float angle; /* 保存 angle。 */
} ext_game_robot_pos_t; /* 结束 ext_game_robot_pos_t 类型定义。 */

/* ID: 0x0204  Byte:  6    机器人增益数据 */
typedef struct /* 开始定义数据类型。 */
{
    uint8_t recovery_buff; /* 保存 recovery_buff。 */
    uint16_t cooling_buff; /* 保存 cooling_buff。 */
    uint8_t defence_buff; /* 保存 defence_buff。 */
    uint8_t vulnerability_buff; /* 保存 vulnerability_buff。 */
    uint16_t attack_buff; /* 保存 attack_buff。 */
    uint8_t remaining_energy; // 剩余能量反馈（位域）
} ext_buff_musk_t; /* 结束 ext_buff_musk_t 类型定义。 */

// /* ID: 0x0205  Byte:  2    空中机器人能量状态数据 */
// typedef struct
// {
//   uint8_t airforce_status;
//   uint8_t time_remain;
// } aerial_robot_energy_t;

/* ID: 0x0206  Byte:  1    伤害状态数据 */
typedef struct /* 开始定义数据类型。 */
{
    uint8_t armor_id : 4; /* 完成本行操作。 */
    uint8_t HP_deduction_reason : 4; /* 完成本行操作。 */
    // uint8_t hurt_type :4;
} ext_robot_hurt_t; /* 结束 ext_robot_hurt_t 类型定义。 */

/* ID: 0x0207  Byte:  7    实时射击数据 */
typedef struct /* 开始定义数据类型。 */
{
    uint8_t bullet_type; /* 保存 bullet_type。 */
    uint8_t shooter_number; /* 保存 shooter_number。 */
    uint8_t launching_frequency; /* 保存 launching_frequency。 */
    float bullet_speed; /* 保存 bullet_speed。 */
    // float initial_speed;
} ext_shoot_data_t; /* 结束 ext_shoot_data_t 类型定义。 */

/* ID: 0x0208 允许发弹量 */
typedef struct /* 开始定义数据类型。 */
{
    uint16_t projectile_allowance_17mm; /* 保存 projectile_allowance_17mm。 */
    uint16_t projectile_allowance_42mm; /* 保存 projectile_allowance_42mm。 */
    uint16_t remaining_gold_coin; /* 保存 remaining_gold_coin。 */
    uint16_t projectile_allowance_fortress; // 堡垒提供储备
} ext_projectile_allowance_t; /* 结束 ext_projectile_allowance_t 类型定义。 */

/* ID: 0x0209 RFID状态 */
typedef struct /* 开始定义数据类型。 */
{
    uint32_t rfid_status;  // 低32位
    uint8_t rfid_status_2; // 高8位
} ext_rfid_status_t; /* 结束 ext_rfid_status_t 类型定义。 */

/* ID: 0x020A 飞镖发射站状态 */
typedef struct /* 开始定义数据类型。 */
{
    uint8_t dart_launch_opening_status; // 0:已开启 1:关闭 2:开启/关闭中
    uint8_t reserved; /* 保存 reserved。 */
    uint16_t target_change_time; /* 保存 target_change_time。 */
    uint16_t latest_launch_cmd_time; /* 保存 latest_launch_cmd_time。 */
} ext_dart_launch_status_t; /* 结束 ext_dart_launch_status_t 类型定义。 */

/* ID: 0x020B 地面机器人位置数据 */
typedef struct /* 开始定义数据类型。 */
{
    float hero_x; /* 保存 hero_x。 */
    float hero_y; /* 保存 hero_y。 */
    float engineer_x; /* 保存 engineer_x。 */
    float engineer_y; /* 保存 engineer_y。 */
    float standard_3_x; /* 保存 standard_3_x。 */
    float standard_3_y; /* 保存 standard_3_y。 */
    float standard_4_x; /* 保存 standard_4_x。 */
    float standard_4_y; /* 保存 standard_4_y。 */
    float reserved0; /* 保存 reserved0。 */
    float reserved1; /* 保存 reserved1。 */
} ext_ground_robot_position_t; /* 结束 ext_ground_robot_position_t 类型定义。 */

/* ID: 0x020C 雷达标记进度 */
typedef struct /* 开始定义数据类型。 */
{
    uint16_t mark_progress; // 位域详见协议
} ext_radar_mark_data_t; /* 结束 ext_radar_mark_data_t 类型定义。 */

/* ID: 0x020D 哨兵信息 */
typedef struct /* 开始定义数据类型。 */
{
    uint32_t sentry_info;   // 低位
    uint16_t sentry_info_2; // 高位
} ext_sentry_info_t; /* 结束 ext_sentry_info_t 类型定义。 */

/* ID: 0x020E 雷达信息 */
typedef struct /* 开始定义数据类型。 */
{
    uint8_t radar_info; // 位域详见协议
} ext_radar_info_t; /* 结束 ext_radar_info_t 类型定义。 */

/****************************机器人交互数据****************************/
/* 发送的内容数据段最大为 113 检测是否超出大小限制?实际上图形段不会超，数据段最多30个，也不会超*/
/* 交互数据头结构 */
typedef struct /* 开始定义数据类型。 */
{
    uint16_t data_cmd_id; // 由于存在多个内容 ID，但整个cmd_id 上行频率最大为 10Hz，请合理安排带宽。注意交互部分的上行频率
    uint16_t sender_ID; /* 保存 sender_ID。 */
    uint16_t receiver_ID; /* 保存 receiver_ID。 */
} ext_student_interactive_header_data_t; /* 结束 ext_student_interactive_header_data_t 类型定义。 */

/* 机器人id */
typedef enum /* 开始定义数据类型。 */
{
    // 红方机器人ID
    RobotID_RHero = 1, /* 定义 RobotID_RHero 枚举项。 */
    RobotID_REngineer = 2, /* 定义 RobotID_REngineer 枚举项。 */
    RobotID_RStandard1 = 3, /* 定义 RobotID_RStandard1 枚举项。 */
    RobotID_RStandard2 = 4, /* 定义 RobotID_RStandard2 枚举项。 */
    RobotID_RStandard3 = 5, /* 定义 RobotID_RStandard3 枚举项。 */
    RobotID_RAerial = 6, /* 定义 RobotID_RAerial 枚举项。 */
    RobotID_RSentry = 7, /* 定义 RobotID_RSentry 枚举项。 */
    RobotID_RRadar = 9, /* 定义 RobotID_RRadar 枚举项。 */
    // 蓝方机器人ID
    RobotID_BHero = 101, /* 定义 RobotID_BHero 枚举项。 */
    RobotID_BEngineer = 102, /* 定义 RobotID_BEngineer 枚举项。 */
    RobotID_BStandard1 = 103, /* 定义 RobotID_BStandard1 枚举项。 */
    RobotID_BStandard2 = 104, /* 定义 RobotID_BStandard2 枚举项。 */
    RobotID_BStandard3 = 105, /* 定义 RobotID_BStandard3 枚举项。 */
    RobotID_BAerial = 106, /* 定义 RobotID_BAerial 枚举项。 */
    RobotID_BSentry = 107, /* 定义 RobotID_BSentry 枚举项。 */
    RobotID_BRadar = 109, /* 定义 RobotID_BRadar 枚举项。 */
} Robot_ID_e; /* 结束 Robot_ID_e 类型定义。 */

/* 交互数据ID */
typedef enum /* 开始定义数据类型。 */
{
    UI_Data_ID_Del = 0x100, /* 定义 UI_Data_ID_Del 枚举项。 */
    UI_Data_ID_Draw1 = 0x101, /* 定义 UI_Data_ID_Draw1 枚举项。 */
    UI_Data_ID_Draw2 = 0x102, /* 定义 UI_Data_ID_Draw2 枚举项。 */
    UI_Data_ID_Draw5 = 0x103, /* 定义 UI_Data_ID_Draw5 枚举项。 */
    UI_Data_ID_Draw7 = 0x104, /* 定义 UI_Data_ID_Draw7 枚举项。 */
    UI_Data_ID_DrawChar = 0x110, /* 定义 UI_Data_ID_DrawChar 枚举项。 */

    /* 自定义交互数据部分 */
    Communicate_Data_ID = 0x0200, /* 定义 Communicate_Data_ID 枚举项。 */

} Interactive_Data_ID_e; /* 结束 Interactive_Data_ID_e 类型定义。 */
/* 交互数据长度 */
typedef enum /* 开始定义数据类型。 */
{
    Interactive_Data_LEN_Head = 6, /* 定义 Interactive_Data_LEN_Head 枚举项。 */
    UI_Operate_LEN_Del = 2, /* 定义 UI_Operate_LEN_Del 枚举项。 */
    UI_Operate_LEN_PerDraw = 15, /* 定义 UI_Operate_LEN_PerDraw 枚举项。 */
    UI_Operate_LEN_DrawChar = 15 + 30, /* 定义 UI_Operate_LEN_DrawChar 枚举项。 */

    /* 自定义交互数据部分 */
    // Communicate_Data_LEN = 5,
} Interactive_Data_Length_e; /* 结束 Interactive_Data_Length_e 类型定义。 */

/****************************自定义交互数据****************************/
/*
 学生机器人间通信 cmd_id 0x0301，内容 ID:0x0200~0x02FF
 自定义交互数据 机器人间通信：0x0301。
 发送频率：上限 10Hz
 */
// 自定义交互数据协议，可更改，更改后需要修改最上方宏定义数据长度的值
typedef struct /* 开始定义数据类型。 */
{
    uint8_t data[Communicate_Data_LEN]; // 数据段,n需要小于113
} robot_interactive_data_t; /* 结束 robot_interactive_data_t 类型定义。 */

// 机器人交互信息_发送
typedef struct /* 开始定义数据类型。 */
{
    xFrameHeader FrameHeader; /* 保存 FrameHeader。 */
    uint16_t CmdID; /* 保存 CmdID。 */
    ext_student_interactive_header_data_t datahead; /* 保存 datahead。 */
    robot_interactive_data_t Data; // 数据段
    uint16_t frametail; /* 保存 frametail。 */
} Communicate_SendData_t; /* 结束 Communicate_SendData_t 类型定义。 */
// 机器人交互信息_接收
typedef struct /* 开始定义数据类型。 */
{
    ext_student_interactive_header_data_t datahead; /* 保存 datahead。 */
    robot_interactive_data_t Data; // 数据段
} Communicate_ReceiveData_t; /* 结束 Communicate_ReceiveData_t 类型定义。 */

/****************************UI交互数据****************************/

/* 图形数据 */
typedef struct /* 开始定义数据类型。 */
{
    uint8_t graphic_name[3]; /* 保存 graphic_name。 */
    uint32_t operate_tpye : 3; /* 完成本行操作。 */
    uint32_t graphic_tpye : 3; /* 完成本行操作。 */
    uint32_t layer : 4; /* 完成本行操作。 */
    uint32_t color : 4; /* 完成本行操作。 */
    uint32_t start_angle : 9; /* 完成本行操作。 */
    uint32_t end_angle : 9; /* 完成本行操作。 */
    uint32_t width : 10; /* 完成本行操作。 */
    uint32_t start_x : 11; /* 完成本行操作。 */
    uint32_t start_y : 11; /* 完成本行操作。 */
    uint32_t radius : 10; /* 完成本行操作。 */
    uint32_t end_x : 11; /* 完成本行操作。 */
    uint32_t end_y : 11; /* 完成本行操作。 */
} Graph_Data_t; /* 结束 Graph_Data_t 类型定义。 */

typedef struct /* 开始定义数据类型。 */
{
    Graph_Data_t Graph_Control; /* 保存 Graph_Control。 */
    uint8_t show_Data[30]; /* 保存 show_Data。 */
} String_Data_t; // 打印字符串数据

/* 删除操作 */
typedef enum /* 开始定义数据类型。 */
{
    UI_Data_Del_NoOperate = 0, /* 定义 UI_Data_Del_NoOperate 枚举项。 */
    UI_Data_Del_Layer = 1, /* 定义 UI_Data_Del_Layer 枚举项。 */
    UI_Data_Del_ALL = 2, // 删除全部图层，后面的参数已经不重要了。
} UI_Delete_Operate_e; /* 结束 UI_Delete_Operate_e 类型定义。 */

/* 图形配置参数__图形操作 */
typedef enum /* 开始定义数据类型。 */
{
    UI_Graph_ADD = 1, /* 定义 UI_Graph_ADD 枚举项。 */
    UI_Graph_Change = 2, /* 定义 UI_Graph_Change 枚举项。 */
    UI_Graph_Del = 3, /* 定义 UI_Graph_Del 枚举项。 */
} UI_Graph_Operate_e; /* 结束 UI_Graph_Operate_e 类型定义。 */

/* 图形配置参数__图形类型 */
typedef enum /* 开始定义数据类型。 */
{
    UI_Graph_Line = 0,      // 直线
    UI_Graph_Rectangle = 1, // 矩形
    UI_Graph_Circle = 2,    // 整圆
    UI_Graph_Ellipse = 3,   // 椭圆
    UI_Graph_Arc = 4,       // 圆弧
    UI_Graph_Float = 5,     // 浮点型
    UI_Graph_Int = 6,       // 整形
    UI_Graph_Char = 7,      // 字符型
} UI_Graph_Type_e; /* 结束 UI_Graph_Type_e 类型定义。 */

/* 图形配置参数__图形颜色 */
typedef enum /* 开始定义数据类型。 */
{
    UI_Color_Main = 0, // 红蓝主色
    UI_Color_Yellow = 1, /* 定义 UI_Color_Yellow 枚举项。 */
    UI_Color_Green = 2, /* 定义 UI_Color_Green 枚举项。 */
    UI_Color_Orange = 3, /* 定义 UI_Color_Orange 枚举项。 */
    UI_Color_Purplish_red = 4, // 紫红色
    UI_Color_Pink = 5, /* 定义 UI_Color_Pink 枚举项。 */
    UI_Color_Cyan = 6, // 青色
    UI_Color_Black = 7, /* 定义 UI_Color_Black 枚举项。 */
    UI_Color_White = 8, /* 定义 UI_Color_White 枚举项。 */

} UI_Graph_Color_e; /* 结束 UI_Graph_Color_e 类型定义。 */

#pragma pack() /* 配置编译选项 pack()。 */
#endif /* 结束条件编译。 */
#endif /* 结束条件编译。 */
