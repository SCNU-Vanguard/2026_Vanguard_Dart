/*****************************************************
 * 总线舵机控制模块 (优化版)
 * 适配 HX06L 等总线舵机
 * --------------------------------------------------
 * 功能:
 * - 位置控制（0-1000 对应 0°-240°）
 * - 偏差设置与保存
 * - 温度/电压监控
 * - 上电/掉电控制
 * --------------------------------------------------
 * 使用 bsp_uart.c 的统一协议发送接口
 ****************************************************/

#include <stdio.h>
#include "serial_servo.h"
#include <stdarg.h>
#include <string.h>

SerialServoControllerTypeDef serial_servo_controller;

/*====================  宏定义  ====================*/
#define GET_LOW_BYTE(A) ((uint8_t)(A))                           // 获得A的低八位
#define GET_HIGH_BYTE(A) ((uint8_t)((A) >> 8))                   // 获得A的高八位
#define BYTE_TO_HW(A, B) ((((uint16_t)(A)) << 8) | (uint8_t)(B)) // 高低八位合成十六位

// UART 配置
#define SERVO_UART_PORT BSP_UART6 // 舵机使用的UART端口

// 超时配置
#define SERVO_TIMEOUT_MS 100 // 命令超时时间(ms)
#define SERVO_RETRY_COUNT 3  // 重试次数
#define SERVO_CMD_DELAY_MS 2 // 命令间延时(ms)

/**
 * @brief 自动填充数据帧的帧头、ID、命令字段
 *
 * @param  frame      指向SerialServoCmdTypeDef类型的指针
 * @param  servo_id   舵机id号
 * @param  cmd        控制命令
 *   @arg  SERIAL_SERVO_MOVE_TIME_WRITE         设置舵机位置与运行时间
 *   @arg  SERIAL_SERVO_POS_READ                读取舵机位置
 *   @arg  SERIAL_SERVO_ID_WRITE                设置舵机ID
 *   @arg  SERIAL_SERVO_ID_READ                 读取舵机ID
 *   @arg  SERIAL_SERVO_ANGLE_OFFSET_ADJUST     设置舵机偏差
 *   @arg  SERIAL_SERVO_ANGLE_OFFSET_WRITE      保存舵机偏差
 *   @arg  SERIAL_SERVO_ANGLE_OFFSET_READ       读取舵机偏差
 *   @arg  SERIAL_SERVO_ANGLE_LIMIT_WRITE       设置舵机角度范围
 *   @arg  SERIAL_SERVO_ANGLE_LIMIT_READ        读取舵机角度范围
 *   @arg  SERIAL_SERVO_VIN_LIMIT_WRITE         设置舵机电压范围
 *   @arg  SERIAL_SERVO_VIN_LIMIT_READ          读取舵机电压范围
 *   @arg  SERIAL_SERVO_VIN_READ                读取当前舵机电压
 *   @arg  SERIAL_SERVO_TEMP_MAX_LIMIT_WRITE    设置舵机温度范围
 *   @arg  SERIAL_SERVO_TEMP_MAX_LIMIT_READ     读取舵机温度范围
 *   @arg  SERIAL_SERVO_TEMP_READ               读取当前舵机温度
 *   @arg  SERIAL_SERVO_LOAD_OR_UNLOAD_WRITE    设置舵机状态(上电/掉电)
 *   @arg  SERIAL_SERVO_LOAD_OR_UNLOAD_READ     读取舵机状态(上电/掉电)
 *   @arg  SERIAL_SERVO_MOVE_STOP               停止舵机运行
 */
static void cmd_frame_init(SerialServoCmdTypeDef *frame, int servo_id, uint8_t cmd)
{
    frame->header_1 = SERIAL_SERVO_FRAME_HEADER;
    frame->header_2 = SERIAL_SERVO_FRAME_HEADER;
    frame->servo_id = servo_id;
    frame->command = cmd;
}

/**
 * @brief 自动填充数据帧的数据长度、校验值字段
 *
 * @param  frame      指向SerialServoCmdTypeDef类型的指针
 * @param  args_num   发送的数据个数
 */

static void cmd_frame_complete(SerialServoCmdTypeDef *frame, uint8_t args_num)
{
    frame->length = args_num + 3;
    frame->args[args_num] = serial_servo_checksum((uint8_t *)frame);
    //	frame->args[args_num + 1] = 0x00;
}

/**
 * @brief 使用 bsp_uart 发送舵机数据帧
 * @param frame 数据帧指针
 * @param frame_len 帧总长度
 * @return 发送的字节数
 */
static uint16_t servo_send_frame(SerialServoCmdTypeDef *frame, uint8_t frame_len)
{
    // 设置协议为舵机模式
    UART_SetProtocol(SERVO_UART_PORT, true);

    // 发送数据帧
    return UART_Send(SERVO_UART_PORT, (uint8_t *)frame, frame_len);
}

/**
 * @brief 等待并读取舵机响应
 * @param self 控制器指针
 * @param timeout_ms 超时时间(ms)
 * @return true-收到有效响应, false-超时或无效
 */
static bool servo_wait_response(SerialServoControllerTypeDef *self, uint32_t timeout_ms)
{
    uint32_t start = HAL_GetTick();
    ServoPacket_t packet;

    while ((HAL_GetTick() - start) < timeout_ms)
    {
        if (UART_HasPacket(SERVO_UART_PORT))
        {
            if (UART_GetServoPacket(SERVO_UART_PORT, &packet))
            {
                // 复制数据到 rx_frame
                self->rx_frame.header_1 = packet.header[0];
                self->rx_frame.header_2 = packet.header[1];
                self->rx_frame.servo_id = packet.id;
                self->rx_frame.length = packet.length;
                self->rx_frame.command = packet.cmd;
                memcpy(self->rx_frame.args, packet.params, packet.param_len);

                UART_ClearPacket(SERVO_UART_PORT);
                self->it_state = SERIAL_SERVO_READ_DATA_FINISH;
                return true;
            }
        }
    }

    self->it_state = SERIAL_SERVO_READ_DATA_ERROR;
    return false;
}

/**
 * @brief 读写总线舵机数据（使用 bsp_uart 接口）
 *
 * @param  self       指向SerialServoControllerTypeDef类型的指针
 * @param  frame      指向SerialServoCmdTypeDef类型的指针
 * @param  tx_only    读写标志符
 * @return int8_t     1-已完成数据帧发送 0-已完成数据帧读取
 */
static int8_t serial_write_and_read(SerialServoControllerTypeDef *self, SerialServoCmdTypeDef *frame, bool tx_only)
{
    int8_t ret = -1;

    switch (self->it_state)
    {
    case SERIAL_SERVO_WRITE_DATA_READY:
        // 复制发送帧
        memcpy(&self->tx_frame, frame, sizeof(SerialServoCmdTypeDef));
        self->tx_only = tx_only;

        // 计算帧长度：帧头(2) + ID(1) + 长度(1) + 命令(1) + 参数(length-3) + CRC(1) = length + 3
        uint8_t frame_len = frame->length + 3;

        // 发送数据帧
        servo_send_frame(frame, frame_len);
        HAL_Delay(SERVO_CMD_DELAY_MS);

        if (tx_only)
        {
            self->it_state = SERIAL_SERVO_WRITE_DATA_READY;
            ret = 1;
        }
        else
        {
            // 等待响应
            self->it_state = SERIAL_SERVO_READ_DATA;
            if (servo_wait_response(self, SERVO_TIMEOUT_MS))
            {
                ret = 0;
            }
            else
            {
                ret = -1;
            }
            self->it_state = SERIAL_SERVO_WRITE_DATA_READY;
        }
        break;

    case SERIAL_SERVO_READ_DATA_FINISH:
        self->it_state = SERIAL_SERVO_WRITE_DATA_READY;
        ret = 0;
        break;

    case SERIAL_SERVO_READ_DATA_ERROR:
        self->it_state = SERIAL_SERVO_WRITE_DATA_READY;
        ret = -1;
        break;

    default:
        break;
    }
    return ret;
}

void serial_servo_set_id(SerialServoControllerTypeDef *self, uint8_t old_id, uint8_t new_id)
{
    SerialServoCmdTypeDef frame;
    cmd_frame_init(&frame, old_id, SERIAL_SERVO_ID_WRITE);
    frame.args[0] = new_id;
    cmd_frame_complete(&frame, 1);
    self->serial_write_and_read(self, &frame, true);
}

bool serial_servo_read_id(SerialServoControllerTypeDef *self, uint8_t servo_id, uint8_t *ret_servo_id)
{
    SerialServoCmdTypeDef frame;
    cmd_frame_init(&frame, servo_id, SERIAL_SERVO_ID_READ);
    cmd_frame_complete(&frame, 0);
    if (0 == self->serial_write_and_read(self, &frame, false))
    {
        *ret_servo_id = self->rx_frame.args[0];
        memset(&self->rx_frame, 0, sizeof(SerialServoCmdTypeDef));
        memset(&self->tx_frame, 0, sizeof(SerialServoCmdTypeDef));
        self->it_state = SERIAL_SERVO_WRITE_DATA_READY;
        return true;
    }
    return false;
}

void serial_servo_set_position(SerialServoControllerTypeDef *self, uint8_t servo_id, int position, uint16_t duration)
{
    SerialServoCmdTypeDef frame;
    position = position > 1000 ? 1000 : position;
    cmd_frame_init(&frame, servo_id, SERIAL_SERVO_MOVE_TIME_WRITE);
    frame.args[0] = GET_LOW_BYTE(position);
    frame.args[1] = GET_HIGH_BYTE(position);
    frame.args[2] = GET_LOW_BYTE(duration);
    frame.args[3] = GET_HIGH_BYTE(duration);
    cmd_frame_complete(&frame, 4);
    self->serial_write_and_read(self, &frame, true);
}

bool serial_servo_read_position(SerialServoControllerTypeDef *self, uint8_t servo_id, int *position)
{
    SerialServoCmdTypeDef frame;
    cmd_frame_init(&frame, servo_id, SERIAL_SERVO_POS_READ);
    cmd_frame_complete(&frame, 0);
    if (0 == self->serial_write_and_read(self, &frame, false))
    {
        *position = (int)(*((int16_t *)self->rx_frame.args));
        memset(&self->rx_frame, 0, sizeof(SerialServoCmdTypeDef));
        memset(&self->tx_frame, 0, sizeof(SerialServoCmdTypeDef));
        self->it_state = SERIAL_SERVO_WRITE_DATA_READY;
        return true;
    }
    return false;
}

void serial_servo_set_deviation(SerialServoControllerTypeDef *self, uint8_t servo_id, int new_deviation)
{
    SerialServoCmdTypeDef frame;
    cmd_frame_init(&frame, servo_id, SERIAL_SERVO_ANGLE_OFFSET_ADJUST);
    frame.args[0] = (uint8_t)((int8_t)new_deviation);
    cmd_frame_complete(&frame, 1);
    self->serial_write_and_read(self, &frame, true);
}

void serial_servo_save_deviation(SerialServoControllerTypeDef *self, uint8_t servo_id)
{
    SerialServoCmdTypeDef frame;
    cmd_frame_init(&frame, servo_id, SERIAL_SERVO_ANGLE_OFFSET_WRITE);
    cmd_frame_complete(&frame, 0);
    self->serial_write_and_read(self, &frame, true);
}

bool serial_servo_read_deviation(SerialServoControllerTypeDef *self, uint8_t servo_id, int8_t *deviation)
{
    SerialServoCmdTypeDef frame;
    cmd_frame_init(&frame, servo_id, SERIAL_SERVO_ANGLE_OFFSET_READ);
    cmd_frame_complete(&frame, 0);
    if (0 == self->serial_write_and_read(self, &frame, false))
    {
        *deviation = (int8_t)(self->rx_frame.args[0]);
        memset(&self->rx_frame, 0, sizeof(SerialServoCmdTypeDef));
        memset(&self->tx_frame, 0, sizeof(SerialServoCmdTypeDef));
        self->it_state = SERIAL_SERVO_WRITE_DATA_READY;
        return true;
    }
    return false;
}

void serial_servo_set_angle_limit(SerialServoControllerTypeDef *self, uint8_t servo_id, uint32_t limit_l, uint32_t limit_h)
{
    SerialServoCmdTypeDef frame;
    cmd_frame_init(&frame, servo_id, SERIAL_SERVO_ANGLE_LIMIT_WRITE);
    limit_l = limit_l > 1000 ? 1000 : limit_l;
    limit_h = limit_h > 1000 ? 1000 : limit_h;
    uint32_t real_limit_l = limit_l > limit_h ? limit_h : limit_l;
    uint32_t real_limit_h = limit_l > limit_h ? limit_l : limit_h;
    frame.args[0] = GET_LOW_BYTE(real_limit_l);
    frame.args[1] = GET_HIGH_BYTE(real_limit_l);
    frame.args[2] = GET_LOW_BYTE(real_limit_h);
    frame.args[3] = GET_HIGH_BYTE(real_limit_h);
    cmd_frame_complete(&frame, 4);
    self->serial_write_and_read(self, &frame, true);
}

bool serial_servo_read_angle_limit(SerialServoControllerTypeDef *self, uint8_t servo_id, uint16_t limit[2])
{
    SerialServoCmdTypeDef frame;
    cmd_frame_init(&frame, servo_id, SERIAL_SERVO_ANGLE_LIMIT_READ);
    cmd_frame_complete(&frame, 0);
    if (0 == self->serial_write_and_read(self, &frame, false))
    {
        limit[0] = *((uint16_t *)(&self->rx_frame.args[0]));
        limit[1] = *((uint16_t *)(&self->rx_frame.args[2]));
        memset(&self->rx_frame, 0, sizeof(SerialServoCmdTypeDef));
        memset(&self->tx_frame, 0, sizeof(SerialServoCmdTypeDef));
        self->it_state = SERIAL_SERVO_WRITE_DATA_READY;
        return true;
    }
    return false;
}

void serial_servo_set_temp_limit(SerialServoControllerTypeDef *self, uint8_t servo_id, uint32_t limit)
{
    SerialServoCmdTypeDef frame;
    cmd_frame_init(&frame, servo_id, SERIAL_SERVO_TEMP_MAX_LIMIT_WRITE);
    frame.args[0] = limit > 100 ? 100 : (uint8_t)limit;
    cmd_frame_complete(&frame, 1);
    self->serial_write_and_read(self, &frame, true);
}

bool serial_servo_read_temp_limit(SerialServoControllerTypeDef *self, uint8_t servo_id, uint8_t *limit)
{
    SerialServoCmdTypeDef frame;
    cmd_frame_init(&frame, servo_id, SERIAL_SERVO_TEMP_MAX_LIMIT_READ);
    cmd_frame_complete(&frame, 0);
    if (0 == self->serial_write_and_read(self, &frame, false))
    {
        *limit = (uint8_t)(self->rx_frame.args[0]);
        memset(&self->rx_frame, 0, sizeof(SerialServoCmdTypeDef));
        memset(&self->tx_frame, 0, sizeof(SerialServoCmdTypeDef));
        self->it_state = SERIAL_SERVO_WRITE_DATA_READY;
        return true;
    }
    return false;
}

bool serial_servo_read_temp(SerialServoControllerTypeDef *self, uint8_t servo_id, uint8_t *temp)
{
    SerialServoCmdTypeDef frame;
    cmd_frame_init(&frame, servo_id, SERIAL_SERVO_TEMP_READ);
    cmd_frame_complete(&frame, 0);
    if (0 == self->serial_write_and_read(self, &frame, false))
    {
        *temp = (uint8_t)(self->rx_frame.args[0]);
        memset(&self->rx_frame, 0, sizeof(SerialServoCmdTypeDef));
        memset(&self->tx_frame, 0, sizeof(SerialServoCmdTypeDef));
        self->it_state = SERIAL_SERVO_WRITE_DATA_READY;
        return true;
    }
    return false;
}

void serial_servo_set_vin_limit(SerialServoControllerTypeDef *self, uint8_t servo_id, uint32_t limit_l, uint32_t limit_h)
{
    SerialServoCmdTypeDef frame;
    cmd_frame_init(&frame, servo_id, SERIAL_SERVO_VIN_LIMIT_WRITE);
    limit_l = limit_l < 4500 ? 4500 : limit_l;
    limit_h = limit_h > 14000 ? 14000 : limit_h;
    uint32_t real_limit_l = limit_l > limit_h ? limit_h : limit_l;
    uint32_t real_limit_h = limit_l > limit_h ? limit_l : limit_h;
    frame.args[0] = GET_LOW_BYTE(real_limit_l);
    frame.args[1] = GET_HIGH_BYTE(real_limit_l);
    frame.args[2] = GET_LOW_BYTE(real_limit_h);
    frame.args[3] = GET_HIGH_BYTE(real_limit_h);
    cmd_frame_complete(&frame, 4);
    self->serial_write_and_read(self, &frame, true);
}

bool serial_servo_read_vin_limit(SerialServoControllerTypeDef *self, uint8_t servo_id, uint16_t limit[2])
{
    SerialServoCmdTypeDef frame;
    cmd_frame_init(&frame, servo_id, SERIAL_SERVO_VIN_LIMIT_READ);
    cmd_frame_complete(&frame, 0);
    if (0 == self->serial_write_and_read(self, &frame, false))
    {
        limit[0] = *((uint16_t *)(&self->rx_frame.args[0]));
        limit[1] = *((uint16_t *)(&self->rx_frame.args[2]));
        memset(&self->rx_frame, 0, sizeof(SerialServoCmdTypeDef));
        memset(&self->tx_frame, 0, sizeof(SerialServoCmdTypeDef));
        self->it_state = SERIAL_SERVO_WRITE_DATA_READY;
        return true;
    }
    return false;
}

bool serial_servo_read_vin(SerialServoControllerTypeDef *self, uint8_t servo_id, uint16_t *vin)
{
    SerialServoCmdTypeDef frame;
    cmd_frame_init(&frame, servo_id, SERIAL_SERVO_VIN_READ);
    cmd_frame_complete(&frame, 0);
    if (0 == self->serial_write_and_read(self, &frame, false))
    {
        *vin = ((uint32_t)*((uint16_t *)self->rx_frame.args));
        memset(&self->rx_frame, 0, sizeof(SerialServoCmdTypeDef));
        memset(&self->tx_frame, 0, sizeof(SerialServoCmdTypeDef));
        self->it_state = SERIAL_SERVO_WRITE_DATA_READY;
        return true;
    }
    return false;
}

void serial_servo_stop(SerialServoControllerTypeDef *self, uint8_t servo_id)
{
    SerialServoCmdTypeDef frame;
    cmd_frame_init(&frame, servo_id, SERIAL_SERVO_MOVE_STOP);
    cmd_frame_complete(&frame, 0);
    self->serial_write_and_read(self, &frame, true);
}

void serial_servo_load_unload(SerialServoControllerTypeDef *self, uint8_t servo_id, uint8_t load)
{
    SerialServoCmdTypeDef frame;
    cmd_frame_init(&frame, servo_id, SERIAL_SERVO_LOAD_OR_UNLOAD_WRITE);
    frame.args[0] = load;
    cmd_frame_complete(&frame, 1);
    self->serial_write_and_read(self, &frame, true);
}

bool serial_servo_read_load_unload(SerialServoControllerTypeDef *self, uint8_t servo_id, uint8_t *load_unload)
{
    SerialServoCmdTypeDef frame;
    cmd_frame_init(&frame, servo_id, SERIAL_SERVO_LOAD_OR_UNLOAD_READ);
    cmd_frame_complete(&frame, 0);
    if (0 == self->serial_write_and_read(self, &frame, false))
    {
        *load_unload = (uint8_t)(self->rx_frame.args[0]);
        memset(&self->rx_frame, 0, sizeof(SerialServoCmdTypeDef));
        memset(&self->tx_frame, 0, sizeof(SerialServoCmdTypeDef));
        self->it_state = SERIAL_SERVO_WRITE_DATA_READY;
        return true;
    }
    return false;
}

static void serial_servo_controller_object_init(SerialServoControllerTypeDef *self)
{
    self->rx_args_index = 0;
    self->it_state = SERIAL_SERVO_WRITE_DATA_READY;
    self->rx_state = SERIAL_SERVO_RECV_STARTBYTE_1;
    memset(&self->rx_frame, 0, sizeof(SerialServoCmdTypeDef));

    self->tx_only = true;
    self->tx_byte_index = 0;
    memset(&self->tx_frame, 0, sizeof(SerialServoCmdTypeDef));

    self->write_pin = NULL;
    self->serial_write_and_read = NULL;
}

void serial_servo_init(void)
{
    serial_servo_controller_object_init(&serial_servo_controller);
    serial_servo_controller.serial_write_and_read = serial_write_and_read;

    // 设置UART为舵机协议模式
    UART_SetProtocol(SERVO_UART_PORT, true);

    // 启动接收
    UART_StartRx(SERVO_UART_PORT);
}

/*====================  便捷接口函数  ====================*/

/**
 * @brief 等待舵机命令完成（带超时）
 * @param timeout_ms 超时时间(ms)
 * @return true-完成, false-超时
 */
bool serial_servo_wait_complete(uint32_t timeout_ms)
{
    uint32_t start = HAL_GetTick();
    while (serial_servo_controller.it_state != SERIAL_SERVO_WRITE_DATA_READY)
    {
        if ((HAL_GetTick() - start) > timeout_ms)
        {
            serial_servo_controller.it_state = SERIAL_SERVO_WRITE_DATA_READY;
            return false;
        }
    }
    return true;
}

/**
 * @brief 初始化并使能舵机
 * @param servo_id 舵机ID
 * @return true-成功, false-失败
 */
bool serial_servo_enable(uint8_t servo_id)
{
    serial_servo_load_unload(&serial_servo_controller, servo_id, 1);
    HAL_Delay(SERVO_CMD_DELAY_MS);
    return serial_servo_wait_complete(SERVO_TIMEOUT_MS);
}

/**
 * @brief 失能舵机（掉电）
 * @param servo_id 舵机ID
 * @return true-成功, false-失败
 */
bool serial_servo_disable(uint8_t servo_id)
{
    serial_servo_load_unload(&serial_servo_controller, servo_id, 0);
    HAL_Delay(SERVO_CMD_DELAY_MS);
    return serial_servo_wait_complete(SERVO_TIMEOUT_MS);
}

/**
 * @brief 设置舵机位置（简化接口）
 * @param servo_id 舵机ID
 * @param position 位置(0-1000)
 * @param duration 运动时间(ms)
 * @return true-成功, false-失败
 */
bool serial_servo_move(uint8_t servo_id, int position, uint16_t duration)
{
    serial_servo_set_position(&serial_servo_controller, servo_id, position, duration);
    HAL_Delay(SERVO_CMD_DELAY_MS);
    return serial_servo_wait_complete(SERVO_TIMEOUT_MS);
}

/**
 * @brief 设置舵机角度（角度制接口）
 * @param servo_id 舵机ID
 * @param angle_deg 角度(0-240°)
 * @param duration 运动时间(ms)
 * @return true-成功, false-失败
 */
bool serial_servo_move_angle(uint8_t servo_id, float angle_deg, uint16_t duration)
{
    // 限幅
    if (angle_deg < 0)
        angle_deg = 0;
    if (angle_deg > 240.0f)
        angle_deg = 240.0f;

    // 转换: 0-240° -> 0-1000
    int position = (int)(angle_deg * 1000.0f / 240.0f);
    return serial_servo_move(servo_id, position, duration);
}

/**
 * @brief 读取舵机当前角度
 * @param servo_id 舵机ID
 * @param angle_deg 返回的角度值
 * @return true-成功, false-失败
 */
bool serial_servo_read_angle(uint8_t servo_id, float *angle_deg)
{
    int position = 0;
    if (serial_servo_read_position(&serial_servo_controller, servo_id, &position))
    {
        *angle_deg = position * 240.0f / 1000.0f;
        return true;
    }
    return false;
}

/**
 * @brief 舵机扫描测试（检测通信是否正常）
 * @param servo_id 舵机ID
 * @return true-舵机响应正常, false-无响应
 */
bool serial_servo_ping(uint8_t servo_id)
{
    uint8_t ret_id = 0;
    for (int retry = 0; retry < SERVO_RETRY_COUNT; retry++)
    {
        if (serial_servo_read_id(&serial_servo_controller, servo_id, &ret_id))
        {
            return (ret_id == servo_id);
        }
        HAL_Delay(10);
    }
    return false;
}

/**
 * @brief 舵机初始化（完整版，包含上电和通信检测）
 * @param servo_id 舵机ID
 * @return true-初始化成功, false-失败
 */
bool serial_servo_init_single(uint8_t servo_id)
{
    // 1. 检测舵机通信
    if (!serial_servo_ping(servo_id))
    {
        return false;
    }

    // 2. 使能舵机（上电）
    if (!serial_servo_enable(servo_id))
    {
        return false;
    }

    return true;
}

/**
 * @brief 停止舵机运动（简化接口）
 * @param servo_id 舵机ID
 */
void serial_servo_stop_motion(uint8_t servo_id)
{
    serial_servo_stop(&serial_servo_controller, servo_id);
    HAL_Delay(SERVO_CMD_DELAY_MS);
}

/**
 * @brief 读取舵机状态信息
 * @param servo_id 舵机ID
 * @param info 返回的状态信息结构体
 * @return true-成功, false-失败
 */
bool serial_servo_get_status(uint8_t servo_id, ServoStatusInfo_t *info)
{
    if (info == NULL)
        return false;

    // 读取位置
    int pos = 0;
    if (!serial_servo_read_position(&serial_servo_controller, servo_id, &pos))
    {
        return false;
    }
    info->position = pos;
    info->angle_deg = pos * 240.0f / 1000.0f;

    // 读取电压
    uint16_t vin = 0;
    if (serial_servo_read_vin(&serial_servo_controller, servo_id, &vin))
    {
        info->voltage_mv = vin;
    }

    // 读取温度
    uint8_t temp = 0;
    if (serial_servo_read_temp(&serial_servo_controller, servo_id, &temp))
    {
        info->temperature = temp;
    }

    // 读取上电状态
    uint8_t load = 0;
    if (serial_servo_read_load_unload(&serial_servo_controller, servo_id, &load))
    {
        info->is_loaded = (load == 1);
    }

    return true;
}
