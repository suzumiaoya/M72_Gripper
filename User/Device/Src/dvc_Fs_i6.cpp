/**
 * @file dvc_Fs_i6.cpp
 * @brief FS-i6/FS-iA6B 遥控器（iBUS 接收）
 */

/* Includes ------------------------------------------------------------------*/

#include "dvc_Fs_i6.h"
#include "drv_math.h"
#include <string.h>

/* Private constants ---------------------------------------------------------*/

static const uint8_t FS_I6_FRAME_LENGTH_BYTE = 0x20U;
static const uint8_t FS_I6_FRAME_COMMAND = 0x40U;
static const uint16_t FS_I6_SWITCH_UP_MAX = 1200U;
static const uint16_t FS_I6_SWITCH_DOWN_MIN = 1800U;

/* Function prototypes -------------------------------------------------------*/

/**
 * @brief 遥控器初始化，绑定 UART 并选择物理摇杆映射
 * @param huart 指定的 UART
 * @param __Stick_Mode 与遥控器 Sticks mode 一致的模式
 */
void Class_FS_I6::Init(UART_HandleTypeDef *huart, Enum_FS_I6_Stick_Mode __Stick_Mode)
{
    UART_Manage_Object = nullptr;
    Stick_Mode = FS_I6_Stick_Mode_INVALID;
    Right_X_Channel = Right_Y_Channel = Left_X_Channel = Left_Y_Channel = 0U;
    Now_UART_Rx_Data = {};
    Pre_UART_Rx_Data = {};
    Data = {};
    FS_I6_Flag = 0;
    Last_Rx_Tick = Last_UART_Rx_Tick = HAL_GetTick();
    Rx_Valid_Flag = false;
    UART_Rx_Length = 0;
    FS_I6_Status = FS_I6_Status_DISABLE;
    FS_I6_Updata_Status = FS_I6_Status_DisUpdata;
    if (huart == nullptr)
    {
        return;
    }

    // 只保留初始化时的通道边界和重复分配检查，错误配置保持失能。
    const int Channel[] = {FS_I6_AILERON_CHANNEL, FS_I6_ELEVATOR_CHANNEL,
        FS_I6_THROTTLE_CHANNEL, FS_I6_RUDDER_CHANNEL, FS_I6_SWA_CHANNEL, FS_I6_SWD_CHANNEL};
    for (uint8_t i = 0U; i < FS_I6_CONTROL_CHANNEL_COUNT; i++)
    {
        if (Channel[i] < 0 || Channel[i] >= (int)FS_I6_CONTROL_CHANNEL_COUNT)
        {
            return;
        }
        for (uint8_t j = 0U; j < i; j++)
        {
            if (Channel[i] == Channel[j])
            {
                return;
            }
        }
    }

    // 四种模式仅改变通道角色所在的物理位置，不改变原始 iBUS 通道。
    switch (__Stick_Mode)
    {
    case FS_I6_Stick_Mode_1:
        Left_X_Channel = FS_I6_RUDDER_CHANNEL;
        Left_Y_Channel = FS_I6_ELEVATOR_CHANNEL;
        Right_X_Channel = FS_I6_AILERON_CHANNEL;
        Right_Y_Channel = FS_I6_THROTTLE_CHANNEL;
        break;
    case FS_I6_Stick_Mode_2:
        Left_X_Channel = FS_I6_RUDDER_CHANNEL;
        Left_Y_Channel = FS_I6_THROTTLE_CHANNEL;
        Right_X_Channel = FS_I6_AILERON_CHANNEL;
        Right_Y_Channel = FS_I6_ELEVATOR_CHANNEL;
        break;
    case FS_I6_Stick_Mode_3:
        Left_X_Channel = FS_I6_AILERON_CHANNEL;
        Left_Y_Channel = FS_I6_ELEVATOR_CHANNEL;
        Right_X_Channel = FS_I6_RUDDER_CHANNEL;
        Right_Y_Channel = FS_I6_THROTTLE_CHANNEL;
        break;
    case FS_I6_Stick_Mode_4:
        Left_X_Channel = FS_I6_AILERON_CHANNEL;
        Left_Y_Channel = FS_I6_THROTTLE_CHANNEL;
        Right_X_Channel = FS_I6_RUDDER_CHANNEL;
        Right_Y_Channel = FS_I6_ELEVATOR_CHANNEL;
        break;
    default:
        return;
    }

    if (huart->Instance == USART1)
    {
        UART_Manage_Object = &UART1_Manage_Object;
    }
    else if (huart->Instance == USART2)
    {
        UART_Manage_Object = &UART2_Manage_Object;
    }
    else if (huart->Instance == USART3)
    {
        UART_Manage_Object = &UART3_Manage_Object;
    }
    else if (huart->Instance == UART4)
    {
        UART_Manage_Object = &UART4_Manage_Object;
    }
    else if (huart->Instance == UART5)
    {
        UART_Manage_Object = &UART5_Manage_Object;
    }
    else if (huart->Instance == USART6)
    {
        UART_Manage_Object = &UART6_Manage_Object;
    }

    if (UART_Manage_Object != nullptr)
    {
        Stick_Mode = __Stick_Mode;
    }
}

uint16_t Class_FS_I6::Get_Channel_Value(const uint8_t *Rx_Data, uint8_t Channel)
{
    const uint8_t Byte_Index = static_cast<uint8_t>(2U + Channel * 2U);
    return static_cast<uint16_t>(static_cast<uint16_t>(Rx_Data[Byte_Index]) |
                                 (static_cast<uint16_t>(Rx_Data[Byte_Index + 1U] & 0x0fU) << 8U));
}

/**
 * @brief 获取原始通道归一化值，仅用于调试，不包含物理操纵件方向反转
 */
float Class_FS_I6::Get_Channel_Normalized(uint8_t Channel) const
{
    if (Channel >= FS_I6_CHANNEL_COUNT)
    {
        return 0.0f;
    }
    float Value = (Now_UART_Rx_Data.Channel[Channel] - Rocker_Offset) / Rocker_Num;
    Math_Constrain(&Value, -1.0f, 1.0f);
    return Value;
}

Enum_FS_I6_Switch_Status Class_FS_I6::Get_Switch_Status(uint16_t Channel_Value, bool Reverse)
{
    if (Channel_Value < FS_I6_SWITCH_UP_MAX)
    {
        return Reverse ? FS_I6_Switch_Status_DOWN : FS_I6_Switch_Status_UP;
    }
    if (Channel_Value > FS_I6_SWITCH_DOWN_MIN)
    {
        return Reverse ? FS_I6_Switch_Status_UP : FS_I6_Switch_Status_DOWN;
    }
    return FS_I6_Switch_Status_MIDDLE;
}

/**
 * @brief 判断拨动开关跳变，与 DR16/VT13 一样通过指针写入处理结果
 */
void Class_FS_I6::Judge_Switch(Enum_FS_I6_Switch_Status *Switch, Enum_FS_I6_Switch_Status Status,
                              Enum_FS_I6_Switch_Status Pre_Status)
{
    *Switch = Status;
    if (Pre_Status == FS_I6_Switch_Status_UP && Status == FS_I6_Switch_Status_MIDDLE)
    {
        *Switch = FS_I6_Switch_Status_TRIG_UP_MIDDLE;
    }
    else if (Pre_Status == FS_I6_Switch_Status_MIDDLE && Status == FS_I6_Switch_Status_UP)
    {
        *Switch = FS_I6_Switch_Status_TRIG_MIDDLE_UP;
    }
    else if (Pre_Status == FS_I6_Switch_Status_MIDDLE && Status == FS_I6_Switch_Status_DOWN)
    {
        *Switch = FS_I6_Switch_Status_TRIG_MIDDLE_DOWN;
    }
    else if (Pre_Status == FS_I6_Switch_Status_DOWN && Status == FS_I6_Switch_Status_MIDDLE)
    {
        *Switch = FS_I6_Switch_Status_TRIG_DOWN_MIDDLE;
    }
    else if (Pre_Status == FS_I6_Switch_Status_UP && Status == FS_I6_Switch_Status_DOWN)
    {
        *Switch = FS_I6_Switch_Status_TRIG_UP_DOWN;
    }
    else if (Pre_Status == FS_I6_Switch_Status_DOWN && Status == FS_I6_Switch_Status_UP)
    {
        *Switch = FS_I6_Switch_Status_TRIG_DOWN_UP;
    }
}

bool Class_FS_I6::Judge_Frame(const uint8_t *Rx_Data, uint16_t Length)
{
    if ((Rx_Data == nullptr) || (Length < FS_I6_FRAME_LENGTH) ||
        (Rx_Data[0] != FS_I6_FRAME_LENGTH_BYTE) ||
        (Rx_Data[1] != FS_I6_FRAME_COMMAND))
        return false;

    uint16_t Checksum = 0xffffU;
    for (uint8_t i = 0U; i < 30U; ++i)
        Checksum = static_cast<uint16_t>(Checksum - Rx_Data[i]);

    const uint16_t Rx_Checksum = static_cast<uint16_t>(Rx_Data[30]) |
                                       (static_cast<uint16_t>(Rx_Data[31]) << 8U);
    if (Checksum != Rx_Checksum)
        return false;
    // 原厂 6 路支持端点调整；后 8 槽可为 0，不能误判为坏帧。
    for (uint8_t i = 0U; i < 6U; ++i)
    {
        const uint16_t Channel = Get_Channel_Value(Rx_Data, i);
        if (Channel < Min_Channel_Num || Channel > Max_Channel_Num)
            return false;
    }
    return true;
}

void Class_FS_I6::Judge_Updata()
{
    FS_I6_Updata_Status = FS_I6_Status_DisUpdata;
    for (uint8_t i = 0U; i < FS_I6_CHANNEL_COUNT; ++i)
    {
        if (Pre_UART_Rx_Data.Channel[i] != Now_UART_Rx_Data.Channel[i])
        {
            FS_I6_Updata_Status = FS_I6_Status_Updata;
            break;
        }
    }
}

void Class_FS_I6::FS_I6_Frame_Data_Process(const uint8_t *Rx_Data)
{
    for (uint8_t i = 0U; i < FS_I6_CHANNEL_COUNT; ++i)
        Now_UART_Rx_Data.Channel[i] = Get_Channel_Value(Rx_Data, i);
    Now_UART_Rx_Data.Length = Rx_Data[0];
    Now_UART_Rx_Data.Command = Rx_Data[1];
    Now_UART_Rx_Data.Checksum = static_cast<uint16_t>(Rx_Data[30]) |
                                (static_cast<uint16_t>(Rx_Data[31]) << 8U);

    FS_I6_Data_Process();
    Judge_Updata();
    Pre_UART_Rx_Data = Now_UART_Rx_Data;
    ++FS_I6_Flag;
    Last_Rx_Tick = HAL_GetTick();
    Rx_Valid_Flag = true;
    FS_I6_Status = FS_I6_Status_ENABLE;
#if FS_I6_FAILSAFE_SWA_MIDDLE
    if (Data.SWA == FS_I6_Switch_Status_MIDDLE)
    {
        FS_I6_Status = FS_I6_Status_DISABLE;
    }
#endif
}

/**
 * @brief 将原始通道转换为物理操纵件语义，业务层不需要知道通道编号
 */
void Class_FS_I6::FS_I6_Data_Process()
{
    const Enum_FS_I6_Switch_Status Pre_SWA_Status = Data.SWA;
    const Enum_FS_I6_Switch_Status Pre_SWD_Status = Data.SWD;
    const bool Pre_Online_Flag = FS_I6_Status == FS_I6_Status_ENABLE &&
        static_cast<uint32_t>(HAL_GetTick() - Last_Rx_Tick) < FS_I6_OFFLINE_TIMEOUT_MS;
    // 摇杆信息，按 DR16/VT13 的统一偏移量与刻度归一化。
    // 不回中轴同样读取实际位置，不在启动或松手时自动归零。
    Data.Right_X = (Now_UART_Rx_Data.Channel[Right_X_Channel] - Rocker_Offset) / Rocker_Num;
    Data.Right_Y = (Now_UART_Rx_Data.Channel[Right_Y_Channel] - Rocker_Offset) / Rocker_Num;
    Data.Left_X = (Now_UART_Rx_Data.Channel[Left_X_Channel] - Rocker_Offset) / Rocker_Num;
    Data.Left_Y = (Now_UART_Rx_Data.Channel[Left_Y_Channel] - Rocker_Offset) / Rocker_Num;

    if (FS_I6_RIGHT_X_REVERSE)
    {
        Data.Right_X = -Data.Right_X;
    }
    if (FS_I6_RIGHT_Y_REVERSE)
    {
        Data.Right_Y = -Data.Right_Y;
    }
    if (FS_I6_LEFT_X_REVERSE)
    {
        Data.Left_X = -Data.Left_X;
    }
    if (FS_I6_LEFT_Y_REVERSE)
    {
        Data.Left_Y = -Data.Left_Y;
    }
    Math_Constrain(&Data.Right_X, -1.0f, 1.0f);
    Math_Constrain(&Data.Right_Y, -1.0f, 1.0f);
    Math_Constrain(&Data.Left_X, -1.0f, 1.0f);
    Math_Constrain(&Data.Left_Y, -1.0f, 1.0f);

    Data.SWA = Get_Switch_Status(Now_UART_Rx_Data.Channel[FS_I6_SWA_CHANNEL], FS_I6_SWA_REVERSE);
    Data.SWD = Get_Switch_Status(Now_UART_Rx_Data.Channel[FS_I6_SWD_CHANNEL], FS_I6_SWD_REVERSE);
    if (Pre_Online_Flag)
    {
        Judge_Switch(&Data.SWA_Trigger, Data.SWA, Pre_SWA_Status);
        Judge_Switch(&Data.SWD_Trigger, Data.SWD, Pre_SWD_Status);
    }
    else
    {
        Data.SWA_Trigger = Data.SWA;
        Data.SWD_Trigger = Data.SWD;
    }
}

bool Class_FS_I6::FS_I6_UART_Data_Process(const uint8_t *Rx_Data, uint16_t Length)
{
    if (UART_Manage_Object == nullptr || Rx_Data == nullptr || Length == 0U)
        return false;
    const uint32_t Now_Tick = HAL_GetTick();
    if (static_cast<uint32_t>(Now_Tick - Last_UART_Rx_Tick) >= FS_I6_OFFLINE_TIMEOUT_MS)
        UART_Rx_Length = 0;
    Last_UART_Rx_Tick = Now_Tick;

    bool Rx_Flag = false;
    // IDLE/TC 的边界不等于 iBUS 帧边界：保留半帧，处理全部粘连帧，坏帧逐字节重同步。
    for (uint16_t i = 0U; i < Length; ++i)
    {
        UART_Rx_Buffer[UART_Rx_Length++] = Rx_Data[i];
        if (UART_Rx_Length < FS_I6_FRAME_LENGTH)
            continue;
        if (Judge_Frame(UART_Rx_Buffer, UART_Rx_Length))
        {
            FS_I6_Frame_Data_Process(UART_Rx_Buffer);
            UART_Rx_Length = 0;
            Rx_Flag = true;
        }
        else
        {
            memmove(UART_Rx_Buffer, UART_Rx_Buffer + 1U, FS_I6_FRAME_LENGTH - 1U);
            UART_Rx_Length = FS_I6_FRAME_LENGTH - 1U;
        }
    }
    return Rx_Flag;
}

bool Class_FS_I6::FS_I6_UART_RxCpltCallback(const uint8_t *Rx_Data, uint16_t Length)
{
    return FS_I6_UART_Data_Process(Rx_Data, Length);
}

void Class_FS_I6::TIM1msMod50_Alive_PeriodElapsedCallback()
{
    if (!Rx_Valid_Flag ||
        static_cast<uint32_t>(HAL_GetTick() - Last_Rx_Tick) >= FS_I6_OFFLINE_TIMEOUT_MS)
    {
        FS_I6_Status = FS_I6_Status_DISABLE;
        FS_I6_Updata_Status = FS_I6_Status_DisUpdata;
        Data.Right_X = Data.Right_Y = 0.0f;
        Data.Left_X = Data.Left_Y = 0.0f;
        Data.SWA_Trigger = Data.SWA;
        Data.SWD_Trigger = Data.SWD;
    }
}
