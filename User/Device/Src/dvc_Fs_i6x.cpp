/**
 * @file dvc_Fs_i6x.cpp
 * @brief FS-i6X 遥控器（SBUS 接收）
 */

#include "dvc_Fs_i6x.h"

namespace
{
constexpr uint8_t FS_I6X_FRAME_HEADER = 0x0fU;
constexpr uint8_t FS_I6X_FLAG_FRAME_LOST = 0x04U;
constexpr uint8_t FS_I6X_FLAG_FAILSAFE = 0x08U;
constexpr float FS_I6X_ROCKER_CENTER = 1023.0f;
constexpr float FS_I6X_ROCKER_RANGE = 783.0f;
constexpr uint16_t FS_I6X_SWITCH_UP_MAX = 600U;
constexpr uint16_t FS_I6X_SWITCH_DOWN_MIN = 1400U;
}

void Class_Fs_i6x::Init(UART_HandleTypeDef *huart)
{
    if (huart == nullptr)
    {
        return;
    }

    if (huart->Instance == USART1)
        UART_Manage_Object = &UART1_Manage_Object;
    else if (huart->Instance == USART2)
        UART_Manage_Object = &UART2_Manage_Object;
    else if (huart->Instance == USART3)
        UART_Manage_Object = &UART3_Manage_Object;
    else if (huart->Instance == UART4)
        UART_Manage_Object = &UART4_Manage_Object;
    else if (huart->Instance == UART5)
        UART_Manage_Object = &UART5_Manage_Object;
    else if (huart->Instance == USART6)
        UART_Manage_Object = &UART6_Manage_Object;
}

uint16_t Class_Fs_i6x::Decode_Channel(const uint8_t *frame, uint8_t channel)
{
    const uint16_t bit_index = static_cast<uint16_t>(channel) * 11U;
    const uint8_t byte_index = static_cast<uint8_t>(1U + bit_index / 8U);
    const uint8_t bit_offset = static_cast<uint8_t>(bit_index % 8U);
    const uint32_t packed = static_cast<uint32_t>(frame[byte_index]) |
                            (static_cast<uint32_t>(frame[byte_index + 1U]) << 8U) |
                            (static_cast<uint32_t>(frame[byte_index + 2U]) << 16U);
    return static_cast<uint16_t>((packed >> bit_offset) & 0x07ffU);
}

float Class_Fs_i6x::Normalize_Rocker(uint16_t raw)
{
    float value = (static_cast<float>(raw) - FS_I6X_ROCKER_CENTER) / FS_I6X_ROCKER_RANGE;
    if (value > 1.0f) value = 1.0f;
    if (value < -1.0f) value = -1.0f;
    return value;
}

Enum_FS_Switch_Status Class_Fs_i6x::Decode_Switch(uint16_t raw)
{
    if (raw < FS_I6X_SWITCH_UP_MAX) return FS_Switch_Status_UP;
    if (raw > FS_I6X_SWITCH_DOWN_MIN) return FS_Switch_Status_DOWN;
    return FS_Switch_Status_MIDDLE;
}

bool Class_Fs_i6x::Is_Valid_End_Byte(uint8_t end)
{
    return (end == 0x00U) || (end == 0x04U) || (end == 0x14U) ||
           (end == 0x24U) || (end == 0x34U) || (end == 0xfeU);
}

void Class_Fs_i6x::Judge_Updata()
{
    FS_Updata_Status = FS_Status_DisUpdata;
    for (uint8_t i = 0; i < FS_I6X_CHANNEL_COUNT; ++i)
    {
        if (Pre_UART_Rx_Data.Channel[i] != Now_UART_Rx_Data.Channel[i])
        {
            FS_Updata_Status = FS_Status_Updata;
            break;
        }
    }
}

bool Class_Fs_i6x::FS_Data_Process(const uint8_t *rx_data, uint16_t length)
{
    if ((rx_data == nullptr) || (length != FS_I6X_FRAME_LENGTH) ||
        (rx_data[0] != FS_I6X_FRAME_HEADER) || !Is_Valid_End_Byte(rx_data[24]))
    {
        return false;
    }

    const uint8_t flags = rx_data[23];
    if ((flags & (FS_I6X_FLAG_FRAME_LOST | FS_I6X_FLAG_FAILSAFE)) != 0U)
    {
        FS_Status = FS_Status_DISABLE;
        return false;
    }

    for (uint8_t i = 0; i < FS_I6X_CHANNEL_COUNT; ++i)
        Now_UART_Rx_Data.Channel[i] = Decode_Channel(rx_data, i);
    Now_UART_Rx_Data.Flags = flags;
    Now_UART_Rx_Data.End = rx_data[24];

    FS_Data.Right_X = Normalize_Rocker(Now_UART_Rx_Data.Channel[0]);
    FS_Data.Right_Y = Normalize_Rocker(Now_UART_Rx_Data.Channel[1]);
    FS_Data.Left_X = Normalize_Rocker(Now_UART_Rx_Data.Channel[2]);
    FS_Data.Left_Y = Normalize_Rocker(Now_UART_Rx_Data.Channel[3]);
    FS_Data.Yaw_Left = Normalize_Rocker(Now_UART_Rx_Data.Channel[4]);
    FS_Data.Yaw_Right = Normalize_Rocker(Now_UART_Rx_Data.Channel[5]);
    FS_Data.Switch_0 = Decode_Switch(Now_UART_Rx_Data.Channel[FS_I6X_SWA_CHANNEL]);
    FS_Data.Switch_1 = Decode_Switch(Now_UART_Rx_Data.Channel[FS_I6X_SWB_CHANNEL]);
    FS_Data.Switch_2 = Decode_Switch(Now_UART_Rx_Data.Channel[FS_I6X_SWC_CHANNEL]);
    FS_Data.Switch_3 = Decode_Switch(Now_UART_Rx_Data.Channel[FS_I6X_SWD_CHANNEL]);

    Judge_Updata();
    Pre_UART_Rx_Data = Now_UART_Rx_Data;
    ++FS_Flag;
    FS_Status = FS_Status_ENABLE;
    Offline_Count = 0;
    return true;
}

bool Class_Fs_i6x::FS_UART_RxCpltCallback(const uint8_t *rx_data, uint16_t length)
{
    return FS_Data_Process(rx_data, length);
}

void Class_Fs_i6x::TIM1msMod50_Alive_PeriodElapsedCallback()
{
    if (FS_Flag == Pre_FS_Flag)
    {
        FS_Status = FS_Status_DISABLE;
        ++Offline_Count;
    }
    else
    {
        Offline_Count = 0;
    }
    Pre_FS_Flag = FS_Flag;
}
