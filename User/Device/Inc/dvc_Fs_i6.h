/**
 * @file dvc_Fs_i6.h
 * @brief FS-i6/FS-iA6B 遥控器（iBUS 接收）
 */

#ifndef DVC_FS_I6_H
#define DVC_FS_I6_H

/* Includes ------------------------------------------------------------------*/

#include <stdint.h>
#include "drv_uart.h"

/* Exported macros -----------------------------------------------------------*/

#define FS_I6_FRAME_LENGTH 32U
#define FS_I6_RX_BUFFER_LENGTH 64U
#define FS_I6_CHANNEL_COUNT 14U
#define FS_I6_CONTROL_CHANNEL_COUNT 6U
#define FS_I6_OFFLINE_TIMEOUT_MS 100U

// 通道角色映射：数组下标从 0 开始，即 CH1 = 0、CH6 = 5。
// 原厂 i6 仅前 6 路有效；Sticks mode 在 Init 中选择，不在此处写死左右摇杆。
#define FS_I6_AILERON_CHANNEL 0U // 副翼 CH1
#define FS_I6_ELEVATOR_CHANNEL 1U // 升降 CH2
#define FS_I6_THROTTLE_CHANNEL 2U // 油门 CH3
#define FS_I6_RUDDER_CHANNEL 3U // 方向 CH4
// Aux. Channels 中分别将 CH5/CH6 分配给 SWA/SWD；修改分配时同步修改以下宏。
#define FS_I6_SWA_CHANNEL 4U
#define FS_I6_SWD_CHANNEL 5U

// 四轴共用 Rocker_Offset/Rocker_Num，方向需要修正时将对应 REVERSE 设为 1。
#define FS_I6_RIGHT_X_REVERSE 0U
#define FS_I6_RIGHT_Y_REVERSE 0U
#define FS_I6_LEFT_X_REVERSE 0U
#define FS_I6_LEFT_Y_REVERSE 0U

// 两档开关方向；默认低值为上拨，高值为下拨，反向只交换上下，不改变中间值。
#define FS_I6_SWA_REVERSE 0U
#define FS_I6_SWD_REVERSE 0U

// iBUS 无射频失控标志。须将 SWA 对应接收机通道的 Failsafe 配置为 1500（0%），
// 本项目把两档 SWA 正常情况下不会输出的中间值作为失控标记。
#define FS_I6_FAILSAFE_SWA_MIDDLE 1U

/* Exported types ------------------------------------------------------------*/

/**
 * @brief 摇杆模式，须与遥控器 Sticks mode 设置一致
 * @note Mode 1/3 为右手油门，Mode 2/4 为左手油门；机械回中结构由遥控器决定。
 */
enum Enum_FS_I6_Stick_Mode
{
    FS_I6_Stick_Mode_INVALID = 0,
    FS_I6_Stick_Mode_1,
    FS_I6_Stick_Mode_2,
    FS_I6_Stick_Mode_3,
    FS_I6_Stick_Mode_4,
};

enum Enum_FS_I6_Status
{
    FS_I6_Status_DISABLE = 0,
    FS_I6_Status_ENABLE,
};

enum Enum_FS_I6_Updata_Status
{
    FS_I6_Status_DisUpdata = 0,
    FS_I6_Status_Updata,
};

enum Enum_FS_I6_Switch_Status
{
    FS_I6_Switch_Status_UP = 0,
    FS_I6_Switch_Status_TRIG_UP_MIDDLE,
    FS_I6_Switch_Status_TRIG_MIDDLE_UP,
    FS_I6_Switch_Status_MIDDLE,
    FS_I6_Switch_Status_TRIG_MIDDLE_DOWN,
    FS_I6_Switch_Status_TRIG_DOWN_MIDDLE,
    FS_I6_Switch_Status_DOWN,
    FS_I6_Switch_Status_TRIG_UP_DOWN,
    FS_I6_Switch_Status_TRIG_DOWN_UP,
};

struct Struct_FS_I6_UART_Data
{
    uint16_t Channel[FS_I6_CHANNEL_COUNT];
    uint8_t Length;
    uint8_t Command;
    uint16_t Checksum;
};

/**
 * @brief 物理操纵件数据，四轴均归一化到 -1~1；稳定开关状态与跳变状态分离
 */
struct Struct_FS_I6_Data
{
    float Right_X;
    float Right_Y;
    float Left_X;
    float Left_Y;
    Enum_FS_I6_Switch_Status SWA;
    Enum_FS_I6_Switch_Status SWD;
    Enum_FS_I6_Switch_Status SWA_Trigger;
    Enum_FS_I6_Switch_Status SWD_Trigger;
};

class Class_FS_I6
{
public:
    // 模式必须显式指定；不自动根据接收值或机械回中位置猜测模式。
    void Init(UART_HandleTypeDef *huart, Enum_FS_I6_Stick_Mode __Stick_Mode);

    // 无效初始化返回 INVALID，且不会接受接收数据进入使能状态。
    inline Enum_FS_I6_Stick_Mode Get_Stick_Mode() const;
    inline Enum_FS_I6_Status Get_FS_I6_Status() const;
    inline Enum_FS_I6_Updata_Status Get_FS_I6_Updata_Status() const;
    inline float Get_Right_X() const;
    inline float Get_Right_Y() const;
    inline float Get_Left_X() const;
    inline float Get_Left_Y() const;
    inline Enum_FS_I6_Switch_Status Get_SWA() const;
    inline Enum_FS_I6_Switch_Status Get_SWD() const;
    inline Enum_FS_I6_Switch_Status Get_SWA_Trigger() const;
    inline Enum_FS_I6_Switch_Status Get_SWD_Trigger() const;
    // 仅用于调试原始通道，上层控制使用物理操纵件接口。
    // Channel 为从 0 开始的 iBUS 通道下标，全部 14 槽可读；原厂 i6 仅前 6 路有效。
    inline uint16_t Get_Channel_Raw(uint8_t Channel) const;
    // 原始通道的通用 1000/1500/2000 归一化，不包含物理轴标定或方向反转。
    float Get_Channel_Normalized(uint8_t Channel) const;
    inline const Struct_FS_I6_UART_Data &Get_UART_Data() const;

    bool FS_I6_UART_RxCpltCallback(const uint8_t *Rx_Data, uint16_t Length);
    void TIM1msMod50_Alive_PeriodElapsedCallback();

private:
    // 初始化相关变量
    Struct_UART_Manage_Object *UART_Manage_Object = nullptr;
    Enum_FS_I6_Stick_Mode Stick_Mode = FS_I6_Stick_Mode_INVALID;
    uint8_t Right_X_Channel = 0U;
    uint8_t Right_Y_Channel = 0U;
    uint8_t Left_X_Channel = 0U;
    uint8_t Left_Y_Channel = 0U;

    // 常量，与 DR16/VT13 一样使用统一的摇杆偏移量和刻度
    const uint16_t Min_Channel_Num = 900U;
    const uint16_t Max_Channel_Num = 2100U;
    const float Rocker_Offset = 1500.0f;
    const float Rocker_Num = 500.0f;

    // 内部变量
    Struct_FS_I6_UART_Data Now_UART_Rx_Data = {};
    Struct_FS_I6_UART_Data Pre_UART_Rx_Data = {};
    Struct_FS_I6_Data Data = {};
    uint32_t FS_I6_Flag = 0;
    uint32_t Last_Rx_Tick = 0;
    uint32_t Last_UART_Rx_Tick = 0;
    bool Rx_Valid_Flag = false;
    uint8_t UART_Rx_Buffer[FS_I6_FRAME_LENGTH] = {};
    uint8_t UART_Rx_Length = 0;
    Enum_FS_I6_Status FS_I6_Status = FS_I6_Status_DISABLE;
    Enum_FS_I6_Updata_Status FS_I6_Updata_Status = FS_I6_Status_DisUpdata;

    // 内部函数
    uint16_t Get_Channel_Value(const uint8_t *Rx_Data, uint8_t Channel);
    Enum_FS_I6_Switch_Status Get_Switch_Status(uint16_t Channel_Value, bool Reverse);
    void Judge_Switch(Enum_FS_I6_Switch_Status *Switch, Enum_FS_I6_Switch_Status Status,
                             Enum_FS_I6_Switch_Status Pre_Status);
    bool Judge_Frame(const uint8_t *Rx_Data, uint16_t Length);
    void Judge_Updata();
    void FS_I6_Data_Process();
    void FS_I6_Frame_Data_Process(const uint8_t *Rx_Data);
    bool FS_I6_UART_Data_Process(const uint8_t *Rx_Data, uint16_t Length);
};

/* Exported function definitions ---------------------------------------------*/

inline Enum_FS_I6_Stick_Mode Class_FS_I6::Get_Stick_Mode() const { return Stick_Mode; }
inline Enum_FS_I6_Status Class_FS_I6::Get_FS_I6_Status() const { return FS_I6_Status; }
inline Enum_FS_I6_Updata_Status Class_FS_I6::Get_FS_I6_Updata_Status() const { return FS_I6_Updata_Status; }
inline float Class_FS_I6::Get_Right_X() const { return Data.Right_X; }
inline float Class_FS_I6::Get_Right_Y() const { return Data.Right_Y; }
inline float Class_FS_I6::Get_Left_X() const { return Data.Left_X; }
inline float Class_FS_I6::Get_Left_Y() const { return Data.Left_Y; }
inline Enum_FS_I6_Switch_Status Class_FS_I6::Get_SWA() const { return Data.SWA; }
inline Enum_FS_I6_Switch_Status Class_FS_I6::Get_SWD() const { return Data.SWD; }
inline Enum_FS_I6_Switch_Status Class_FS_I6::Get_SWA_Trigger() const { return Data.SWA_Trigger; }
inline Enum_FS_I6_Switch_Status Class_FS_I6::Get_SWD_Trigger() const { return Data.SWD_Trigger; }
inline uint16_t Class_FS_I6::Get_Channel_Raw(uint8_t Channel) const
{
    return Channel < FS_I6_CHANNEL_COUNT ? Now_UART_Rx_Data.Channel[Channel] : 0U;
}
inline const Struct_FS_I6_UART_Data &Class_FS_I6::Get_UART_Data() const { return Now_UART_Rx_Data; }

#endif
