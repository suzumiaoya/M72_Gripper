/**
 * @file dvc_Fs_i6x.h
 * @brief FS-i6X 遥控器（SBUS 接收）
 */

#ifndef DVC_FS_I6X_H
#define DVC_FS_I6X_H

#include <stdint.h>
#include "drv_uart.h"

#define FS_I6X_FRAME_LENGTH 25U
#define FS_I6X_CHANNEL_COUNT 16U

// 当前遥控器通道配置：CH7~CH10 依次对应 SWA~SWD（数组下标 6~9）。
#define FS_I6X_SWA_CHANNEL 6U
#define FS_I6X_SWB_CHANNEL 7U
#define FS_I6X_SWC_CHANNEL 8U
#define FS_I6X_SWD_CHANNEL 9U

enum Enum_FS_Status
{
    FS_Status_DISABLE = 0,
    FS_Status_ENABLE,
};

enum Enum_FS_Updata_Status
{
    FS_Status_DisUpdata = 0,
    FS_Status_Updata,
};

enum Enum_FS_Switch_Status
{
    FS_Switch_Status_UP = 0,
    FS_Switch_Status_TRIG_UP_MIDDLE,
    FS_Switch_Status_TRIG_MIDDLE_UP,
    FS_Switch_Status_MIDDLE,
    FS_Switch_Status_TRIG_MIDDLE_DOWN,
    FS_Switch_Status_TRIG_DOWN_MIDDLE,
    FS_Switch_Status_DOWN,
    FS_Switch_Status_TRIG_UP_DOWN,
    FS_Switch_Status_TRIG_DOWN_UP,
};

struct Struct_FS_UART_Data
{
    uint16_t Channel[FS_I6X_CHANNEL_COUNT];
    uint8_t Flags;
    uint8_t End;
};

struct Struct_FS_Data
{
    float Right_X;
    float Right_Y;
    float Left_X;
    float Left_Y;
    Enum_FS_Switch_Status Switch_0;
    Enum_FS_Switch_Status Switch_1;
    Enum_FS_Switch_Status Switch_2;
    Enum_FS_Switch_Status Switch_3;
    float Yaw_Left;
    float Yaw_Right;
};

class Class_Fs_i6x
{
public:
    void Init(UART_HandleTypeDef *huart);

    inline Enum_FS_Status Get_FS_Status() const;
    inline Enum_FS_Updata_Status Get_FS_Updata_Status() const;
    inline float Get_Right_X() const;
    inline float Get_Right_Y() const;
    inline float Get_Left_X() const;
    inline float Get_Left_Y() const;
    inline Enum_FS_Switch_Status Get_Switch_0() const;
    inline Enum_FS_Switch_Status Get_Switch_1() const;
    inline Enum_FS_Switch_Status Get_Switch_2() const;
    inline Enum_FS_Switch_Status Get_Switch_3() const;
    inline Enum_FS_Switch_Status Get_SWA() const;
    inline Enum_FS_Switch_Status Get_SWD() const;
    inline float Get_Yaw_left() const;
    inline float Get_Yaw_right() const;

    inline void Set_Right_X(float value);
    inline void Set_Right_Y(float value);
    inline void Set_Left_X(float value);
    inline void Set_Left_Y(float value);
    inline void Set_Switch_0(Enum_FS_Switch_Status value);
    inline void Set_Switch_1(Enum_FS_Switch_Status value);
    inline void Set_Switch_2(Enum_FS_Switch_Status value);
    inline void Set_Switch_3(Enum_FS_Switch_Status value);
    inline void Set_Yaw_left(float value);
    inline void Set_Yaw_right(float value);
    inline void Set_FS_Status(Enum_FS_Status status);

    bool FS_UART_RxCpltCallback(const uint8_t *rx_data, uint16_t length);
    void TIM1msMod50_Alive_PeriodElapsedCallback();

private:
    Struct_UART_Manage_Object *UART_Manage_Object = nullptr;
    Struct_FS_UART_Data Now_UART_Rx_Data = {};
    Struct_FS_UART_Data Pre_UART_Rx_Data = {};
    Struct_FS_Data FS_Data = {};
    uint32_t FS_Flag = 0;
    uint32_t Pre_FS_Flag = 0;
    uint16_t Offline_Count = 0;
    Enum_FS_Status FS_Status = FS_Status_DISABLE;
    Enum_FS_Updata_Status FS_Updata_Status = FS_Status_DisUpdata;

    static uint16_t Decode_Channel(const uint8_t *frame, uint8_t channel);
    static float Normalize_Rocker(uint16_t raw);
    static Enum_FS_Switch_Status Decode_Switch(uint16_t raw);
    static bool Is_Valid_End_Byte(uint8_t end);
    void Judge_Updata();
    bool FS_Data_Process(const uint8_t *rx_data, uint16_t length);
};

inline Enum_FS_Status Class_Fs_i6x::Get_FS_Status() const { return FS_Status; }
inline Enum_FS_Updata_Status Class_Fs_i6x::Get_FS_Updata_Status() const { return FS_Updata_Status; }
inline float Class_Fs_i6x::Get_Right_X() const { return FS_Data.Right_X; }
inline float Class_Fs_i6x::Get_Right_Y() const { return FS_Data.Right_Y; }
inline float Class_Fs_i6x::Get_Left_X() const { return FS_Data.Left_X; }
inline float Class_Fs_i6x::Get_Left_Y() const { return FS_Data.Left_Y; }
inline Enum_FS_Switch_Status Class_Fs_i6x::Get_Switch_0() const { return FS_Data.Switch_0; }
inline Enum_FS_Switch_Status Class_Fs_i6x::Get_Switch_1() const { return FS_Data.Switch_1; }
inline Enum_FS_Switch_Status Class_Fs_i6x::Get_Switch_2() const { return FS_Data.Switch_2; }
inline Enum_FS_Switch_Status Class_Fs_i6x::Get_Switch_3() const { return FS_Data.Switch_3; }
inline Enum_FS_Switch_Status Class_Fs_i6x::Get_SWA() const { return FS_Data.Switch_0; }
inline Enum_FS_Switch_Status Class_Fs_i6x::Get_SWD() const { return FS_Data.Switch_3; }
inline float Class_Fs_i6x::Get_Yaw_left() const { return FS_Data.Yaw_Left; }
inline float Class_Fs_i6x::Get_Yaw_right() const { return FS_Data.Yaw_Right; }
inline void Class_Fs_i6x::Set_Right_X(float value) { FS_Data.Right_X = value; }
inline void Class_Fs_i6x::Set_Right_Y(float value) { FS_Data.Right_Y = value; }
inline void Class_Fs_i6x::Set_Left_X(float value) { FS_Data.Left_X = value; }
inline void Class_Fs_i6x::Set_Left_Y(float value) { FS_Data.Left_Y = value; }
inline void Class_Fs_i6x::Set_Switch_0(Enum_FS_Switch_Status value) { FS_Data.Switch_0 = value; }
inline void Class_Fs_i6x::Set_Switch_1(Enum_FS_Switch_Status value) { FS_Data.Switch_1 = value; }
inline void Class_Fs_i6x::Set_Switch_2(Enum_FS_Switch_Status value) { FS_Data.Switch_2 = value; }
inline void Class_Fs_i6x::Set_Switch_3(Enum_FS_Switch_Status value) { FS_Data.Switch_3 = value; }
inline void Class_Fs_i6x::Set_Yaw_left(float value) { FS_Data.Yaw_Left = value; }
inline void Class_Fs_i6x::Set_Yaw_right(float value) { FS_Data.Yaw_Right = value; }
inline void Class_Fs_i6x::Set_FS_Status(Enum_FS_Status status) { FS_Status = status; }

#endif
