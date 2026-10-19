/**
 * @file dvc_gripper.h
 * @author hsl by suzumiaoya
 * @brief 旋转夹爪类驱动, 对外提供Roll角度及OPEN/CLOSE张合控制
 * @version 0.2
 * @date 2026-10-19 0.2 电流夹持版
 *
 * @copyright ZLLC 2027
 *
 */

#ifndef DVC_GRIPPER_H
#define DVC_GRIPPER_H

/* Includes ------------------------------------------------------------------*/

#include "dvc_djimotor.h"
#include "dvc_dmmotor.h"
#include "alg_fsm.h"

/* Exported macros -----------------------------------------------------------*/

/* Exported types ------------------------------------------------------------*/

// 特化类达妙电机控制方式枚举
enum Enum_DM_Motor_PID_Control_Method
{
    DM_Motor_Control_Method_PID_OPENLOOP = 0,
    DM_Motor_Control_Method_PID_OMEGA,
    DM_Motor_Control_Method_PID_POSITION,
};

// 特化类达妙电机，支持在MIT模式下进行PID控制
class Class_DM_Motor_J4310_PID : public Class_DM_Motor_J4310
{
public:
    Class_PID PID_Omega;
    Class_PID PID_Position;

    void Init(FDCAN_HandleTypeDef *hcan, Enum_DM_Motor_ID __CAN_ID, Enum_DM_Motor_PID_Control_Method __Control_Method = DM_Motor_Control_Method_PID_OPENLOOP, int32_t __Position_Offset = 0, float __Omega_Max = 20.94359f, float __Torque_Max = 10.0f);

    inline void Set_DM_Motor_Control_Method(Enum_DM_Motor_PID_Control_Method __DM_Motor_Control_Method);
    inline Enum_DM_Motor_PID_Control_Method Get_DM_Motor_Control_Method();
    
    void TIM_PID_PeriodElapsedCallback();

private:
    Enum_DM_Motor_PID_Control_Method DM_Motor_PID_Control_Method = DM_Motor_Control_Method_PID_OPENLOOP;
};


// 夹爪控制状态枚举
enum Enum_Gripper_Control_Type
{
    Gripper_Control_Type_DISABLE = 0,
    Gripper_Control_Type_ENABLE,
};

// 夹爪张合指令枚举
enum Enum_Gripper_Clamp_Command
{
    Gripper_Clamp_Command_OPEN = 0,
    Gripper_Clamp_Command_CLOSE,
};

enum Enum_Gripper_Clamp_Status
{
    Gripper_Clamp_Status_DISABLED = 0,
    Gripper_Clamp_Status_OPENING,
    Gripper_Clamp_Status_OPENED,
    Gripper_Clamp_Status_CLOSING,
    Gripper_Clamp_Status_CONTACT,
    Gripper_Clamp_Status_GRIPPED,
    Gripper_Clamp_Status_TIMEOUT,
    Gripper_Clamp_Status_NUM,
};

// 夹爪电机数据结构体
struct Struct_Gripper_Data
{
    float Now_Roll_Radian;
    float Now_Roll_Omega_Radian;
    // M2006与DM运动解耦后的相对张合角速度
    float Now_Clamp_Omega_Radian;
};

class Class_Gripper : public Class_FSM
{
public:
    void Init();

    // Roll轴目标角度与最大角速度, 单位分别为rad、rad/s
    // 17:34传动下Roll默认5rad/s对应DM轴10rad/s
    inline void Set_Target_Roll_Radian(float __Target_Roll_Radian);
    inline void Set_Target_Roll_Omega_Radian(float __Target_Roll_Omega_Radian);
    inline void Set_Gripper_Clamp_Command(Enum_Gripper_Clamp_Command __Gripper_Clamp_Command);

    inline float Get_Now_Roll_Radian();
    inline float Get_Now_Roll_Omega_Radian();
    inline float Get_Now_Clamp_Omega_Radian();

    inline void Set_Gripper_Control_Type(Enum_Gripper_Control_Type __Gripper_Control_Type);
    inline Enum_Gripper_Control_Type Get_Gripper_Control_Type();
    inline Enum_Gripper_Clamp_Status Get_Gripper_Clamp_Status();

    inline Enum_DM_Motor_Status Get_DM_Motor_Rotary_Status();
    inline Enum_DJI_Motor_Status Get_DJI_Motor_Clamp_Status();

    // 1ms定时器解算回调, 内部已调用张合状态机, 上层不应再单独调用状态机
    void TIM_Calculate_PeriodElapsedCallback();
    // 张合及夹持状态机
    void Reload_TIM_Status_PeriodElapsedCallback();
    // 成员电机CAN接收回调函数
    void DM_Motor_Rotary_CAN_RxCpltCallback(uint8_t *Rx_Data);
    void DJI_Motor_Clamp_CAN_RxCpltCallback(uint8_t *Rx_Data);
    // 存活检测函数
    void TIM_Alive_PeriodElapsedCallback();

private:
    // 成员电机子类
    Class_DM_Motor_J4310 DM_Motor_Rotary;
    Class_DJI_Motor_C610 DJI_Motor_Clamp;

    //夹爪自身控制状态
    Enum_Gripper_Control_Type Gripper_Control_Type = Gripper_Control_Type_DISABLE;

    // 控制相关变量
    float Target_Roll_Radian = 0.0f;
    float Target_Roll_Omega_Radian = 5.0f;

    // 内部连续Roll目标角度
    float Target_Roll_Total_Radian = 0.0f;

    float Target_DM_Radian = 0.0f;
    float Target_DM_Omega = 0.0f;
    float Target_DJI_Omega_Radian = 0.0f;

    // 夹持基础电流映射值
    float Clamp_Current = 0.0f;
    // 为抵消Roll运动而手动计算的M2006速度环输出
    float Follow_Omega_Current = 0.0f;
    // 夹持电流与速度环叠加后的目标输出
    float Target_DJI_Output = 0.0f;
    // M2006电机类内部实际下发的输出映射值
    float Now_DJI_Output = 0.0f;

    Struct_Gripper_Data Gripper_Data = {};

    // 张合状态机相关变量
    Enum_Gripper_Clamp_Command Gripper_Clamp_Command = Gripper_Clamp_Command_CLOSE;
    Enum_Gripper_Clamp_Command Timeout_Clamp_Command = Gripper_Clamp_Command_CLOSE;
    Enum_Gripper_Clamp_Status Gripper_Clamp_Status = Gripper_Clamp_Status_DISABLED;
    float Clamp_Hold_DM_Radian = 0.0f;
    float Contact_Start_Current = 0.0f;
    uint16_t Contact_Confirm_Count = 0;

    // 达妙电机底层反馈转换后的模角度及夹爪层连续角度
    bool DM_Radian_Initialized = false;
    float DM_Mod_Radian = 0.0f;
    float Pre_DM_Mod_Radian = 0.0f;
    float Now_DM_Radian = 0.0f;
    // 内部连续Roll角度, 用于运动学及Ozone调试
    float Now_Roll_Total_Radian = 0.0f;

    void Output();
    void Set_Clamp_Status(Enum_Gripper_Clamp_Status __Gripper_Clamp_Status);
    void Start_Clamp_Motion(Enum_Gripper_Clamp_Command __Gripper_Clamp_Command);
    void Enter_Roll_Enabled_Status(Enum_Gripper_Clamp_Status __Gripper_Clamp_Status);
    void Reset_DM_Multi_Turn_Radian();
    // 17:34外啮合机构的Roll正逆运动学及相对张合速度解算
    void Calculate_Kinematics();
};

/* Exported variables --------------------------------------------------------*/

/* Exported function declarations --------------------------------------------*/

inline void Class_Gripper::Set_Target_Roll_Radian(float __Target_Roll_Radian)
{
    if ((Gripper_Clamp_Status == Gripper_Clamp_Status_OPENED) ||
        (Gripper_Clamp_Status == Gripper_Clamp_Status_GRIPPED))
    {
        Target_Roll_Radian = Normalize_Angle_Radian_PI_to_PI(__Target_Roll_Radian);
    }
}

inline void Class_Gripper::Set_Target_Roll_Omega_Radian(float __Target_Roll_Omega_Radian)
{
    Target_Roll_Omega_Radian = __Target_Roll_Omega_Radian;
}

inline void Class_Gripper::Set_Gripper_Clamp_Command(Enum_Gripper_Clamp_Command __Gripper_Clamp_Command)
{
    Gripper_Clamp_Command = __Gripper_Clamp_Command;
}

inline float Class_Gripper::Get_Now_Roll_Radian()
{
    return Gripper_Data.Now_Roll_Radian;
}

inline float Class_Gripper::Get_Now_Roll_Omega_Radian()
{
    return Gripper_Data.Now_Roll_Omega_Radian;
}

inline float Class_Gripper::Get_Now_Clamp_Omega_Radian()
{
    return Gripper_Data.Now_Clamp_Omega_Radian;
}

inline void Class_Gripper::Set_Gripper_Control_Type(Enum_Gripper_Control_Type __Gripper_Control_Type)
{
    Gripper_Control_Type = __Gripper_Control_Type;
}

inline Enum_Gripper_Control_Type Class_Gripper::Get_Gripper_Control_Type()
{
    return Gripper_Control_Type;
}

inline Enum_Gripper_Clamp_Status Class_Gripper::Get_Gripper_Clamp_Status()
{
    return Gripper_Clamp_Status;
}

inline Enum_DM_Motor_Status Class_Gripper::Get_DM_Motor_Rotary_Status()
{
    return DM_Motor_Rotary.Get_DM_Motor_Status();
}

inline Enum_DJI_Motor_Status Class_Gripper::Get_DJI_Motor_Clamp_Status()
{
    return DJI_Motor_Clamp.Get_DJI_Motor_Status();
}

inline void Class_DM_Motor_J4310_PID::Set_DM_Motor_Control_Method(Enum_DM_Motor_PID_Control_Method __DM_Motor_Control_Method)
{
    DM_Motor_PID_Control_Method = __DM_Motor_Control_Method;
}

inline Enum_DM_Motor_PID_Control_Method Class_DM_Motor_J4310_PID::Get_DM_Motor_Control_Method()
{
    return DM_Motor_PID_Control_Method;
}

#endif

/************************ COPYRIGHT(C) NEUQ-MOSASAURUS **************************/
