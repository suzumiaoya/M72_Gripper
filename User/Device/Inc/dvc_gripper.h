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

// 夹爪特化类达妙电机
class Class_Gripper_DM_Motor : public Class_DM_Motor_J4310
{
public:
    void CAN_RxCpltCallback(uint8_t *Rx_Data);
    void TIM_Alive_PeriodElapsedCallback();
    void Reset_Multi_Turn_Radian();

    inline float Get_Now_Multi_Turn_Radian();
    inline bool Get_Radian_Initialized();

private:
    bool DM_Radian_Initialized = false;
    // 离线期间收到的新反馈，留到在线检测确认后再处理；重置时一并清除
    bool Feedback_Pending = false;
    float DM_Mod_Radian = 0.0f;
    float Pre_DM_Mod_Radian = 0.0f;
    float Now_DM_Radian = 0.0f;

    // 基类驱动解包后进行二次处理；因为基类的Data_Process非虚函数所以没有采用重写的方式实现
    void Multi_Turn_Radian_Data_Process();
};

enum Enum_Gripper_DJI_Control_Method
{
    Gripper_DJI_Control_Method_CURRENT = 0,
    Gripper_DJI_Control_Method_OMEGA,
    Gripper_DJI_Control_Method_CLAMP_FOLLOW,
};

// 夹爪特化M2006子类，实现电流与速度闭环
class Class_Gripper_DJI_Motor : public Class_DJI_Motor_C610
{
public:
    // 直接电流、普通速度环、夹持电流叠加随动速度环，目标速度均为电机输出轴rad/s
    void Set_Current_Control(float __Target_Current);
    void Set_Omega_Control(float __Target_Omega_Radian);
    void Set_Clamp_Follow_Control(float __Clamp_Current, float __Target_Omega_Radian);
    // 保留原有行为，仅清除速度环积分
    void Reset_Omega_PID();
    // 每控制周期调用一次，完成计算并写入CAN发送缓存
    void TIM_Control_PeriodElapsedCallback();

    inline float Get_Clamp_Current();
    inline float Get_Follow_Omega_Current();
    inline float Get_Target_Output();

private:
    Enum_Gripper_DJI_Control_Method Gripper_DJI_Control_Method = Gripper_DJI_Control_Method_CURRENT;
    float Target_Current = 0.0f;
    // 仅夹持随动模式使用的基础电流与随动电流分量
    float Clamp_Current = 0.0f;
    float Follow_Omega_Current = 0.0f;
    // 当前模式计算得到的电流指令，Out为实际下发值
    float Target_Output = 0.0f;
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

// 夹爪张合状态机枚举
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
    Class_Gripper_DM_Motor DM_Motor_Rotary;
    Class_Gripper_DJI_Motor DJI_Motor_Clamp;

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
    // 夹持模式中M2006随动速度环的电流输出反馈
    float Follow_Omega_Current = 0.0f;
    // M2006当前控制模式计算得到的电流指令反馈
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

    // 旋转齿轮的连续角度及角速度，已包含DM到齿轮的传动比和方向
    float Now_Roll_Total_Radian = 0.0f;
    float Now_Roll_Gear_Omega_Radian = 0.0f;

    void Output();
    void Set_Clamp_Status(Enum_Gripper_Clamp_Status __Gripper_Clamp_Status);
    void Start_Clamp_Motion(Enum_Gripper_Clamp_Command __Gripper_Clamp_Command);
    void Enter_Roll_Enabled_Status(Enum_Gripper_Clamp_Status __Gripper_Clamp_Status);
    // 外啮合机构的Roll正逆运动学及相对张合速度解算
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

inline float Class_Gripper_DM_Motor::Get_Now_Multi_Turn_Radian()
{
    return Now_DM_Radian;
}

inline bool Class_Gripper_DM_Motor::Get_Radian_Initialized()
{
    return DM_Radian_Initialized;
}

inline float Class_Gripper_DJI_Motor::Get_Clamp_Current()
{
    return Clamp_Current;
}

inline float Class_Gripper_DJI_Motor::Get_Follow_Omega_Current()
{
    return Follow_Omega_Current;
}

inline float Class_Gripper_DJI_Motor::Get_Target_Output()
{
    return Target_Output;
}

#endif

/************************ COPYRIGHT(C) NEUQ-MOSASAURUS **************************/
