/**
 * @file dvc_gripper.h
 * @author hsl by suzumiaoya
 * @brief 旋转夹爪类驱动，给定用于组成夹爪的两个电机，通过夹爪类的接口设置目标Yaw、角速度和张合长度Length，自动计算和获取两个电机的目标角度和角速度
 * @version 0.1
 * @date 2026-09-29 0.1 测试版
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
#define LENGTH2RAD 1.0f
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

// 夹爪校准状态枚举
enum Enum_Gripper_Cali_Status
{
    Gripper_Cali_Status_UNCALIBRATED = 0,
    Gripper_Cali_Status_CALIBRATING,
    Gripper_Cali_Status_CALIBRATED,
};

// 夹爪电机数据结构体
struct Struct_Gripper_Data
{
    float Now_Roll;
    float Now_Omega;
    float Now_Length;
    float Now_Velocity;
};

class Class_Gripper : public Class_FSM
{
public:
    void Init();

    inline void Set_Target_Roll_Radian(float __Target_Roll_Radian);
    inline void Set_Target_Omega_Radian(float __Target_Omega_Radian);
    inline void Set_Target_Length(float __Target_Length);
    inline void Set_Target_Velocity(float __Target_Velocity);

    inline float Get_Now_Roll();
    inline float Get_Now_Omega();
    inline float Get_Now_Length();
    inline float Get_Now_Velocity();

    inline void Set_Gripper_Control_Type(Enum_Gripper_Control_Type __Gripper_Control_Type);
    inline Enum_Gripper_Control_Type Get_Gripper_Control_Type();

    inline Enum_DM_Motor_Status Get_DM_Motor_Rotary_Status();
    inline Enum_DJI_Motor_Status Get_DJI_Motor_Clamp_Status();

    // 定时器解算回调函数
    void TIM_Calculate_PeriodElapsedCallback();
    // 上电以及校准状态机
    void Reload_TIM_Status_PeriodElapsedCallback();
    // 成员电机CAN接收回调函数
    void DM_Motor_Rotary_CAN_RxCpltCallback(uint8_t *Rx_Data);
    void DJI_Motor_Clamp_CAN_RxCpltCallback(uint8_t *Rx_Data);
    // 存活检测函数
    void TIM_Alive_PeriodElapsedCallback();

private:
    // 成员电机子类
    Class_DM_Motor_J4310_PID DM_Motor_Rotary;
    Class_DJI_Motor_C610 DJI_Motor_Clamp;

    //夹爪自身控制状态
    Enum_Gripper_Control_Type Gripper_Control_Type = Gripper_Control_Type_DISABLE;

    // 控制相关变量
    float Target_Roll_Radian = 0.0f;
    float Target_Omega_Radian = 0.0f;
    float Target_Length = 0.0f;
    float Target_Velocity = 0.0f;

    float Target_DM_Radian = 0.0f;
    float Target_DM_Omega = 0.0f;
    float Target_DJI_Radian = 0.0f;
    float Target_DJI_Omega = 0.0f;

    Struct_Gripper_Data Gripper_Data;

    // 上电校准状态机相关变量
    Enum_Gripper_Cali_Status Gripper_Cali_Status = Gripper_Cali_Status_UNCALIBRATED;
    // 校准得到的夹持行程相对的角度偏移量
    float Clamp_Offset_Radian = 0.0f;

    void Output();
    // 夹爪运动学正逆解算，将目标Yaw角度及速度，张合长度及速度逆解算为两个电机的目标角度及速度；读取电机返回数据正解算为夹爪Yaw和张合运动
    void Calculate_Kinematics();
};

/* Exported variables --------------------------------------------------------*/

/* Exported function declarations --------------------------------------------*/

inline void Class_Gripper::Set_Target_Roll_Radian(float __Target_Roll_Radian)
{
    Target_Roll_Radian = __Target_Roll_Radian;
}

inline void Class_Gripper::Set_Target_Omega_Radian(float __Target_Omega_Radian)
{
    Target_Omega_Radian = __Target_Omega_Radian;
}

inline void Class_Gripper::Set_Target_Length(float __Target_Length)
{
    Target_Length = __Target_Length;
}

inline void Class_Gripper::Set_Target_Velocity(float __Target_Velocity)
{
    Target_Velocity = __Target_Velocity;
}

inline float Class_Gripper::Get_Now_Roll()
{
    return Gripper_Data.Now_Roll;
}

inline float Class_Gripper::Get_Now_Omega()
{
    return Gripper_Data.Now_Omega;
}

inline float Class_Gripper::Get_Now_Length()
{
    return Gripper_Data.Now_Length;
}

inline float Class_Gripper::Get_Now_Velocity()
{
    return Gripper_Data.Now_Velocity;
}

inline void Class_Gripper::Set_Gripper_Control_Type(Enum_Gripper_Control_Type __Gripper_Control_Type)
{
    Gripper_Control_Type = __Gripper_Control_Type;
}

inline Enum_Gripper_Control_Type Class_Gripper::Get_Gripper_Control_Type()
{
    return Gripper_Control_Type;
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