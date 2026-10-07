/**
 * @file dvc_gripper.h
 * @author hsl by suzumiaoya
 * @brief 旋转夹爪类驱动，给定用于组成夹爪的两个电机，通过夹爪类的接口设置目标Roll、角速度和张合相对角，自动计算和获取两个电机的目标角度和角速度
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

// 夹爪张合最大相对转角, rad
#define GRIPPER_CLAMP_MAX_RADIAN 1.8f
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
    Gripper_Cali_Status_FAILED,
    Gripper_Cali_Status_NUM,
};

// 夹爪电机数据结构体
struct Struct_Gripper_Data
{
    float Now_Roll_Radian;
    float Now_Roll_Omega_Radian;
    float Now_Clamp_Radian;
    float Now_Clamp_Omega_Radian;
};

class Class_Gripper : public Class_FSM
{
public:
    void Init();

    inline void Set_Target_Roll_Radian(float __Target_Roll_Radian);
    inline void Set_Target_Roll_Omega_Radian(float __Target_Roll_Omega_Radian);
    inline void Set_Target_Clamp_Radian(float __Target_Clamp_Radian);

    inline float Get_Now_Roll_Radian();
    inline float Get_Now_Roll_Omega_Radian();
    inline float Get_Now_Clamp_Radian();
    inline float Get_Now_Clamp_Omega_Radian();

    inline void Set_Gripper_Control_Type(Enum_Gripper_Control_Type __Gripper_Control_Type);
    inline Enum_Gripper_Control_Type Get_Gripper_Control_Type();
    inline Enum_Gripper_Cali_Status Get_Gripper_Cali_Status();

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
    Class_DM_Motor_J4310 DM_Motor_Rotary;
    Class_DJI_Motor_C610 DJI_Motor_Clamp;

    //夹爪自身控制状态
    Enum_Gripper_Control_Type Gripper_Control_Type = Gripper_Control_Type_DISABLE;

    // 控制相关变量
    float Target_Roll_Radian = 0.0f;
    float Target_Roll_Omega_Radian = 5.0f;
    float Target_Clamp_Radian = 0.0f;

    // 内部连续Roll目标角度
    float Target_Roll_Total_Radian = 0.0f;

    float Target_DM_Radian = 0.0f;
    float Target_DM_Omega = 0.0f;
    float Target_DJI_Omega_Radian = 0.0f;

    // 夹爪张合位置外环, 输出M2006主动开合修正速度
    Class_PID PID_Clamp;

    Struct_Gripper_Data Gripper_Data = {};

    // 上电校准状态机相关变量
    Enum_Gripper_Cali_Status Gripper_Cali_Status = Gripper_Cali_Status_UNCALIBRATED;
    // 外限位对应的两电机相对角偏移量
    float Clamp_Open_Offset_Radian = 0.0f;
    // 校准时保持的达妙电机角度
    float Cali_Hold_DM_Radian = 0.0f;
    // 校准堵转持续时间计数
    uint16_t Cali_Stall_Count = 0;

    // 达妙电机底层反馈转换后的模角度及夹爪层连续角度
    bool DM_Radian_Initialized = false;
    float DM_Mod_Radian = 0.0f;
    float Pre_DM_Mod_Radian = 0.0f;
    float Now_DM_Radian = 0.0f;
    // M2006底层反馈角度
    float Now_DJI_Radian = 0.0f;
    // 内部连续Roll角度, 用于运动学及Ozone调试
    float Now_Roll_Total_Radian = 0.0f;
    // 尚未扣除外限位偏移的张合相对角
    float Now_Clamp_Raw_Radian = 0.0f;

    void Output();
    void Set_Cali_Status(Enum_Gripper_Cali_Status __Gripper_Cali_Status);
    void Reset_DM_Multi_Turn_Radian();
    // 夹爪运动学正逆解算，将目标Roll和张合相对角逆解算为两个电机的运动；读取电机返回数据正解算为夹爪Roll和张合运动
    void Calculate_Kinematics();
};

/* Exported variables --------------------------------------------------------*/

/* Exported function declarations --------------------------------------------*/

inline void Class_Gripper::Set_Target_Roll_Radian(float __Target_Roll_Radian)
{
    Target_Roll_Radian = Normalize_Angle_Radian_PI_to_PI(__Target_Roll_Radian);
}

inline void Class_Gripper::Set_Target_Roll_Omega_Radian(float __Target_Roll_Omega_Radian)
{
    Target_Roll_Omega_Radian = __Target_Roll_Omega_Radian;
}

inline void Class_Gripper::Set_Target_Clamp_Radian(float __Target_Clamp_Radian)
{
    Math_Constrain(&__Target_Clamp_Radian, 0.0f, GRIPPER_CLAMP_MAX_RADIAN);
    Target_Clamp_Radian = __Target_Clamp_Radian;
}

inline float Class_Gripper::Get_Now_Roll_Radian()
{
    return Gripper_Data.Now_Roll_Radian;
}

inline float Class_Gripper::Get_Now_Roll_Omega_Radian()
{
    return Gripper_Data.Now_Roll_Omega_Radian;
}

inline float Class_Gripper::Get_Now_Clamp_Radian()
{
    return Gripper_Data.Now_Clamp_Radian;
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

inline Enum_Gripper_Cali_Status Class_Gripper::Get_Gripper_Cali_Status()
{
    return Gripper_Cali_Status;
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
