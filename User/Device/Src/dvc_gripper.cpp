/**
 * @file dvc_gripper.cpp
 * @author hsl by suzumiaoya
 * @brief 夹爪类驱动
 * @version 0.1
 * @date 2026-09-29 0.1 测试版
 *
 * @copyright ZLLC 2027
 *
 */

/* Includes ------------------------------------------------------------------*/

#include "dvc_gripper.h"

/* Private macros ------------------------------------------------------------*/

/* Private types -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

/* Private function declarations ---------------------------------------------*/

/* Function prototypes -------------------------------------------------------*/

void Class_DM_Motor_J4310_PID::Init(FDCAN_HandleTypeDef *hcan, Enum_DM_Motor_ID __CAN_ID, Enum_DM_Motor_PID_Control_Method __Control_Method, int32_t __Position_Offset, float __Omega_Max, float __Torque_Max)
{
    Class_DM_Motor_J4310::Init(hcan, __CAN_ID, DM_Motor_Control_Method_MIT_TORQUE, __Position_Offset, __Omega_Max, __Torque_Max);
    DM_Motor_PID_Control_Method = __Control_Method;
}

void Class_DM_Motor_J4310_PID::TIM_PID_PeriodElapsedCallback()
{
    switch (DM_Motor_PID_Control_Method)
    {
        case (DM_Motor_Control_Method_PID_OPENLOOP):
        {
            // 开环模式不需要PID控制，直接把Target_Torque输出到电机，在切换到开环模式时清除其他PID类中的积分项
            PID_Omega.Set_Integral_Error(0.0f);
            PID_Position.Set_Integral_Error(0.0f);
        }
        break;

        case (DM_Motor_Control_Method_PID_OMEGA):
        {
            PID_Omega.Set_Target(Get_Target_Omega());
            PID_Omega.Set_Now(Get_Now_Omega());
            PID_Omega.TIM_Adjust_PeriodElapsedCallback();

            Set_Target_Torque(PID_Omega.Get_Out());
        }
        break;
        
        case (DM_Motor_Control_Method_PID_POSITION):
        {
            PID_Position.Set_Target(Get_Target_Angle());
            PID_Position.Set_Now(Get_Now_Angle());
            PID_Position.TIM_Adjust_PeriodElapsedCallback();

            PID_Omega.Set_Target(PID_Position.Get_Out());
            PID_Omega.Set_Now(Get_Now_Omega());
            PID_Omega.TIM_Adjust_PeriodElapsedCallback();

            Set_Target_Torque(PID_Omega.Get_Out());
        }
        break;

        default:
        {

        }
        break;
    }
}


void Class_Gripper::Init()
{
    // 成员电机初始化
    DM_Motor_Rotary.Init(&hfdcan1, DM_Motor_ID_0xA6, DM_Motor_Control_Method_PID_POSITION, 0, 50.0f, 5.0f);
    DJI_Motor_Clamp.Init(&hfdcan1, DJI_Motor_ID_0x204, DJI_Motor_Control_Method_ANGLE, 36.0f, 10000.0f);

    // 电机控制参数初始化
    DJI_Motor_Clamp.PID_Omega.Init(1000.0f, 0.0f, 0.0f, 0.0f, 0.0f, 6000.0f);
    DJI_Motor_Clamp.PID_Angle.Init(12.0f, 0.0f, 0.0f, 0.0f, 0.0f, 20.0f);
    DM_Motor_Rotary.PID_Omega.Init(0.25f, 0.0f, 0.0f, 0.0f, 0.0f, 5.0f);
    DM_Motor_Rotary.PID_Position.Init(8.0f, 0.0f, 0.0f, 0.0f, 0.0f, 8.0f);

    // 父类状态机类初始化
    Class_FSM::Init(10, 0);
}

void Class_Gripper::Output()
{
    if (Gripper_Control_Type == Gripper_Control_Type_ENABLE)
    {
        DM_Motor_Rotary.Set_DM_Motor_Control_Method(DM_Motor_Control_Method_PID_POSITION);
        DM_Motor_Rotary.Set_Target_Angle(Target_DM_Radian);
        DM_Motor_Rotary.Set_Target_Omega(Target_DM_Omega);

        DJI_Motor_Clamp.Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_ANGLE);
        DJI_Motor_Clamp.Set_Target_Radian(Target_DJI_Radian);
        DJI_Motor_Clamp.Set_Target_Omega_Radian(Target_DJI_Omega);
    }
    else // 失能模式
    {
        DM_Motor_Rotary.Set_DM_Control_Status(DM_Motor_Control_Status_DISABLE);

        DJI_Motor_Clamp.Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_OPENLOOP);
        DJI_Motor_Clamp.Set_Out(0.0f);
    }
}

// 定时器回调解算函数
void Class_Gripper::TIM_Calculate_PeriodElapsedCallback()
{
    // 运动学解算
    Calculate_Kinematics();

    // Output();

    // 电机PID计算回调函数
    DM_Motor_Rotary.TIM_PID_PeriodElapsedCallback();
    DJI_Motor_Clamp.TIM_PID_PeriodElapsedCallback();

    // 达妙电机输出回调函数
    DM_Motor_Rotary.TIM_Process_PeriodElapsedCallback();
}

// 上电以及校准状态机
void Class_Gripper::Reload_TIM_Status_PeriodElapsedCallback()
{
}

void Class_Gripper::Calculate_Kinematics()
{
    // 夹持端相对转角
    float Now_Delta_Radian = DJI_Motor_Clamp.Get_Now_Radian() - DM_Motor_Rotary.Get_Now_Angle() + Clamp_Offset_Radian;
    float Target_Delta_Radian = Target_Length * LENGTH2RAD;
    // 夹持端相对角速度
    float Now_Delta_Omega = DJI_Motor_Clamp.Get_Now_Omega_Radian() - DM_Motor_Rotary.Get_Now_Omega();
    float Target_Delta_Omega = Target_Velocity * LENGTH2RAD;

    // 正运动学解算，读取电机返回数据，计算夹爪Roll角度和张合长度
    Gripper_Data.Now_Roll = DM_Motor_Rotary.Get_Now_Angle();
    Gripper_Data.Now_Omega = DM_Motor_Rotary.Get_Now_Omega();
    Gripper_Data.Now_Length = Now_Delta_Radian / LENGTH2RAD;
    Gripper_Data.Now_Velocity = Now_Delta_Omega / LENGTH2RAD;

    // 逆运动学解算，将夹爪Roll和张合运动解算到成员电机的运动
    Target_DM_Radian = Target_Roll_Radian;
    Target_DM_Omega = Target_Omega_Radian;
    Target_DJI_Radian = Target_Delta_Radian + Target_Roll_Radian;
    Target_DJI_Omega = Target_Delta_Omega + Target_Omega_Radian;
}

void Class_Gripper::DM_Motor_Rotary_CAN_RxCpltCallback(uint8_t *Rx_Data)
{
    DM_Motor_Rotary.CAN_RxCpltCallback(Rx_Data);
}

void Class_Gripper::DJI_Motor_Clamp_CAN_RxCpltCallback(uint8_t *Rx_Data)
{
    DJI_Motor_Clamp.CAN_RxCpltCallback(Rx_Data);
}

void Class_Gripper::TIM_Alive_PeriodElapsedCallback()
{
    DM_Motor_Rotary.TIM_Alive_PeriodElapsedCallback();
    DJI_Motor_Clamp.TIM_Alive_PeriodElapsedCallback();
}
/************************ COPYRIGHT(C) NEUQ-MOSASAURUS **************************/
