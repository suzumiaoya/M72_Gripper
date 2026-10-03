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
void Class_Gripper::Init()
{
    // 成员电机初始化
    DM_Motor_Rotary.Init(&hfdcan1, DM_Motor_ID_0xA1, DM_Motor_Control_Method_MIT_POSITION, 0, 20.94359f, 10.0f);
    DJI_Motor_Clamp.Init(&hfdcan1, DJI_Motor_ID_0x204, DJI_Motor_Control_Method_ANGLE, 36.0f, 10000.0f);
    
    DJI_Motor_Clamp.PID_Omega.Init(1000.0f, 0.0f, 0.0f, 0.0f, 0.0f, 6000.0f);
    DJI_Motor_Clamp.PID_Angle.Init(10.0f, 0.0f, 0.0f, 0.0f, 0.0f, 20.0f);

    // 父类状态机类初始化
    Class_FSM::Init(10, 0);
}

void Class_Gripper::Output()
{
    if (Gripper_Control_Type == Gripper_Control_Type_ENABLE)
    {
        DM_Motor_Rotary.Set_DM_Motor_Control_Method(DM_Motor_Control_Method_POSITION_OMEGA);
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

    // 电机自身回调函数
    DM_Motor_Rotary.TIM_Process_PeriodElapsedCallback();
    DJI_Motor_Clamp.TIM_PID_PeriodElapsedCallback();
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
