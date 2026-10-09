/**
 * @file crt_gimbal.cpp
 * @author lez by wanghongxi
 * @brief 云台
 * @version 0.1
 * @date 2024-07-1 0.1 24赛季定稿
 *
 * @copyright ZLLC 2024
 *
 */

/* Includes ------------------------------------------------------------------*/

#include "crt_gimbal.h"

/* Private macros ------------------------------------------------------------*/

/* Private types -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

/* Private function declarations ---------------------------------------------*/

/* Function prototypes -------------------------------------------------------*/

/**
 * @brief 云台初始化
 *
 */
void Class_Gimbal::Init()
{
    // imu初始化
    Boardc_BMI.Init();

    Gripper.Init();
}

/**
 * @brief 输出到电机
 *
 */
float temp_err = 0.0f;
float temp_target_angle = 0.0f;

float Tmp_Target_Yaw_Angle = 0.0f, Tmp_Ture_Yaw_Angle = 0.0f;
void Class_Gimbal::Output()
{
    if (Gimbal_Control_Type == Gimbal_Control_Type_DISABLE)
    {
        Gripper.Set_Gripper_Control_Type(Gripper_Control_Type_DISABLE);
    }
    else // 非失能模式
    {

        if (Gimbal_Control_Type == Gimbal_Control_Type_NORMAL)
        {
            Gripper.Set_Gripper_Control_Type(Gripper_Control_Type_ENABLE);
        }
        else if ((Gimbal_Control_Type == Gimbal_Control_Type_MINIPC) && (MiniPC->Get_MiniPC_Status() == MiniPC_Status_DISABLE))
        {
            
        }
        else if ((Gimbal_Control_Type == Gimbal_Control_Type_MINIPC) && (MiniPC->Get_MiniPC_Status() != MiniPC_Status_DISABLE))
        {

        }

    }
}

/**
 * @brief TIM定时器中断计算回调函数
 *
 */
void Class_Gimbal::TIM_Calculate_PeriodElapsedCallback()
{
    Output();

    Gripper.TIM_Calculate_PeriodElapsedCallback();
}

/************************ COPYRIGHT(C) USTC-ROBOWALKER **************************/
