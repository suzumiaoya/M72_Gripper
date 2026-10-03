/**
 * @file crt_gimbal.h
 * @author lez by wanghongxi
 * @brief 云台
 * @version 0.1
 * @date 2024-07-1 0.1 24赛季定稿
 *
 * @copyright ZLLC 2024
 *
 */

#ifndef CRT_GIMBAL_H
#define CRT_GIMBAL_H

/* Includes ------------------------------------------------------------------*/

#include "dvc_djimotor.h"
// #include "dvc_witahrs.h"
#include "dvc_minipc.h"
#include "dvc_imu.h"
#include "dvc_lkmotor.h"
#include "dvc_gripper.h"

/* Exported macros -----------------------------------------------------------*/

/* Exported types ------------------------------------------------------------*/


/**
 * @brief 云台控制类型
 *
 */
enum Enum_Gimbal_Control_Type :uint8_t
{
    Gimbal_Control_Type_DISABLE = 0,
    Gimbal_Control_Type_NORMAL,
    Gimbal_Control_Type_MINIPC,
};

/**
 * @brief Specialized, 云台类
 *
 */
class Class_Gimbal
{
public:

    //imu对象
    Class_IMU Boardc_BMI;

    Class_MiniPC *MiniPC;

    // 夹爪电机类及其成员电机
    Class_Gripper Gripper;

    void Init();

    inline Enum_Gimbal_Control_Type Get_Gimbal_Control_Type();

    inline void Set_Gimbal_Control_Type(Enum_Gimbal_Control_Type __Gimbal_Control_Type);

    void TIM_Calculate_PeriodElapsedCallback();

protected:
    //初始化相关常量

    //常量
    // yaw轴最小值
    float Min_Yaw_Angle = - 180.0f;
    // yaw轴最大值
    float Max_Yaw_Angle = 180.0f;

    //yaw总角度
    float Yaw_Total_Angle;
    float Yaw_Half_Turns;

    // pitch轴最小值
    float Min_Pitch_Angle = -15.0f;
    // pitch轴最大值
    float Max_Pitch_Angle = 30.0f ; //多10°

    //内部变量 

    //读变量

    //写变量

    //云台状态
    Enum_Gimbal_Control_Type Gimbal_Control_Type = Gimbal_Control_Type_DISABLE ;

    //读写变量

    // yaw轴角度
    float Target_Yaw_Angle = 0.0f;
    // pitch轴角度
    float Target_Pitch_Angle = 0.0f;

    //内部函数

    void Output();
};

/* Exported variables --------------------------------------------------------*/

/* Exported function declarations --------------------------------------------*/

/**
 * @brief 获取云台控制类型
 *
 * @return Enum_Gimbal_Control_Type 获取云台控制类型
 */
Enum_Gimbal_Control_Type Class_Gimbal::Get_Gimbal_Control_Type()
{
    return (Gimbal_Control_Type);
}

/**
 * @brief 设定云台状态
 *
 * @param __Gimbal_Control_Type 云台状态
 */
void Class_Gimbal::Set_Gimbal_Control_Type(Enum_Gimbal_Control_Type __Gimbal_Control_Type)
{
    Gimbal_Control_Type = __Gimbal_Control_Type;
}

#endif

/************************ COPYRIGHT(C) USTC-ROBOWALKER **************************/
