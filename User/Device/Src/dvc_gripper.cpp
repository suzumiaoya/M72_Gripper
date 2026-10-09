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

// 达妙电机转角到M2006跟随转角的实测比例
#define GRIPPER_DM_FOLLOW_RATIO 1.05f
// 外限位校准角速度, rad/s
#define GRIPPER_CALI_OMEGA (-3.0f)
// 外限位堵转相对角速度阈值, rad/s
#define GRIPPER_CALI_STALL_OMEGA 0.05f
// 外限位堵转确认时间, ms
#define GRIPPER_CALI_STALL_TIME 100
// 达妙电机使能等待时间, ms
#define GRIPPER_DM_ENABLE_WAIT_TIME 100
// 外限位搜索超时时间, ms
#define GRIPPER_CALI_TIMEOUT 4000
// M2006角速度环参数及输出限幅
#define GRIPPER_DJI_OMEGA_KP 1000.0f
#define GRIPPER_DJI_OUTPUT_MAX 9000.0f
// 夹爪张合位置外环参数及M2006目标角速度限幅
#define GRIPPER_CLAMP_POSITION_KP 10.0f
#define GRIPPER_DJI_OMEGA_MAX 20.0f
// 堵转阈值只由校准目标速度计算, 不使用带噪声的实时反馈速度
#define GRIPPER_CALI_CURRENT_RATIO 0.9f
#define GRIPPER_CALI_CURRENT_CALCULATED (GRIPPER_CALI_CURRENT_RATIO * GRIPPER_DJI_OMEGA_KP * (-GRIPPER_CALI_OMEGA))
#define GRIPPER_CALI_CURRENT_THRESHOLD ((GRIPPER_CALI_CURRENT_CALCULATED < GRIPPER_DJI_OUTPUT_MAX) ? GRIPPER_CALI_CURRENT_CALCULATED : GRIPPER_DJI_OUTPUT_MAX)

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
        // 根据 Target_Omega 为位置环设置最大限幅，即最大旋转转速
        // PID_Position.Set_Out_Max(Math_Abs(Get_Target_Omega()));
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
    DM_Motor_Rotary.Init(&hfdcan2, DM_Motor_ID_0xA6, DM_Motor_Control_Method_POSITION_OMEGA, 0, 50.0f, 5.0f);
    DJI_Motor_Clamp.Init(&hfdcan1, DJI_Motor_ID_0x204, DJI_Motor_Control_Method_OPENLOOP, 36.0f, 10000.0f);
    // DM_Motor_Rotary.Set_Target_Torque(0.0f);
    DJI_Motor_Clamp.Set_Out(0.0f);

    // 电机控制参数初始化
    DJI_Motor_Clamp.PID_Omega.Init(GRIPPER_DJI_OMEGA_KP, 0.0f, 0.0f, 0.0f, 0.0f, GRIPPER_DJI_OUTPUT_MAX);
    PID_Clamp.Init(GRIPPER_CLAMP_POSITION_KP, 0.0f, 0.0f, 0.0f, 0.0f, GRIPPER_DJI_OMEGA_MAX);
    // DM_Motor_Rotary.PID_Omega.Init(0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 5.0f);
    // DM_Motor_Rotary.PID_Position.Init(0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 5.0f);

    // 父类状态机类初始化
    Class_FSM::Init(Gripper_Cali_Status_NUM, Gripper_Cali_Status_UNCALIBRATED);
}

void Class_Gripper::Output()
{
    // 所有阶段的电机模式和目标均由此函数单点写入, 状态机只负责状态转移和校准判定
    if ((Gripper_Control_Type == Gripper_Control_Type_ENABLE) &&
        (Gripper_Cali_Status == Gripper_Cali_Status_CALIBRATING))
    {
        // 校准状态下保持Roll不变, 等待达妙电机使能后控制M2006向外限位运动
        DM_Motor_Rotary.Set_DM_Control_Status(DM_Motor_Control_Status_ENABLE);
        DM_Motor_Rotary.Set_DM_Motor_Control_Method(DM_Motor_Control_Method_POSITION_OMEGA);
        DM_Motor_Rotary.Set_Target_Angle(Cali_Hold_DM_Radian);
        DM_Motor_Rotary.Set_Target_Omega(0.0f);

        if (Status[Now_Status_Serial].Time > GRIPPER_DM_ENABLE_WAIT_TIME)
        {
            DJI_Motor_Clamp.Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_OMEGA);
            DJI_Motor_Clamp.Set_Target_Omega_Radian(GRIPPER_CALI_OMEGA);
        }
        else
        {
            DJI_Motor_Clamp.Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_OPENLOOP);
            DJI_Motor_Clamp.Set_Target_Torque(0.0f);
            DJI_Motor_Clamp.Set_Out(0.0f);
        }
    }
    else if ((Gripper_Control_Type == Gripper_Control_Type_ENABLE) &&
             (Gripper_Cali_Status == Gripper_Cali_Status_CALIBRATED))
    {
        // 正常控制状态
        DM_Motor_Rotary.Set_DM_Control_Status(DM_Motor_Control_Status_ENABLE);
        DM_Motor_Rotary.Set_DM_Motor_Control_Method(DM_Motor_Control_Method_POSITION_OMEGA);
        DM_Motor_Rotary.Set_Target_Angle(Target_DM_Radian);
        DM_Motor_Rotary.Set_Target_Omega(Target_DM_Omega);

        DJI_Motor_Clamp.Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_OMEGA);
        DJI_Motor_Clamp.Set_Target_Omega_Radian(Target_DJI_Omega_Radian);
    }
    else // 失能、未校准、校准失败或电机离线时安全停机
    {
        DM_Motor_Rotary.Set_DM_Control_Status(DM_Motor_Control_Status_DISABLE);
        DM_Motor_Rotary.Set_Target_Omega(0.0f);

        DJI_Motor_Clamp.Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_OPENLOOP);
        DJI_Motor_Clamp.Set_Target_Torque(0.0f);
        DJI_Motor_Clamp.Set_Out(0.0f);
    }
}

// 定时器回调解算函数
void Class_Gripper::TIM_Calculate_PeriodElapsedCallback()
{
    // 运动学解算
    Calculate_Kinematics();

    // 上电、掉线恢复及外限位校准状态机
    Reload_TIM_Status_PeriodElapsedCallback();

    // 输出目标到成员电机
    Output();

    // 电机PID计算回调函数
    DJI_Motor_Clamp.TIM_PID_PeriodElapsedCallback();
    // 达妙电机输出回调函数
    DM_Motor_Rotary.TIM_Process_PeriodElapsedCallback();
}

// 上电以及校准状态机
void Class_Gripper::Reload_TIM_Status_PeriodElapsedCallback()
{
    Status[Now_Status_Serial].Time++;

    bool DM_Online = (DM_Motor_Rotary.Get_DM_Motor_Status() == DM_Motor_Status_ENABLE);
    bool DJI_Online = (DJI_Motor_Clamp.Get_DJI_Motor_Status() == DJI_Motor_Status_ENABLE);

    // 任一成员电机离线后校准失效并停止控制
    if (!DM_Online || !DJI_Online)
    {
        Cali_Stall_Count = 0;

        // M2006单独离线时DM仍保有内部多圈位置, 软件必须继续保留同一累计坐标。
        // 两台电机共同离线视为电机电源掉电, DM内部圈数将清零, 软件多圈角也需重新锚定。
        if (!DM_Online && !DJI_Online)
        {
            Reset_DM_Multi_Turn_Radian();
        }

        if (Gripper_Cali_Status != Gripper_Cali_Status_UNCALIBRATED)
        {
            Set_Cali_Status(Gripper_Cali_Status_UNCALIBRATED);
        }
        return;
    }

    switch (Gripper_Cali_Status)
    {
    case (Gripper_Cali_Status_UNCALIBRATED):
    {
        // 两电机在线且上层使能后开始外限位校准
        if ((Gripper_Control_Type == Gripper_Control_Type_ENABLE) && DM_Radian_Initialized)
        {
            Cali_Hold_DM_Radian = Now_DM_Radian;
            Cali_Stall_Count = 0;
            Set_Cali_Status(Gripper_Cali_Status_CALIBRATING);
        }
    }
    break;

    case (Gripper_Cali_Status_CALIBRATING):
    {
        // 校准过程中失能则中止, 下次使能重新开始
        if (Gripper_Control_Type == Gripper_Control_Type_DISABLE)
        {
            Cali_Stall_Count = 0;
            Set_Cali_Status(Gripper_Cali_Status_UNCALIBRATED);
            break;
        }

        // 达妙电机完成使能等待后才开始进行堵转检测与校准超时计时
        if (Status[Now_Status_Serial].Time > GRIPPER_DM_ENABLE_WAIT_TIME)
        {
            if ((Math_Abs(DJI_Motor_Clamp.Get_Now_Omega_Radian()) < GRIPPER_CALI_STALL_OMEGA) &&
                (Math_Abs(DJI_Motor_Clamp.Get_Now_Torque()) >= GRIPPER_CALI_CURRENT_THRESHOLD))
            {
                Cali_Stall_Count++;
            }
            else
            {
                Cali_Stall_Count = 0;
            }

            if (Cali_Stall_Count >= GRIPPER_CALI_STALL_TIME)
            {
                // 最大张开位置定义为张合相对角0rad
                Clamp_Open_Offset_Radian = Now_Clamp_Raw_Radian;
                // 校准完成后默认保持当前Roll并张开，避免沿用校准前的旧目标
                Target_Roll_Radian = Gripper_Data.Now_Roll_Radian;
                Target_Clamp_Radian = 0.0f;
                // 状态切换当周期先保持当前位置, 下一周期再执行上层最新目标
                Target_DM_Radian = Now_DM_Radian;
                Target_DM_Omega = 0.0f;
                Target_DJI_Omega_Radian = 0.0f;
                PID_Clamp.Set_Integral_Error(0.0f);
                Cali_Stall_Count = 0;
                Set_Cali_Status(Gripper_Cali_Status_CALIBRATED);
            }
            else if (Status[Now_Status_Serial].Time > (GRIPPER_DM_ENABLE_WAIT_TIME + GRIPPER_CALI_TIMEOUT))
            {
                // 超时后锁定失败状态, 需先失能再使能才能重试
                Cali_Stall_Count = 0;
                Set_Cali_Status(Gripper_Cali_Status_FAILED);
            }
        }
    }
    break;

    case (Gripper_Cali_Status_CALIBRATED):
    {
        // 上层失能但成员电机仍在线时保留校准结果
    }
    break;

    case (Gripper_Cali_Status_FAILED):
    {
        // 先失能解除失败锁定, 下次使能重新校准
        if (Gripper_Control_Type == Gripper_Control_Type_DISABLE)
        {
            Set_Cali_Status(Gripper_Cali_Status_UNCALIBRATED);
        }
    }
    break;

    default:
    {
        Set_Cali_Status(Gripper_Cali_Status_UNCALIBRATED);
    }
    break;
    }
}

void Class_Gripper::Calculate_Kinematics()
{
    // 达妙电机底层将-pmax~+pmax平移为0~2PI, 在夹爪层恢复为-PI~+PI模角度
    DM_Mod_Radian = Normalize_Angle_Radian_PI_to_PI(DM_Motor_Rotary.Get_Now_Angle() - PI);

    // 夹爪层独立展开达妙电机模角, Now_DM_Radian与电机内部的n*2PI+q保持一致
    if ((DM_Motor_Rotary.Get_DM_Motor_Status() == DM_Motor_Status_ENABLE) && !DM_Radian_Initialized)
    {
        Pre_DM_Mod_Radian = DM_Mod_Radian;
        Now_DM_Radian = DM_Mod_Radian;
        DM_Radian_Initialized = true;
    }
    else if ((DM_Motor_Rotary.Get_DM_Motor_Status() == DM_Motor_Status_ENABLE) && DM_Radian_Initialized)
    {
        float Delta_DM_Radian = Normalize_Angle_Radian_PI_to_PI(DM_Mod_Radian - Pre_DM_Mod_Radian);
        Now_DM_Radian += Delta_DM_Radian;
        Pre_DM_Mod_Radian = DM_Mod_Radian;
    }

    Now_DJI_Radian = DJI_Motor_Clamp.Get_Now_Radian();
    Now_Roll_Total_Radian = -Now_DM_Radian;
    Now_Clamp_Raw_Radian = Now_DJI_Radian + GRIPPER_DM_FOLLOW_RATIO * Now_DM_Radian;

    // 正运动学解算
    Gripper_Data.Now_Roll_Radian = Normalize_Angle_Radian_PI_to_PI(Now_Roll_Total_Radian);
    Gripper_Data.Now_Roll_Omega_Radian = -DM_Motor_Rotary.Get_Now_Omega();
    Gripper_Data.Now_Clamp_Omega_Radian = DJI_Motor_Clamp.Get_Now_Omega_Radian() + GRIPPER_DM_FOLLOW_RATIO * DM_Motor_Rotary.Get_Now_Omega();

    if (Gripper_Cali_Status == Gripper_Cali_Status_CALIBRATED)
    {
        Gripper_Data.Now_Clamp_Radian = Now_Clamp_Raw_Radian - Clamp_Open_Offset_Radian;
    }
    else
    {
        Gripper_Data.Now_Clamp_Radian = 0.0f;
    }

    // 正常控制状态下进行逆运动学解算
    if ((Gripper_Cali_Status == Gripper_Cali_Status_CALIBRATED) && DM_Radian_Initialized)
    {
        Target_Roll_Radian = Normalize_Angle_Radian_PI_to_PI(Target_Roll_Radian);
        Math_Constrain(&Target_Clamp_Radian, 0.0f, GRIPPER_CLAMP_MAX_RADIAN);

        // 选择距离当前Roll最近的等价连续目标, 实现+-PI过零点连续转动
        float Delta_Target_Roll_Radian = Normalize_Angle_Radian_PI_to_PI(Target_Roll_Radian - Gripper_Data.Now_Roll_Radian);
        // Now_DM_Radian是n*2PI+q, 在此基础上叠加相对变化量, 不能直接下发模角q
        Target_DM_Radian = Now_DM_Radian - Delta_Target_Roll_Radian;
        Target_Roll_Total_Radian = -Target_DM_Radian;
        Target_DM_Omega = -Target_Roll_Omega_Radian;

        // 夹爪张合位置外环输出主动开合速度, DM实际角速度前馈抵消Roll运动引起的被动开合
        PID_Clamp.Set_Target(Target_Clamp_Radian);
        PID_Clamp.Set_Now(Gripper_Data.Now_Clamp_Radian);
        PID_Clamp.TIM_Adjust_PeriodElapsedCallback();
        Target_DJI_Omega_Radian = PID_Clamp.Get_Out() - GRIPPER_DM_FOLLOW_RATIO * DM_Motor_Rotary.Get_Now_Omega();
        Math_Constrain(&Target_DJI_Omega_Radian, -GRIPPER_DJI_OMEGA_MAX, GRIPPER_DJI_OMEGA_MAX);
    }
}

void Class_Gripper::Set_Cali_Status(Enum_Gripper_Cali_Status __Gripper_Cali_Status)
{
    Gripper_Cali_Status = __Gripper_Cali_Status;
    Set_Status(static_cast<uint8_t>(__Gripper_Cali_Status));
}

void Class_Gripper::Reset_DM_Multi_Turn_Radian()
{
    DM_Radian_Initialized = false;
    DM_Mod_Radian = 0.0f;
    Pre_DM_Mod_Radian = 0.0f;
    Now_DM_Radian = 0.0f;
    Now_Roll_Total_Radian = 0.0f;
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
