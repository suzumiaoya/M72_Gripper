/**
 * @file dvc_gripper.cpp
 * @author hsl by suzumiaoya
 * @brief 旋转夹爪类驱动
 * @version 0.2
 * @date 2026-10-19
 *
 * @copyright ZLLC 2027
 */

#include "dvc_gripper.h"

/* Private variables ---------------------------------------------------------*/

// 17:34外啮合传动, DM转角到Roll转角的幅值比例
const float GRIPPER_DM_TO_ROLL_RATIO = 17.0f / 34.0f;

// M2006速度环参数及总输出限幅
const float GRIPPER_DJI_OMEGA_KP = 1000.0f;
const float GRIPPER_DJI_OUTPUT_MAX = 9000.0f;

// 主动张合相对角速度, rad/s
const float GRIPPER_OPEN_OMEGA = -3.0f;
const float GRIPPER_CLOSE_OMEGA = 4.0f;

// 夹持与外限位保持电流映射值
const float GRIPPER_CLAMP_CURRENT = 4500.0f;
const float GRIPPER_OPEN_HOLD_CURRENT = -1000.0f;

// 接触与夹紧判定参数
const float GRIPPER_STALL_OMEGA = 0.1f;
const float GRIPPER_STALL_CURRENT_RATIO = 0.9f;
const float GRIPPER_OPEN_STALL_CURRENT = GRIPPER_STALL_CURRENT_RATIO * GRIPPER_DJI_OMEGA_KP * GRIPPER_OPEN_OMEGA;
const float GRIPPER_CLOSE_STALL_CURRENT = GRIPPER_STALL_CURRENT_RATIO * GRIPPER_DJI_OMEGA_KP * GRIPPER_CLOSE_OMEGA;
const float GRIPPER_GRIPPED_CURRENT = GRIPPER_STALL_CURRENT_RATIO * GRIPPER_CLAMP_CURRENT;
const uint16_t GRIPPER_CONTACT_IGNORE_TIME = 50;
const uint16_t GRIPPER_CONTACT_CONFIRM_TIME = 20;
const uint16_t GRIPPER_CONTACT_CURRENT_RAMP_TIME = 30;

// 主动张合超时及停止时间, ms
const uint16_t GRIPPER_MOTION_TIMEOUT = 4000;
const uint16_t GRIPPER_MOTION_STOP_TIME = 1000;

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
    DM_Motor_Rotary.Init(&hfdcan2, DM_Motor_ID_0xA6, DM_Motor_Control_Method_POSITION_OMEGA, 0, 50.0f, 5.0f);
    // Torque_Max与Output_Max均为10000, TORQUE模式下目标值与电流映射值一一对应
    DJI_Motor_Clamp.Init(&hfdcan1, DJI_Motor_ID_0x204, DJI_Motor_Control_Method_TORQUE, 36.0f, 10000.0f);
    DJI_Motor_Clamp.Set_Out(0.0f);

    DJI_Motor_Clamp.PID_Omega.Init(GRIPPER_DJI_OMEGA_KP, 0.0f, 0.0f, 0.0f, 0.0f, GRIPPER_DJI_OUTPUT_MAX);
    Class_FSM::Init(Gripper_Clamp_Status_NUM, Gripper_Clamp_Status_DISABLED);
}

void Class_Gripper::Output()
{
    bool DM_Online = (DM_Motor_Rotary.Get_DM_Motor_Status() == DM_Motor_Status_ENABLE);
    bool DJI_Online = (DJI_Motor_Clamp.Get_DJI_Motor_Status() == DJI_Motor_Status_ENABLE);
    if ((Gripper_Control_Type != Gripper_Control_Type_ENABLE) ||
        (Gripper_Clamp_Status == Gripper_Clamp_Status_DISABLED) ||
        !DM_Online || !DJI_Online || !DM_Radian_Initialized)
    {
        DM_Motor_Rotary.Set_DM_Control_Status(DM_Motor_Control_Status_DISABLE);
        DM_Motor_Rotary.Set_Target_Omega(0.0f);

        Clamp_Current = 0.0f;
        Follow_Omega_Current = 0.0f;
        Target_DJI_Output = 0.0f;
        Target_DJI_Omega_Radian = 0.0f;
        DJI_Motor_Clamp.Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_TORQUE);
        DJI_Motor_Clamp.Set_Target_Torque(0.0f);
        return;
    }

    DM_Motor_Rotary.Set_DM_Control_Status(DM_Motor_Control_Status_ENABLE);
    DM_Motor_Rotary.Set_DM_Motor_Control_Method(DM_Motor_Control_Method_POSITION_OMEGA);

    if ((Gripper_Clamp_Status == Gripper_Clamp_Status_OPENED) ||
        (Gripper_Clamp_Status == Gripper_Clamp_Status_GRIPPED))
    {
        DM_Motor_Rotary.Set_Target_Angle(Target_DM_Radian);
        DM_Motor_Rotary.Set_Target_Omega(Target_DM_Omega);
    }
    else
    {
        DM_Motor_Rotary.Set_Target_Angle(Clamp_Hold_DM_Radian);
        DM_Motor_Rotary.Set_Target_Omega(0.0f);
    }

    Follow_Omega_Current = 0.0f;
    switch (Gripper_Clamp_Status)
    {
    case (Gripper_Clamp_Status_OPENING):
    case (Gripper_Clamp_Status_CLOSING):
    {
        float Motion_Omega = (Gripper_Clamp_Status == Gripper_Clamp_Status_OPENING) ? GRIPPER_OPEN_OMEGA : GRIPPER_CLOSE_OMEGA;
        if (Status[Now_Status_Serial].Time >= GRIPPER_MOTION_TIMEOUT)
        {
            float Stop_Ratio = 1.0f - (float)(Status[Now_Status_Serial].Time - GRIPPER_MOTION_TIMEOUT) / (float)GRIPPER_MOTION_STOP_TIME;
            Math_Constrain(&Stop_Ratio, 0.0f, 1.0f);
            Motion_Omega *= Stop_Ratio;
        }

        Clamp_Current = 0.0f;
        Target_DJI_Output = 0.0f;
        Target_DJI_Omega_Radian = Motion_Omega - GRIPPER_DM_TO_ROLL_RATIO * DM_Motor_Rotary.Get_Now_Omega();
        DJI_Motor_Clamp.Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_OMEGA);
        DJI_Motor_Clamp.Set_Target_Omega_Radian(Target_DJI_Omega_Radian);
    }
    break;

    case (Gripper_Clamp_Status_CONTACT):
    {
        Target_DJI_Omega_Radian = 0.0f;
        Target_DJI_Output = Clamp_Current;
        DJI_Motor_Clamp.Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_TORQUE);
        DJI_Motor_Clamp.Set_Target_Torque(Target_DJI_Output);
    }
    break;

    case (Gripper_Clamp_Status_GRIPPED):
    {
        Clamp_Current = GRIPPER_CLAMP_CURRENT;
        Target_DJI_Omega_Radian = -GRIPPER_DM_TO_ROLL_RATIO * DM_Motor_Rotary.Get_Now_Omega();
        DJI_Motor_Clamp.PID_Omega.Set_Target(Target_DJI_Omega_Radian);
        DJI_Motor_Clamp.PID_Omega.Set_Now(DJI_Motor_Clamp.Get_Now_Omega_Radian());
        DJI_Motor_Clamp.PID_Omega.TIM_Adjust_PeriodElapsedCallback();
        Follow_Omega_Current = DJI_Motor_Clamp.PID_Omega.Get_Out();
        Target_DJI_Output = Clamp_Current + Follow_Omega_Current;
        Math_Constrain(&Target_DJI_Output, 0.0f, GRIPPER_DJI_OUTPUT_MAX);
        DJI_Motor_Clamp.Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_TORQUE);
        DJI_Motor_Clamp.Set_Target_Torque(Target_DJI_Output);
    }
    break;

    case (Gripper_Clamp_Status_OPENED):
    {
        Clamp_Current = GRIPPER_OPEN_HOLD_CURRENT;
        Target_DJI_Omega_Radian = 0.0f;
        Target_DJI_Output = Clamp_Current;
        DJI_Motor_Clamp.Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_TORQUE);
        DJI_Motor_Clamp.Set_Target_Torque(Target_DJI_Output);
    }
    break;

    case (Gripper_Clamp_Status_TIMEOUT):
    default:
    {
        Clamp_Current = 0.0f;
        Target_DJI_Omega_Radian = 0.0f;
        Target_DJI_Output = 0.0f;
        DJI_Motor_Clamp.Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_TORQUE);
        DJI_Motor_Clamp.Set_Target_Torque(0.0f);
    }
    break;
    }
}

void Class_Gripper::TIM_Calculate_PeriodElapsedCallback()
{
    Calculate_Kinematics();
    Reload_TIM_Status_PeriodElapsedCallback();
    Output();

    DJI_Motor_Clamp.TIM_PID_PeriodElapsedCallback();
    if ((Gripper_Clamp_Status == Gripper_Clamp_Status_OPENING) ||
        (Gripper_Clamp_Status == Gripper_Clamp_Status_CLOSING))
    {
        Target_DJI_Output = DJI_Motor_Clamp.Get_Out();
    }
    Now_DJI_Output = DJI_Motor_Clamp.Get_Out();
    DM_Motor_Rotary.TIM_Process_PeriodElapsedCallback();
}

void Class_Gripper::Reload_TIM_Status_PeriodElapsedCallback()
{
    Status[Now_Status_Serial].Time++;

    bool DM_Online = (DM_Motor_Rotary.Get_DM_Motor_Status() == DM_Motor_Status_ENABLE);
    bool DJI_Online = (DJI_Motor_Clamp.Get_DJI_Motor_Status() == DJI_Motor_Status_ENABLE);
    if (!DM_Online || !DJI_Online)
    {
        Contact_Confirm_Count = 0;
        if (!DM_Online && !DJI_Online)
        {
            Reset_DM_Multi_Turn_Radian();
        }
        if (Gripper_Clamp_Status != Gripper_Clamp_Status_DISABLED)
        {
            Set_Clamp_Status(Gripper_Clamp_Status_DISABLED);
        }
        return;
    }

    if (Gripper_Control_Type == Gripper_Control_Type_DISABLE)
    {
        Contact_Confirm_Count = 0;
        DJI_Motor_Clamp.PID_Omega.Set_Integral_Error(0.0f);
        if (Gripper_Clamp_Status != Gripper_Clamp_Status_DISABLED)
        {
            Set_Clamp_Status(Gripper_Clamp_Status_DISABLED);
        }
        return;
    }

    if (!DM_Radian_Initialized)
    {
        return;
    }

    switch (Gripper_Clamp_Status)
    {
    case (Gripper_Clamp_Status_DISABLED):
    {
        Start_Clamp_Motion(Gripper_Clamp_Command);
    }
    break;

    case (Gripper_Clamp_Status_OPENING):
    {
        if (Gripper_Clamp_Command == Gripper_Clamp_Command_CLOSE)
        {
            Start_Clamp_Motion(Gripper_Clamp_Command_CLOSE);
            break;
        }
        if (Status[Now_Status_Serial].Time >= (GRIPPER_MOTION_TIMEOUT + GRIPPER_MOTION_STOP_TIME))
        {
            Timeout_Clamp_Command = Gripper_Clamp_Command;
            Set_Clamp_Status(Gripper_Clamp_Status_TIMEOUT);
            break;
        }
        if ((Status[Now_Status_Serial].Time >= GRIPPER_CONTACT_IGNORE_TIME) &&
            (Status[Now_Status_Serial].Time < GRIPPER_MOTION_TIMEOUT) &&
            (Math_Abs(Gripper_Data.Now_Clamp_Omega_Radian) < GRIPPER_STALL_OMEGA) &&
            (DJI_Motor_Clamp.Get_Now_Torque() <= GRIPPER_OPEN_STALL_CURRENT))
        {
            Contact_Confirm_Count++;
        }
        else
        {
            Contact_Confirm_Count = 0;
        }
        if (Contact_Confirm_Count >= GRIPPER_CONTACT_CONFIRM_TIME)
        {
            Enter_Roll_Enabled_Status(Gripper_Clamp_Status_OPENED);
        }
    }
    break;

    case (Gripper_Clamp_Status_OPENED):
    {
        if (Gripper_Clamp_Command == Gripper_Clamp_Command_CLOSE)
        {
            Start_Clamp_Motion(Gripper_Clamp_Command_CLOSE);
        }
    }
    break;

    case (Gripper_Clamp_Status_CLOSING):
    {
        if (Gripper_Clamp_Command == Gripper_Clamp_Command_OPEN)
        {
            Start_Clamp_Motion(Gripper_Clamp_Command_OPEN);
            break;
        }
        if (Status[Now_Status_Serial].Time >= (GRIPPER_MOTION_TIMEOUT + GRIPPER_MOTION_STOP_TIME))
        {
            Timeout_Clamp_Command = Gripper_Clamp_Command;
            Set_Clamp_Status(Gripper_Clamp_Status_TIMEOUT);
            break;
        }
        if ((Status[Now_Status_Serial].Time >= GRIPPER_CONTACT_IGNORE_TIME) &&
            (Status[Now_Status_Serial].Time < GRIPPER_MOTION_TIMEOUT) &&
            (Math_Abs(Gripper_Data.Now_Clamp_Omega_Radian) < GRIPPER_STALL_OMEGA) &&
            (DJI_Motor_Clamp.Get_Now_Torque() >= GRIPPER_CLOSE_STALL_CURRENT))
        {
            Contact_Confirm_Count++;
        }
        else
        {
            Contact_Confirm_Count = 0;
        }
        if (Contact_Confirm_Count >= GRIPPER_CONTACT_CONFIRM_TIME)
        {
            Contact_Start_Current = DJI_Motor_Clamp.Get_Out();
            Math_Constrain(&Contact_Start_Current, 0.0f, GRIPPER_CLAMP_CURRENT);
            Clamp_Current = Contact_Start_Current;
            Contact_Confirm_Count = 0;
            Set_Clamp_Status(Gripper_Clamp_Status_CONTACT);
        }
    }
    break;

    case (Gripper_Clamp_Status_CONTACT):
    {
        if (Gripper_Clamp_Command == Gripper_Clamp_Command_OPEN)
        {
            Start_Clamp_Motion(Gripper_Clamp_Command_OPEN);
            break;
        }

        float Ramp_Ratio = (float)Status[Now_Status_Serial].Time / (float)GRIPPER_CONTACT_CURRENT_RAMP_TIME;
        Math_Constrain(&Ramp_Ratio, 0.0f, 1.0f);
        Clamp_Current = Contact_Start_Current + (GRIPPER_CLAMP_CURRENT - Contact_Start_Current) * Ramp_Ratio;

        if ((Status[Now_Status_Serial].Time >= GRIPPER_CONTACT_CURRENT_RAMP_TIME) &&
            (Math_Abs(Gripper_Data.Now_Clamp_Omega_Radian) < GRIPPER_STALL_OMEGA) &&
            (DJI_Motor_Clamp.Get_Now_Torque() >= GRIPPER_GRIPPED_CURRENT))
        {
            Contact_Confirm_Count++;
        }
        else
        {
            Contact_Confirm_Count = 0;
        }
        if (Contact_Confirm_Count >= GRIPPER_CONTACT_CONFIRM_TIME)
        {
            Enter_Roll_Enabled_Status(Gripper_Clamp_Status_GRIPPED);
        }
    }
    break;

    case (Gripper_Clamp_Status_GRIPPED):
    {
        if (Gripper_Clamp_Command == Gripper_Clamp_Command_OPEN)
        {
            Start_Clamp_Motion(Gripper_Clamp_Command_OPEN);
        }
    }
    break;

    case (Gripper_Clamp_Status_TIMEOUT):
    {
        if (Gripper_Clamp_Command != Timeout_Clamp_Command)
        {
            Start_Clamp_Motion(Gripper_Clamp_Command);
        }
    }
    break;

    default:
    {
        Set_Clamp_Status(Gripper_Clamp_Status_DISABLED);
    }
    break;
    }
}

void Class_Gripper::Calculate_Kinematics()
{
    // DM底层将-pmax~+pmax平移为0~2PI, 在夹爪层恢复为-PI~+PI模角度
    DM_Mod_Radian = Normalize_Angle_Radian_PI_to_PI(DM_Motor_Rotary.Get_Now_Angle() - PI);

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

    Now_Roll_Total_Radian = -GRIPPER_DM_TO_ROLL_RATIO * Now_DM_Radian;

    Gripper_Data.Now_Roll_Radian = Normalize_Angle_Radian_PI_to_PI(Now_Roll_Total_Radian);
    Gripper_Data.Now_Roll_Omega_Radian = -GRIPPER_DM_TO_ROLL_RATIO * DM_Motor_Rotary.Get_Now_Omega();
    Gripper_Data.Now_Clamp_Omega_Radian = DJI_Motor_Clamp.Get_Now_Omega_Radian() + GRIPPER_DM_TO_ROLL_RATIO * DM_Motor_Rotary.Get_Now_Omega();

    if (DM_Radian_Initialized &&
        ((Gripper_Clamp_Status == Gripper_Clamp_Status_OPENED) ||
         (Gripper_Clamp_Status == Gripper_Clamp_Status_GRIPPED)))
    {
        Target_Roll_Radian = Normalize_Angle_Radian_PI_to_PI(Target_Roll_Radian);
        float Delta_Target_Roll_Radian = Normalize_Angle_Radian_PI_to_PI(Target_Roll_Radian - Gripper_Data.Now_Roll_Radian);
        Target_DM_Radian = Now_DM_Radian - Delta_Target_Roll_Radian / GRIPPER_DM_TO_ROLL_RATIO;
        Target_Roll_Total_Radian = -GRIPPER_DM_TO_ROLL_RATIO * Target_DM_Radian;
        Target_DM_Omega = -Target_Roll_Omega_Radian / GRIPPER_DM_TO_ROLL_RATIO;
    }
}

void Class_Gripper::Set_Clamp_Status(Enum_Gripper_Clamp_Status __Gripper_Clamp_Status)
{
    Gripper_Clamp_Status = __Gripper_Clamp_Status;
    Set_Status(static_cast<uint8_t>(__Gripper_Clamp_Status));
}

void Class_Gripper::Start_Clamp_Motion(Enum_Gripper_Clamp_Command __Gripper_Clamp_Command)
{
    Clamp_Hold_DM_Radian = Now_DM_Radian;
    Target_DM_Radian = Now_DM_Radian;
    Target_DM_Omega = 0.0f;
    Target_Roll_Radian = Gripper_Data.Now_Roll_Radian;
    Target_Roll_Total_Radian = Now_Roll_Total_Radian;
    Target_DJI_Omega_Radian = 0.0f;
    Clamp_Current = 0.0f;
    Follow_Omega_Current = 0.0f;
    Target_DJI_Output = 0.0f;
    Contact_Confirm_Count = 0;
    DJI_Motor_Clamp.PID_Omega.Set_Integral_Error(0.0f);

    if (__Gripper_Clamp_Command == Gripper_Clamp_Command_OPEN)
    {
        Set_Clamp_Status(Gripper_Clamp_Status_OPENING);
    }
    else
    {
        Set_Clamp_Status(Gripper_Clamp_Status_CLOSING);
    }
}

void Class_Gripper::Enter_Roll_Enabled_Status(Enum_Gripper_Clamp_Status __Gripper_Clamp_Status)
{
    Target_Roll_Radian = Gripper_Data.Now_Roll_Radian;
    Target_Roll_Total_Radian = Now_Roll_Total_Radian;
    Target_DM_Radian = Now_DM_Radian;
    Target_DM_Omega = 0.0f;
    Target_DJI_Omega_Radian = 0.0f;
    Contact_Confirm_Count = 0;
    DJI_Motor_Clamp.PID_Omega.Set_Integral_Error(0.0f);
    Set_Clamp_Status(__Gripper_Clamp_Status);
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
