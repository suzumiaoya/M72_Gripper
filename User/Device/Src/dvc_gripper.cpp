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

// DM到旋转齿轮的转角幅值比与方向分别定义，外啮合时方向相反
#define GRIPPER_DM_TO_ROLL_RATIO (17.0f / 34.0f)
#define GRIPPER_DM_TO_ROLL_DIRECTION (-1.0f)

// M2006速度环参数及总输出限幅
const float GRIPPER_DJI_OMEGA_KP = 1000.0f;
const float GRIPPER_DJI_OUTPUT_MAX = 9000.0f;

// 主动张合相对角速度, rad/s
const float GRIPPER_OPEN_OMEGA = -3.0f;
const float GRIPPER_CLOSE_OMEGA = 4.0f;

// 外限位保持电流映射值，夹持设定值保存在Target_Clamp_Current中
const float GRIPPER_OPEN_HOLD_CURRENT = -1000.0f;

// 接触与夹紧判定参数
const float GRIPPER_STALL_OMEGA = 0.1f;
const float GRIPPER_STALL_CURRENT_RATIO = 0.9f;
// 堵转时速度近似为0，以目标速度的幅值估算电流阈值，两个阈值均为正
const float GRIPPER_OPEN_STALL_CURRENT = GRIPPER_STALL_CURRENT_RATIO * GRIPPER_DJI_OMEGA_KP *
    ((GRIPPER_OPEN_OMEGA < 0.0f) ? -GRIPPER_OPEN_OMEGA : GRIPPER_OPEN_OMEGA);
const float GRIPPER_CLOSE_STALL_CURRENT = GRIPPER_STALL_CURRENT_RATIO * GRIPPER_DJI_OMEGA_KP *
    ((GRIPPER_CLOSE_OMEGA < 0.0f) ? -GRIPPER_CLOSE_OMEGA : GRIPPER_CLOSE_OMEGA);
const uint16_t GRIPPER_CONTACT_IGNORE_TIME = 50;
const uint16_t GRIPPER_CONTACT_CONFIRM_TIME = 20;
const uint16_t GRIPPER_CONTACT_CURRENT_RAMP_TIME = 30;

// 主动张合超时及停止时间, ms
const uint16_t GRIPPER_MOTION_TIMEOUT = 4000;
const uint16_t GRIPPER_MOTION_STOP_TIME = 1000;

/* Function prototypes -------------------------------------------------------*/

void Class_Gripper_DM_Motor::Init(FDCAN_HandleTypeDef *hcan, Enum_DM_Motor_ID __CAN_ID, Enum_DM_Motor_Control_Method __Control_Method, float __PMAX, int32_t __Position_Offset, float __Omega_Max, float __Torque_Max)
{
    Class_DM_Motor_J4310::Init(hcan, __CAN_ID, __Control_Method, __Position_Offset, __Omega_Max, __Torque_Max);
    PMAX = __PMAX;
    Reset_Multi_Turn_Radian();
}

void Class_Gripper_DM_Motor::CAN_RxCpltCallback(uint8_t *Rx_Data)
{
    Class_DM_Motor_J4310::CAN_RxCpltCallback(Rx_Data);
    Feedback_Pending = true;
    Multi_Turn_Radian_Data_Process();
}

void Class_Gripper_DM_Motor::TIM_Alive_PeriodElapsedCallback()
{
    Class_DM_Motor_J4310::TIM_Alive_PeriodElapsedCallback();
    // 首次反馈可能先于在线检测，转在线后再处理该帧
    Multi_Turn_Radian_Data_Process();
}

void Class_Gripper_DM_Motor::Multi_Turn_Radian_Data_Process()
{
    if (!Feedback_Pending || (Get_DM_Motor_Status() != DM_Motor_Status_ENABLE))
    {
        return;
    }
    Feedback_Pending = false;

    // Pre_Position是基类保存的本帧原始位置，不依赖其累计角度和位置映射
    DM_Feedback_Radian = Math_Int_To_Float(Data.Pre_Position, 0, 65535, -PMAX, PMAX);
    if (!DM_Radian_Initialized)
    {
        // 首帧可能已经包含电机内部记录的多圈角度，必须完整保留
        Pre_DM_Feedback_Radian = DM_Feedback_Radian;
        Now_DM_Radian = DM_Feedback_Radian;
        DM_Radian_Initialized = true;
    }
    else
    {
        float Delta_DM_Radian = DM_Feedback_Radian - Pre_DM_Feedback_Radian;
        // 相邻反馈的实际转角应小于PMAX；跨越反馈边界时按2*PMAX还原增量
        if (Delta_DM_Radian > PMAX)
        {
            Delta_DM_Radian -= 2.0f * PMAX;
        }
        else if (Delta_DM_Radian < -PMAX)
        {
            Delta_DM_Radian += 2.0f * PMAX;
        }
        Now_DM_Radian += Delta_DM_Radian;
        Pre_DM_Feedback_Radian = DM_Feedback_Radian;
    }
}

void Class_Gripper_DM_Motor::Reset_Multi_Turn_Radian()
{
    DM_Radian_Initialized = false;
    Feedback_Pending = false;
    DM_Feedback_Radian = 0.0f;
    Pre_DM_Feedback_Radian = 0.0f;
    Now_DM_Radian = 0.0f;
}

void Class_Gripper_DJI_Motor::Init(FDCAN_HandleTypeDef *hcan, Enum_DJI_Motor_ID __CAN_ID, Enum_DJI_Motor_Control_Method __Control_Method, float __Gearbox_Rate)
{
    // Torque_Max与基类Output_Max均为10000，使目标值与CAN电流映射值一一对应
    // 本子类的目标、速度环输出与Out均使用电流映射值；1000对应1A，不在此换算Nm
    Class_DJI_Motor_C610::Init(hcan, __CAN_ID, __Control_Method, __Gearbox_Rate, 10000.0f);
}

void Class_Gripper_DJI_Motor::Set_Current_Control(float __Target_Current)
{
    Gripper_DJI_Control_Method = Gripper_DJI_Control_Method_CURRENT;
    Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_TORQUE);
    Target_Current = __Target_Current;
    Set_Target_Torque(__Target_Current);
    Set_Target_Omega_Radian(0.0f);
    Target_Clamp_Current = 0.0f;
}

void Class_Gripper_DJI_Motor::Set_Omega_Control(float __Target_Omega_Radian)
{
    Gripper_DJI_Control_Method = Gripper_DJI_Control_Method_OMEGA;
    Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_OMEGA);
    Set_Target_Omega_Radian(__Target_Omega_Radian);
    Target_Clamp_Current = 0.0f;
}

void Class_Gripper_DJI_Motor::Set_Clamp_Follow_Control(float __Target_Clamp_Current, float __Target_Omega_Radian)
{
    Gripper_DJI_Control_Method = Gripper_DJI_Control_Method_CLAMP_FOLLOW;
    Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_TORQUE);
    Target_Clamp_Current = __Target_Clamp_Current;
    Set_Target_Omega_Radian(__Target_Omega_Radian);
}

void Class_Gripper_DJI_Motor::Reset_Omega_PID()
{
    PID_Omega.Set_Integral_Error(0.0f);
}

void Class_Gripper_DJI_Motor::TIM_Control_PeriodElapsedCallback()
{
    switch (Gripper_DJI_Control_Method)
    {
    case (Gripper_DJI_Control_Method_CURRENT):
    {
        Out = Target_Current;
    }
    break;

    case (Gripper_DJI_Control_Method_OMEGA):
    case (Gripper_DJI_Control_Method_CLAMP_FOLLOW):
    {
        PID_Omega.Set_Target(Get_Target_Omega_Radian());
        PID_Omega.Set_Now(Get_Now_Omega_Radian());
        PID_Omega.TIM_Adjust_PeriodElapsedCallback();
        Out = PID_Omega.Get_Out();
        if (Gripper_DJI_Control_Method == Gripper_DJI_Control_Method_CLAMP_FOLLOW)
        {
            Out = Target_Clamp_Current + PID_Omega.Get_Out();
            Math_Constrain(&Out, 0.0f, GRIPPER_DJI_OUTPUT_MAX);
            Set_Target_Torque(Out);
        }
    }
    break;

    default:
    {
        Out = 0.0f;
    }
    break;
    }

    // 基类Output仅将Out写入CAN发送缓存，不再调用基类PID回调重复闭环
    Output();
}

void Class_Gripper::Init()
{
    // PMAX必须与达妙上位机设置一致，其余参数使用子类默认值
    DM_Motor_Rotary.Init(&hfdcan2, DM_Motor_ID_0xA6, DM_Motor_Control_Method_POSITION_OMEGA, PI);
    // 电流直接采用C610映射值，电流计算与打包由夹爪专用子类负责
    DJI_Motor_Clamp.Init(&hfdcan1, DJI_Motor_ID_0x204, DJI_Motor_Control_Method_TORQUE, 36.0f);
    DJI_Motor_Clamp.Set_Out(0.0f);

    DJI_Motor_Clamp.PID_Omega.Init(GRIPPER_DJI_OMEGA_KP, 0.0f, 0.0f, 0.0f, 0.0f, GRIPPER_DJI_OUTPUT_MAX);
    Class_FSM::Init(Gripper_Clamp_Status_NUM, Gripper_Clamp_Status_DISABLED);
}

void Class_Gripper::Set_Target_Clamp_Current(float __Target_Clamp_Current)
{
    Math_Constrain(&__Target_Clamp_Current, 0.0f, 4000.0f);
    Target_Clamp_Current = __Target_Clamp_Current;
}

void Class_Gripper::Output()
{
    // 在线状态判断
    bool DM_Online = (DM_Motor_Rotary.Get_DM_Motor_Status() == DM_Motor_Status_ENABLE);
    bool DJI_Online = (DJI_Motor_Clamp.Get_DJI_Motor_Status() == DJI_Motor_Status_ENABLE);

    // 夹爪未使能 or 有电机离线 or 达妙电机通信未稳定时不使能电机
    if ((Gripper_Control_Type != Gripper_Control_Type_ENABLE) ||
        (Gripper_Clamp_Status == Gripper_Clamp_Status_DISABLED) ||
        !DM_Online || !DJI_Online || !DM_Motor_Rotary.Get_Radian_Initialized())
    {
        DM_Motor_Rotary.Set_DM_Control_Status(DM_Motor_Control_Status_DISABLE);
        DM_Motor_Rotary.Set_Target_Omega(0.0f);

        DJI_Motor_Clamp.Set_Current_Control(0.0f);
        return;
    }
    // 夹爪使能
    else
    {
        // 达妙电机使能
        DM_Motor_Rotary.Set_DM_Control_Status(DM_Motor_Control_Status_ENABLE);
        DM_Motor_Rotary.Set_DM_Motor_Control_Method(DM_Motor_Control_Method_POSITION_OMEGA);

        // 当夹爪运动到位后再允许旋转轴的达妙电机运行
        if ((Gripper_Clamp_Status == Gripper_Clamp_Status_OPENED) ||
            (Gripper_Clamp_Status == Gripper_Clamp_Status_GRIPPED))
        {
            DM_Motor_Rotary.Set_Target_Angle(Target_DM_Radian);
            DM_Motor_Rotary.Set_Target_Omega(Target_DM_Omega);
        }
        // 当夹爪运动中时不允许旋转轴的达妙电机运行
        else
        {
            DM_Motor_Rotary.Set_Target_Angle(Clamp_Hold_DM_Radian);
            DM_Motor_Rotary.Set_Target_Omega(0.0f);
        }

        // 夹持端不同状态下2006电机的对应行为
        switch (Gripper_Clamp_Status)
        {
        // 张合运行中状态
        case (Gripper_Clamp_Status_OPENING):
        case (Gripper_Clamp_Status_CLOSING):
        {
            // 设置张合角速度（张与合分别设置）
            float Motion_Omega = (Gripper_Clamp_Status == Gripper_Clamp_Status_OPENING) ? GRIPPER_OPEN_OMEGA : GRIPPER_CLOSE_OMEGA;

            // 超时检测，若机构损坏或电机数据错误，当运行中时间超出GRIPPER_MOTION_TIMEOUT时认为运行超时
            if (Status[Now_Status_Serial].Time >= GRIPPER_MOTION_TIMEOUT)
            {
                // 根据超时时长平缓降低角速度
                float Stop_Ratio = 1.0f - (float)(Status[Now_Status_Serial].Time - GRIPPER_MOTION_TIMEOUT) / (float)GRIPPER_MOTION_STOP_TIME;
                Math_Constrain(&Stop_Ratio, 0.0f, 1.0f);
                Motion_Omega *= Stop_Ratio;
            }

            // 2006电机输出值计算
            float Target_DJI_Omega_Radian = Motion_Omega + Now_Roll_Gear_Omega_Radian;
            DJI_Motor_Clamp.Set_Omega_Control(Target_DJI_Omega_Radian);
        }
        break;

        // 接触状态，此状态下仅输出夹持力用于建立夹持
        case (Gripper_Clamp_Status_CONTACT):
        {
            DJI_Motor_Clamp.Set_Current_Control(Contact_Current);
        }
        break;

        // 夹持状态
        case (Gripper_Clamp_Status_GRIPPED):
        {
            // 按设定的夹持基础电流叠加随动角速度环输出
            DJI_Motor_Clamp.Set_Clamp_Follow_Control(Target_Clamp_Current, Now_Roll_Gear_Omega_Radian);
        }
        break;

        // 张开状态
        case (Gripper_Clamp_Status_OPENED):
        {
            DJI_Motor_Clamp.Set_Current_Control(GRIPPER_OPEN_HOLD_CURRENT);
        }
        break;

        // 超时关闭输出
        case (Gripper_Clamp_Status_TIMEOUT):
        default:
        {
            DJI_Motor_Clamp.Set_Current_Control(0.0f);
        }
        break;
        }
    }
}

void Class_Gripper::TIM_Calculate_PeriodElapsedCallback()
{
    Calculate_Kinematics();
    // 夹持运动状态机
    Reload_TIM_Status_PeriodElapsedCallback();
    Output();

    DJI_Motor_Clamp.TIM_Control_PeriodElapsedCallback();
    DM_Motor_Rotary.TIM_Process_PeriodElapsedCallback();
}

// 夹持运动状态机
void Class_Gripper::Reload_TIM_Status_PeriodElapsedCallback()
{
    Status[Now_Status_Serial].Time++;

    // 在线状态检测
    bool DM_Online = (DM_Motor_Rotary.Get_DM_Motor_Status() == DM_Motor_Status_ENABLE);
    bool DJI_Online = (DJI_Motor_Clamp.Get_DJI_Motor_Status() == DJI_Motor_Status_ENABLE);
    // 电机掉线对应处理
    if (!DM_Online || !DJI_Online)
    {
        // 接触判定计数清零
        Contact_Confirm_Count = 0;
        // 当达妙电机与2006同时掉线时，圈数清零
        if (!DM_Online && !DJI_Online)
        {
            DM_Motor_Rotary.Reset_Multi_Turn_Radian();
            Now_Roll_Total_Radian = 0.0f;
        }
        // 将夹持状态设置为DISABLE
        if (Gripper_Clamp_Status != Gripper_Clamp_Status_DISABLED)
        {
            Set_Clamp_Status(Gripper_Clamp_Status_DISABLED);
        }
        return;
    }

    // 关闭状态，清空接触计数和积分项
    if (Gripper_Control_Type == Gripper_Control_Type_DISABLE)
    {
        Contact_Confirm_Count = 0;
        DJI_Motor_Clamp.Reset_Omega_PID();
        if (Gripper_Clamp_Status != Gripper_Clamp_Status_DISABLED)
        {
            Set_Clamp_Status(Gripper_Clamp_Status_DISABLED);
        }
        return;
    }

    // 当达妙电机通信未稳定时退出
    if (!DM_Motor_Rotary.Get_Radian_Initialized())
    {
        return;
    }

    // 状态机对应处理
    switch (Gripper_Clamp_Status)
    {
    case (Gripper_Clamp_Status_DISABLED):
    {
        // 开始进行运动，此时达妙电机（旋转端）保持当前位置，夹持端电机开始运动
        Start_Clamp_Motion(Gripper_Clamp_Command);
    }
    break;

    case (Gripper_Clamp_Status_OPENING):
    {
        // 接收到闭合指令时转为闭合
        if (Gripper_Clamp_Command == Gripper_Clamp_Command_CLOSE)
        {
            Start_Clamp_Motion(Gripper_Clamp_Command_CLOSE);
            break;
        }

        // 超时判定
        if (Status[Now_Status_Serial].Time >= (GRIPPER_MOTION_TIMEOUT + GRIPPER_MOTION_STOP_TIME))
        {
            Timeout_Clamp_Command = Gripper_Clamp_Command;
            Set_Clamp_Status(Gripper_Clamp_Status_TIMEOUT);
            break;
        }

        // 接触判定 进入状态的前50ms不进行判定，等电流稳定后再判定
        // 当相对角速度小于阈值且反馈电流幅值达到正阈值时计数器自增
        if ((Status[Now_Status_Serial].Time >= GRIPPER_CONTACT_IGNORE_TIME) &&
            (Status[Now_Status_Serial].Time < GRIPPER_MOTION_TIMEOUT) &&
            (Math_Abs(Gripper_Data.Now_Clamp_Omega_Radian) < GRIPPER_STALL_OMEGA) &&
            (Math_Abs(DJI_Motor_Clamp.Get_Now_Torque()) >= GRIPPER_OPEN_STALL_CURRENT))
        {
            Contact_Confirm_Count++;
        }
        else
        {
            Contact_Confirm_Count = 0;
        }

        // 大于阈值时间则转移到张开状态
        if (Contact_Confirm_Count >= GRIPPER_CONTACT_CONFIRM_TIME)
        {
            Enter_Roll_Enabled_Status(Gripper_Clamp_Status_OPENED);
        }
    }
    break;

    case (Gripper_Clamp_Status_OPENED):
    {
        // 接收到关闭指令时转移到关闭开始状态
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
            (Math_Abs(DJI_Motor_Clamp.Get_Now_Torque()) >= GRIPPER_CLOSE_STALL_CURRENT))
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
            Math_Constrain(&Contact_Start_Current, 0.0f, Target_Clamp_Current);
            Contact_Current = Contact_Start_Current;
            Contact_Confirm_Count = 0;
            Set_Clamp_Status(Gripper_Clamp_Status_CONTACT);
        }
    }
    break;

    // 接触状态，平滑增大接触电流
    case (Gripper_Clamp_Status_CONTACT):
    {
        if (Gripper_Clamp_Command == Gripper_Clamp_Command_OPEN)
        {
            Start_Clamp_Motion(Gripper_Clamp_Command_OPEN);
            break;
        }

        float Ramp_Ratio = (float)Status[Now_Status_Serial].Time / (float)GRIPPER_CONTACT_CURRENT_RAMP_TIME;
        Math_Constrain(&Ramp_Ratio, 0.0f, 1.0f);
        Contact_Current = Contact_Start_Current + (Target_Clamp_Current - Contact_Start_Current) * Ramp_Ratio;

        if ((Status[Now_Status_Serial].Time >= GRIPPER_CONTACT_CURRENT_RAMP_TIME) &&
            (Math_Abs(Gripper_Data.Now_Clamp_Omega_Radian) < GRIPPER_STALL_OMEGA) &&
            (Math_Abs(DJI_Motor_Clamp.Get_Now_Torque()) >= GRIPPER_STALL_CURRENT_RATIO * Target_Clamp_Current))
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
    // 电机轴数据先经外啮合传动换算为旋转齿轮数据，再参与机构解算
    float Now_DM_Radian = DM_Motor_Rotary.Get_Now_Multi_Turn_Radian();
    Now_Roll_Total_Radian = GRIPPER_DM_TO_ROLL_DIRECTION * GRIPPER_DM_TO_ROLL_RATIO * Now_DM_Radian;
    Now_Roll_Gear_Omega_Radian = GRIPPER_DM_TO_ROLL_DIRECTION * GRIPPER_DM_TO_ROLL_RATIO * DM_Motor_Rotary.Get_Now_Omega();

    // 正运动学解算出当前夹爪整体的数据
    Gripper_Data.Now_Roll_Radian = Normalize_Angle_Radian_PI_to_PI(Now_Roll_Total_Radian);
    Gripper_Data.Now_Roll_Omega_Radian = Now_Roll_Gear_Omega_Radian;
    Gripper_Data.Now_Clamp_Omega_Radian = DJI_Motor_Clamp.Get_Now_Omega_Radian() - Now_Roll_Gear_Omega_Radian;

    // 逆运动学：优弧目标先在旋转齿轮侧确定，再还原到DM电机轴
    if (DM_Motor_Rotary.Get_Radian_Initialized() &&
        ((Gripper_Clamp_Status == Gripper_Clamp_Status_OPENED) ||
         (Gripper_Clamp_Status == Gripper_Clamp_Status_GRIPPED)))
    {
        Target_Roll_Radian = Normalize_Angle_Radian_PI_to_PI(Target_Roll_Radian);
        float Delta_Target_Roll_Radian = Normalize_Angle_Radian_PI_to_PI(Target_Roll_Radian - Gripper_Data.Now_Roll_Radian);
        Target_DM_Radian = Now_DM_Radian + Delta_Target_Roll_Radian / (GRIPPER_DM_TO_ROLL_DIRECTION * GRIPPER_DM_TO_ROLL_RATIO);
        Target_DM_Omega = Target_Roll_Omega_Radian / (GRIPPER_DM_TO_ROLL_DIRECTION * GRIPPER_DM_TO_ROLL_RATIO);
    }
}

void Class_Gripper::Set_Clamp_Status(Enum_Gripper_Clamp_Status __Gripper_Clamp_Status)
{
    Gripper_Clamp_Status = __Gripper_Clamp_Status;
    Set_Status(static_cast<uint8_t>(__Gripper_Clamp_Status));
}

void Class_Gripper::Start_Clamp_Motion(Enum_Gripper_Clamp_Command __Gripper_Clamp_Command)
{
    Clamp_Hold_DM_Radian = DM_Motor_Rotary.Get_Now_Multi_Turn_Radian();
    Target_DM_Radian = Clamp_Hold_DM_Radian;
    Target_DM_Omega = 0.0f;
    Target_Roll_Radian = Gripper_Data.Now_Roll_Radian;
    Contact_Current = 0.0f;
    Contact_Confirm_Count = 0;
    DJI_Motor_Clamp.Reset_Omega_PID();

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
    Target_DM_Radian = DM_Motor_Rotary.Get_Now_Multi_Turn_Radian();
    Target_DM_Omega = 0.0f;
    Contact_Confirm_Count = 0;
    DJI_Motor_Clamp.Reset_Omega_PID();
    Set_Clamp_Status(__Gripper_Clamp_Status);
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
