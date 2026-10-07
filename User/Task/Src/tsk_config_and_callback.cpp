/**
 * @file tsk_config_and_callback.cpp
 * @author cjw by yssickjgd
 * @brief 临时任务调度测试用函数, 后续用来存放个人定义的回调函数以及若干任务
 * @version 0.1
 * @date 2025-07-1 0.1 26赛季定稿
 * @copyright ZLLC 2026
 */

/**
 * @brief 注意, 每个类的对象分为专属对象Specialized, 同类可复用对象Reusable以及通用对象Generic
 *
 * 专属对象:
 * 单对单来独打独
 * 比如交互类的底盘对象, 只需要交互对象调用且全局只有一个, 这样看来, 底盘就是交互类的专属对象
 * 这种对象直接封装在上层类里面, 初始化在上层类里面, 调用在上层类里面
 *
 * 同类可复用对象:
 * 各调各的
 * 比如电机的对象, 底盘可以调用, 云台可以调用, 而两者调用的是不同的对象, 这种就是同类可复用对象
 * 电机的pid对象也算同类可复用对象, 它们都在底盘类里初始化
 * 这种对象直接封装在上层类里面, 初始化在最近的一个上层专属对象的类里面, 调用在上层类里面
 *
 * 通用对象:
 * 多个调用同一个
 * 比如裁判系统对象, 底盘类要调用它做功率控制, 发射机构要调用它做出膛速度与射击频率的控制, 因此裁判系统是通用对象.
 * 这种对象以指针形式进行指定, 初始化在包含所有调用它的上层的类里面, 调用在上层类里面
 *
 */

/**
 * @brief TIM开头的默认任务均1ms, 特殊任务需额外标记时间
 *
 */

/* Includes ------------------------------------------------------------------*/

#include "tsk_config_and_callback.h"
#include "drv_bsp-boarda.h"
#include "drv_tim.h"
#include "dvc_boardc_bmi088.h"
#include "dvc_dmmotor.h"
#include "ita_chariot.h"
#include "dvc_imu.h"
#include "usbd_cdc.h"
#include "usbd_cdc_if.h"
#include "config.h"
#include "iwdg.h"
/* Private macros ------------------------------------------------------------*/

/* Private types -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

uint32_t init_finished = 0;
bool start_flag = 0;
// 机器人控制对象
Class_Chariot chariot;

/* Private function declarations ---------------------------------------------*/
/* Function prototypes -------------------------------------------------------*/

/**
 * @brief Chassis_CAN1回调函数
 *
 * @param CAN_RxMessage CAN1收到的消息
 */
#ifdef CHASSIS
void Chassis_Device_CAN1_Callback(Struct_CAN_Rx_Buffer *CAN_RxMessage)
{
    switch (CAN_RxMessage->Header.Identifier)
    {
#ifdef AGV
    case (0x201):
    {
        chariot.Chassis.Motor_Wheel[0].CAN_RxCpltCallback(CAN_RxMessage->Data);
    }
    break;
    case (0x202):
    {
        chariot.Chassis.Motor_Wheel[1].CAN_RxCpltCallback(CAN_RxMessage->Data);
    }
    break;
    case (0x203):
    {
        chariot.Chassis.Motor_Wheel[2].CAN_RxCpltCallback(CAN_RxMessage->Data);
    }
    break;
    case (0x204):
    {
        chariot.Chassis.Motor_Wheel[3].CAN_RxCpltCallback(CAN_RxMessage->Data);
    }
    break;
#endif

#ifdef OMNI_WHEEL
    case (0x201):
    {
        chariot.Chassis.Motor_Wheel[1].CAN_RxCpltCallback(CAN_RxMessage->Data);
    }
    break;
    case (0x202):
    {
        chariot.Chassis.Motor_Wheel[3].CAN_RxCpltCallback(CAN_RxMessage->Data);
    }
    break;
    case (0x203):
    {
        chariot.Chassis.Motor_Wheel[0].CAN_RxCpltCallback(CAN_RxMessage->Data);
    }
    break;
    case (0x204):
    {
        chariot.Chassis.Motor_Wheel[2].CAN_RxCpltCallback(CAN_RxMessage->Data);
    }
    break;
#endif
    }
}
#endif
/**
 * @brief Chassis_CAN2回调函数
 *
 * @param CAN_RxMessage CAN2收到的消息
 */
#ifdef CHASSIS
void Chassis_Device_CAN2_Callback(Struct_CAN_Rx_Buffer *CAN_RxMessage)
{
    switch (CAN_RxMessage->Header.Identifier)
    {
#ifdef AGV
    case (0x205):
    {
        chariot.Chassis.Motor_Steer[0].CAN_RxCpltCallback(CAN_RxMessage->Data);
    }
    break;
    case (0x206):
    {
        chariot.Chassis.Motor_Steer[1].CAN_RxCpltCallback(CAN_RxMessage->Data);
    }
    break;
    case (0x207):
    {
        chariot.Chassis.Motor_Steer[2].CAN_RxCpltCallback(CAN_RxMessage->Data);
    }
    break;
    case (0x208):
    {
        chariot.Chassis.Motor_Steer[3].CAN_RxCpltCallback(CAN_RxMessage->Data);
    }
    break;
    case (0xD1):
    {
        chariot.Chassis.Motor_Steer[0].MA600_Data_Process(CAN_RxMessage);
    }
    break;
    case (0xD2):
    {
        chariot.Chassis.Motor_Steer[1].MA600_Data_Process(CAN_RxMessage);
    }
    break;
    case (0xD3):
    {
        chariot.Chassis.Motor_Steer[2].MA600_Data_Process(CAN_RxMessage);
    }
    break;
    case (0xD4):
    {
        chariot.Chassis.Motor_Steer[3].MA600_Data_Process(CAN_RxMessage);
    }
    break;
#endif

    case (0x67): // 超电接收
    {
        chariot.Chassis.Supercap.CAN_RxCpltCallback(CAN_RxMessage->Data);
        break;
    }
    case (0x55):
    {
        chariot.Chassis.Supercap.CAN_RxCpltCallback(CAN_RxMessage->Data);
        break;
    }
    case (0x206):
    {
        chariot.Motor_Yaw.CAN_RxCpltCallback(CAN_RxMessage->Data);
        break;
    }
    }
}
#endif

#ifdef CHASSIS
void Chassis_Device_CAN3_Callback(Struct_CAN_Rx_Buffer *CAN_RxMessage)
{
    switch (CAN_RxMessage->Header.Identifier)
    {
    case (0x77): // 留给上板通讯
    {
        chariot.CAN_Chassis_Rx_Gimbal_Callback();
        break;
    }
    }
}
#endif
/**
 * @brief Gimbal_CAN1回调函数
 *
 * @param CAN_RxMessage CAN1收到的消息
 */
#ifdef GIMBAL
void Gimbal_Device_CAN1_Callback(Struct_CAN_Rx_Buffer *CAN_RxMessage)
{
    switch (CAN_RxMessage->Header.Identifier)
    {
    case (0x204):
    {
        chariot.Gimbal.Gripper.DJI_Motor_Clamp_CAN_RxCpltCallback(CAN_RxMessage->Data);
    }
    }
}
#endif

/**
 * @brief Gimbal_CAN2回调函数
 *
 * @param CAN_RxMessage CAN2收到的消息
 */
#ifdef GIMBAL
void Gimbal_Device_CAN2_Callback(Struct_CAN_Rx_Buffer *CAN_RxMessage)
{
    switch (CAN_RxMessage->Header.Identifier)
    {
    case (0x16):
    {
        chariot.Gimbal.Gripper.DM_Motor_Rotary_CAN_RxCpltCallback(CAN_RxMessage->Data);
    }
    }
}
#endif

#ifdef GIMBAL
void Gimbal_Device_CAN3_Callback(Struct_CAN_Rx_Buffer *CAN_RxMessage)
{
    switch (CAN_RxMessage->Header.Identifier)
    {
    case (0x11):
    {
        chariot.IMU.IMU_UpdateData(CAN_RxMessage->Data);
    }
    break;
    }
}
#endif
/**
 * @brief SPI5回调函数
 *
 * @param Tx_Buffer SPI5发送的消息
 * @param Rx_Buffer SPI5接收的消息
 * @param Length 长度
 */
// void Device_SPI5_Callback(uint8_t *Tx_Buffer, uint8_t *Rx_Buffer, uint16_t Length)
//{
//     if (SPI5_Manage_Object.Now_GPIOx == BoardA_MPU6500_CS_GPIO_Port && SPI5_Manage_Object.Now_GPIO_Pin == BoardA_MPU6500_CS_Pin)
//     {
//         boarda_mpu.SPI_TxRxCpltCallback(Tx_Buffer, Rx_Buffer);
//     }
// }

/**
 * @brief SPI1回调函数
 *
 * @param Tx_Buffer SPI1发送的消息
 * @param Rx_Buffer SPI1接收的消息
 * @param Length 长度
 */
void Device_SPI2_Callback(uint8_t *Tx_Buffer, uint8_t *Rx_Buffer, uint16_t Length)
{
}

/**
 * @brief UART5遥控器回调函数
 *
 * @param Buffer UART9收到的消息
 * @param Length 长度
 */
#ifdef GIMBAL
void DR16_UART5_Callback(uint8_t *Buffer, uint16_t Length)
{
    chariot.DR16.DR16_UART_RxCpltCallback(Buffer);
    // 底盘 云台 发射机构 的控制策略
    chariot.TIM_Control_Callback();
}
#endif
/**
 * @brief UART9遥控器回调函数
 *
 * @param Buffer UART9收到的消息
 * @param Length 长度
 */
#ifdef GIMBAL
void VT13_UART_Callback(uint8_t *Buffer, uint16_t Length)
{
    // 该图传链路的回调函数并未添加自定义控制器的协议
    chariot.VT13.VT13_UART_RxCpltCallback(Buffer);

    // 底盘 云台 发射机构 的控制策略
    if (*(Buffer + 0) == 0xA9 && *(Buffer + 1) == 0x53)
    {
        chariot.TIM_Control_Callback();
    }
}
#endif

/**
 * @brief IIC磁力计回调函数
 *
 * @param Buffer IIC收到的消息
 * @param Length 长度
 */
void Ist8310_IIC3_Callback(uint8_t *Tx_Buffer, uint8_t *Rx_Buffer, uint16_t Tx_Length, uint16_t Rx_Length)
{
}

/**
 * @brief UART裁判系统回调函数
 *
 * @param Buffer UART收到的消息
 * @param Length 长度
 */
#ifdef CHASSIS
void Referee_UART10_Callback(uint8_t *Buffer, uint16_t Length)
{
    chariot.Referee.UART_RxCpltCallback(Buffer, Length);
}
#endif

/**
 * @brief USB MiniPC回调函数
 *
 * @param Buffer USB收到的消息
 *
 * @param Length 长度
 */
#ifdef GIMBAL
void MiniPC_USB_Callback(uint8_t *Buffer, uint32_t Length)
{
    chariot.MiniPC.USB_RxCpltCallback(Buffer);
}
#endif

/**
 * @brief TIM4任务回调函数
 *
 */
void Task100us_TIM4_Callback()
{
#ifdef CHASSIS
    GraphicSendtask();
    static uint16_t Referee_Sand_Cnt = 0;
    // //暂无云台tim4任务
    if (Referee_Sand_Cnt % 50 == 1)
    {
        Task_Loop();
        Referee_Sand_Cnt = 0;
    }

    Referee_Sand_Cnt++;
#elif defined(GIMBAL)
    // 单给IMU消息开的定时器 ims
    chariot.Gimbal.Boardc_BMI.TIM_Calculate_PeriodElapsedCallback();
#endif
}

/**
 * @brief TIM5任务回调函数
 *
 */

void Task1ms_TIM5_Callback()
{
    init_finished++;
    if (init_finished > 2000 && start_flag == 0)
    {
        // Buzzer.Set_NowTask(BUZZER_DEVICE_OFFLINE_PRIORITY);
        // buzzer_setTask(&buzzer, BUZZER_CALIBRATED_PRIORITY);
        start_flag = 1;
    }
    /************ 判断设备在线状态判断 50ms (所有device:电机，遥控器，裁判系统等) ***************/

    chariot.TIM1msMod50_Alive_PeriodElapsedCallback();

    // buzzer_taskScheduler(&buzzer);
    /****************************** 交互层回调函数 1ms *****************************************/
    if (start_flag == 1)
    {
#ifdef GIMBAL
        chariot.FSM_Alive_Control.Reload_TIM_Status_PeriodElapsedCallback();
        chariot.FSM_Alive_Control_VT13.Reload_TIM_Status_PeriodElapsedCallback();
#endif
        chariot.TIM_Calculate_PeriodElapsedCallback();

        /****************************** 驱动层回调函数 1ms *****************************************/
        // 统一打包发送
        TIM_CAN_PeriodElapsedCallback();

        // TIM_UART_PeriodElapsedCallback();

        // 给上位机发数据
        TIM_USB_PeriodElapsedCallback(&MiniPC_USB_Manage_Object);

        static int mod5 = 0;
        mod5++;
        if (mod5 == 10) // 上下板通信 100hz
        {
#ifdef GIMBAL
            // 给下板发送数据
            chariot.CAN_Gimbal_Tx_Chassis_Callback();
            // chariot.CAN_Gimbal_Tx_Chassis_Callback_1();
#elif defined(CHASSIS)
            // 底盘给云台发消息
            chariot.CAN_Chassis_Tx_Gimbal_Callback();
            // chariot.CAN_Chassis_Tx_Gimbal_Callback_1();
#endif
            mod5 = 0;
        }
    }
}

/**
 * @brief 初始化任务
 *
 */
extern "C" void Task_Init()
{

    DWT_Init(480);

/********************************** 驱动层初始化 **********************************/
#ifdef CHASSIS

    // 集中总线can1/can2
    CAN_Init(&hfdcan1, Chassis_Device_CAN1_Callback);
    CAN_Init(&hfdcan2, Chassis_Device_CAN2_Callback);
    CAN_Init(&hfdcan3, Chassis_Device_CAN3_Callback);

    // 裁判系统
    UART_Init(&huart10, Referee_UART10_Callback, 128); // 并未使用环形队列 尽量给长范围增加检索时间 减少丢包

#ifdef POWER_LIMIT

#endif

#endif

#ifdef GIMBAL

    // 集中总线can1/can2
    CAN_Init(&hfdcan1, Gimbal_Device_CAN1_Callback);
    CAN_Init(&hfdcan2, Gimbal_Device_CAN2_Callback);
    CAN_Init(&hfdcan3, Gimbal_Device_CAN3_Callback);

    // c板陀螺仪spi外设
    SPI_Init(&hspi2, Device_SPI2_Callback);

// 遥控器接收
#ifdef USE_DR16
    UART_Init(&huart5, DR16_UART5_Callback, 18);
#endif // DR16
#ifdef IMAGE_VT13
    UART_Init(&huart9, VT13_UART_Callback, 30); // 串口句柄需要根据实际外设口修正，接收缓冲区设为30B是为了同时接收自定义控制器的数据包
#endif                                          // VT13

#endif

    // 定时器循环任务
    TIM_Init(&htim4, Task100us_TIM4_Callback);
    TIM_Init(&htim5, Task1ms_TIM5_Callback);

    /********************************* 设备层初始化 *********************************/

    // 设备层集成在交互层初始化中，没有显视地初始化

    /********************************* 交互层初始化 *********************************/

    __disable_irq();
    chariot.Init();
    __enable_irq();

    /********************************* 使能调度时钟 *********************************/

    HAL_TIM_Base_Start_IT(&htim4);
    HAL_TIM_Base_Start_IT(&htim5);
}

/**
 * @brief 前台循环任务
 *
 */
extern "C" void Task_Loop()
{
#ifdef GIMBAL

#endif
#ifdef CHASSIS
    if (start_flag == 1)
    {
        static float freq;
        static uint32_t time_s;
        freq = 1 / DWT_GetDeltaT(&time_s);

        JudgeReceiveData.robot_id = chariot.Referee.Get_ID();
        JudgeReceiveData.Pitch_Angle = chariot.Gimbal_Tx_Pitch_Angle; // pitch角度
        JudgeReceiveData.Bullet_Status = chariot.Bulletcap_Status;    // 弹舱
        JudgeReceiveData.Fric_Status = chariot.Fric_Status;           // 摩擦轮
        JudgeReceiveData.Minipc_Status = chariot.MiniPC_Status;       // 自瞄是否离线
        JudgeReceiveData.Booster_User_Control_Type = chariot.Booster_User_Control_Type;
        // JudgeReceiveData.Supercap_Energy = chariot.Chassis.Supercap.Get_Stored_Energy();    // 超级电容储能
        JudgeReceiveData.Supercap_Voltage = chariot.Chassis.Supercap.Get_Supercap_Charge_Percentage() / 100.0f; // 超级电容容量
        JudgeReceiveData.Chassis_Control_Type = chariot.Chassis.Get_Chassis_Control_Type();                     // 底盘控制模式
        JudgeReceiveData.Supercap_State = chariot.Sprint_Status;
        JudgeReceiveData.booster_fric_omega_left = chariot.Booster_fric_omega_left; // 左摩擦轮速度; // 左摩擦轮速度
        JudgeReceiveData.booster_fric_omega_right = chariot.Booster_fric_omega_right;
        JudgeReceiveData.Booster_bullet_num = chariot.Booster_bullet_num - chariot.Booster_bullet_num_before;
        JudgeReceiveData.Booster_17mm_Heat = chariot.Booster_Heat; // chariot.Referee.Get_Booster_17mm_1_Heat();
        JudgeReceiveData.Booster_17mm_Heat_Max = chariot.Referee.Get_Booster_17mm_1_Heat_Max();
        JudgeReceiveData.Minipc_Mode = chariot.MiniPC_Type;
        JudgeReceiveData.Antispin_Type = chariot.Antispin_Type;
        if (chariot.Referee_UI_Refresh_Status == Referee_UI_Refresh_Status_ENABLE)
            Init_Cnt = 10;
    }

#endif
}

/************************ COPYRIGHT(C) USTC-ROBOWALKER **************************/
