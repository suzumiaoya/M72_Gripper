# 代码迭代记录
- 经过26赛季联盟赛后，将稳定版本的部分库更新到main分支。
### 底盘部分
- 2026/4/5 底盘部分涉及的库已更新：裁判系统解包、超电通讯、功率限制，相关控制逻辑基本上已补充完成。（zxf）
### 云台部分
- 需要补充VT13与DR16的控制逻辑，主要是下边三个函数的内容，以及涉及的底层库代码。, 2026/4/5已经补充完成。（zxf）
```
/**
 * @brief 控制回调函数
 *
 */
#ifdef GIMBAL
void Class_Chariot::TIM_Control_Callback()
{
    // 判断DR16控制数据来源
    Judge_DR16_Control_Type();
    Judge_VT13_Control_Type();
    // 底盘，云台，发射机构控制逻辑
    Control_Chassis();
    Control_Gimbal();
    Control_Booster();
}
#endif

/**
 * @brief UART3遥控器回调函数
 *
 * @param Buffer UART1收到的消息
 * @param Length 长度
 */
#ifdef GIMBAL
void DR16_UART3_Callback(uint8_t *Buffer, uint16_t Length)
{

    chariot.DR16.DR16_UART_RxCpltCallback(Buffer);
    // 底盘 云台 发射机构 的控制策略

    chariot.TIM_Control_Callback();
}


void VT13_UART_Callback(uint8_t *Buffer, uint16_t Length)
{
    chariot.VT13.VT13_UART_RxCpltCallback(Buffer);

    // 底盘 云台 发射机构 的控制策略
    if (*(Buffer + 0) == 0xA9 && *(Buffer + 1) == 0x53)
    {
        chariot.TIM_Control_Callback();
    }
}
#endif
```
# 该框架的使用指南
- 开发环境为：keil5 、stm32cubemax、vscode、ozone
- 整体架构适配于步兵，外设的接口、上下板通讯协议需要使用者自行修改。
# 软件运行流程图
需要在vscode的Draw.io Integration插件中才能打开流程图
- [软件运行流程图](流程图.dio)
