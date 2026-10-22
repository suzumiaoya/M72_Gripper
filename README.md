# 代码迭代记录

- 2026/10/22：完善旋转夹爪电机子类的数据处理与夹持电流接口。
  - DM 子类直接使用原始16位位置，按 `[-PMAX, +PMAX]` 解码，并按 `2 * PMAX` 处理反馈过零；首次反馈保留范围内的多圈角度。`Init` 必填 CAN 配置、工作模式和 PMAX，位置偏移、速度上限、扭矩上限默认分别为 `0`、`50rad/s`、`5Nm`；PMAX、速度及扭矩上限需与达妙上位机设置一致。
  - M2006 子类新增专用 `Init`，固定基类 `Torque_Max=10000`，统一使用 CAN 电流映射值，`1000` 对应 `1A`。新增 `M2006_TORQUE_TO_CURRENT` 换算宏，按 `3A≈1Nm` 估算，Nm 数值乘以该宏可得到电流映射值。
  - 夹爪基础夹持电流统一命名为 `Target_Clamp_Current`，提供 `Set_Target_Clamp_Current` / `Get_Target_Clamp_Current`；默认值为 `4000`，setter 限幅为 `0～4000`，夹持电流叠加随动速度环后的总输出限幅仍为 `0～9000`。
  - 接触阶段的瞬时斜坡电流与夹持设定值分开保存，张开、失能及重新开始张合不覆盖设定值；夹紧确认电流阈值随设定值更新。删除重复的调试镜像，保留电机子类中的实际计算数据供调试。
  - 已通过 ARM 编译及桌面回归检查，覆盖默认夹持、电流设置限幅、状态切换、随动输出和不同 PMAX 下的双向过零。临时测试及编译产物位于 Git 忽略的 `build` 目录，不纳入提交。
- 2026/10/9：云台侧 UART5 遥控输入由 DR16 切换为独立 FS-i6/FS-iA6B 驱动，接收 32 字节 iBUS 帧。UART5 配置为 115200、8N1，并开启 RX 反相以适配板上的三极管反相器。FS-i6 原厂 6 通道配置中，CH5/CH6 分别通过 Aux. Channels 分配给 SWA/SWD；SWA 上拨失能、下拨使能，右摇杆左右控制夹爪旋转，SWD 上拨张开、下拨闭合。夹爪校准完成后默认张开。
- 经过26赛季联盟赛后，将稳定版本的部分库更新到main分支。

### TO DO

- [ ] 为 M2006 电机转动方向与夹持正方向之间的关系增加独立的方向宏定义，并将方向转换与相关正逆运动学解算、随动目标、接触/夹紧判定及电流限幅代码解耦，消除闭合方向固定为正电流的假设。

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
