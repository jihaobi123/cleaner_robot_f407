# cleaner_robot_f407

本工程为基于 STM32F407 + HAL 的清扫机器人下位机控制固件，包含：
- UART 通信协议解析（与 LubanCat 上位机/ROS2 交互）
- 差速驱动与清扫子系统控制（PWM + GPIO）
- 传感器中断采集（碰撞/悬崖）
- 简单状态机（前进/避障/避崖）

以下为模块划分、函数说明与配置方式的完整中文说明。

---

## 1. 工程结构与模块划分

### 1.1 主要文件

- `Core/Src/main.c`：主程序入口，初始化外设与模块，循环调用任务
- `Core/Inc/comm_luban.h` / `Core/Src/comm_luban.c`：LubanCat 通信协议
- `Core/Inc/bsp_drivetrain.h` / `Core/Src/bsp_drivetrain.c`：差速驱动控制
- `Core/Inc/bsp_cleaning.h` / `Core/Src/bsp_cleaning.c`：清扫系统控制
- `Core/Inc/bsp_sensors.h` / `Core/Src/bsp_sensors.c`：传感器中断采集
- `Core/Inc/robot_state_machine.h` / `Core/Src/robot_state_machine.c`：高层状态机

### 1.2 CubeMX 生成文件

- `Core/Src/gpio.c`：GPIO/EXTI 配置（传感器/风机 GPIO）
- `Core/Src/tim.c`：TIM2/TIM3/TIM4/TIM8 PWM 配置
- `Core/Src/usart.c`：USART3 配置（与 LubanCat 通信）
- `Core/Src/stm32f4xx_it.c`：中断处理函数

---

## 2. 硬件映射说明（PWM/GPIO）

### 2.1 差速驱动

- 右轮 PWM：TIM8 CH2 -> PC7
- 左轮 PWM：TIM3 CH2 -> PB5

### 2.2 清扫子系统

- 水泵 PWM：TIM3 CH1 -> PB4
- 刷子 PWM：TIM4 CH1 -> PD12
- 风机：PB3 GPIO 输出

### 2.3 传感器（EXTI）

- PC4：左碰撞（BUMP_LEFT）
- PC5：右碰撞（BUMP_RIGHT）
- PC7：左悬崖（CLIFF_LEFT）
- PB0：右悬崖（CLIFF_RIGHT）
- PB12：前悬崖（CLIFF_FRONT）

GPIO 均为上拉输入 + 上升沿中断。

---

## 3. LubanCat UART 通信协议

### 3.1 帧格式

```
Header (0xAA55, 2 bytes) + CMD_ID (1 byte) + PayloadLen (2 bytes, little-endian)
+ Payload + CRC16 (2 bytes, little-endian)
```

### 3.2 命令与回应

命令 ID：
- `0x01`：CMD_SEND_TASK
- `0x02`：CMD_REQUEST_STATUS
- `0x03`：CMD_CANCEL_TASK
- `0x04`：CMD_EMERGENCY_STOP

回应 ID：
- `0x81`：RESP_TASK_ACK
- `0x82`：RESP_STATUS
- `0xFF`：RESP_ERROR

### 3.3 Task 载荷结构

```
uint32 task_id
float   x
float   y
uint8   cleaning_type
uint8   priority
```

### 3.4 CRC16

当前实现使用 CRC16-IBM (poly 0xA001, init 0xFFFF)。如果协议使用不同 CRC，请修改 `comm_luban.c` 中的 `crc16_update()`。

---

## 4. 模块详细说明与 API

### 4.1 通信模块 comm_luban

初始化：
- `Comm_Luban_Init(&huart3)`：启动 UART 中断接收

轮询解析：
- `Comm_Luban_Poll()`：从环形缓冲区解析帧，并触发命令处理

任务获取：
- `Comm_Luban_GetNextTask(&task)`：返回最新 task，处理后清除标志

状态反馈：
- `Comm_Luban_SendStatus("OK")`：发送 RESP_STATUS

### 4.2 差速驱动 bsp_drivetrain

- `Drivetrain_Init()`：启动左右轮 PWM
- `Drivetrain_SetRaw(right, left)`：设置左右轮速度，范围 [-1.0, 1.0]
  - 正值表示前进
  - 负值表示反转（当前只计算幅值，方向 GPIO TODO）
- `Drivetrain_Stop()`：停止输出 PWM

### 4.3 清扫模块 bsp_cleaning

- `Cleaning_Init()`：启动泵/刷 PWM，关闭风机
- `Cleaning_SetBrush(bool on)`：刷子开关（80% 占空比）
- `Cleaning_SetPump(bool on)`：水泵开关（80% 占空比）
- `Cleaning_SetFan(bool on)`：风机 GPIO 输出

### 4.4 传感器模块 bsp_sensors

- `Sensors_Init()`：传感器初始化（GPIO 已配置可为空）
- `Sensors_Poll()`：读取中断事件并更新内部状态
- `Sensors_GetState()`：获取当前传感器状态
- `Sensors_ClearEvents()`：清除中断触发标志

EXTI 回调：
- `HAL_GPIO_EXTI_Callback()` 位于 `bsp_sensors.c`，用于设置传感器触发事件。

### 4.5 状态机 robot_state_machine

状态：
- `IDLE`
- `FORWARD`
- `AVOID_BUMP_LEFT / RIGHT`
- `AVOID_CLIFF_LEFT / RIGHT / FRONT`

逻辑说明：
- 正常前进：`Drivetrain_SetRaw(0.5, 0.5)`
- 左碰撞：右原地转一段时间
- 右碰撞：左原地转一段时间
- 左悬崖：右转后恢复前进
- 右悬崖：左转后恢复前进
- 前悬崖：后退 -> 右转 -> 前进

时间控制基于 `HAL_GetTick()`，不使用 RTOS。

---

## 5. main.c 集成说明

主函数流程：

1. 外设初始化（GPIO / TIM / USART）
2. 模块初始化
3. 主循环执行轮询任务

核心代码：

```
Drivetrain_Init();
Cleaning_Init();
Sensors_Init();
Comm_Luban_Init(&huart3);
RobotSM_Init();

while (1)
{
    Comm_Luban_Poll();
    Sensors_Poll();
    RobotSM_Update();
}
```

所有模块调用均放在 `/* USER CODE BEGIN */` 区域，保证 CubeMX 重新生成不覆盖。

---

## 6. 注意事项

- 若 `huart3` 未在 `main.c` 可见，需要在 `main.c` USER CODE 区域包含 `usart.h`。
- 如果使用不同引脚或 TIM 通道，请同步修改 `bsp_drivetrain.c` 和 `bsp_cleaning.c`。
- 传感器 EXTI 回调已在 `bsp_sensors.c` 实现，请确保 `stm32f4xx_it.c` 中 EXTI IRQHandler 调用了 `HAL_GPIO_EXTI_IRQHandler()`（已配置）。
- PWM 占空比依赖 ARR 值，默认 100% = ARR。

---

## 7. 推荐后续扩展

- 补充方向 GPIO 控制（正反转）
- 增加状态机与清扫子系统联动
- 加入任务队列与导航接口
- 增加故障状态与异常处理

---

如需扩展通信协议或增加控制功能，可继续在对应模块内迭代。
