# Copilot Instructions for Gimbal Cooperation Project

## 项目架构概览
- 本项目为二维云台控制系统，核心基于 FreeRTOS 实时操作系统，结合 CAN 总线与 IMU（惯性测量单元）实现高精度运动控制。
- 主要控制流程：IMU 姿态解算 → 目标偏差计算（视觉/遥控）→ 双环控制（角度环+速度环）→ 电机驱动（CAN 总线）。
- 关键任务通过 FreeRTOS 定时调度，保证实时性与任务优先级。

## 关键目录与文件
- `Core/Inc/`：头文件，定义硬件接口、配置、结构体等。
- `Core/Instance/`：驱动与控制逻辑实现，如 `BMI088driver.*`（IMU）、`bsp_can.*`（CAN）、`Gimbal.*`（云台控制）、`pid.*`（PID 控制）、`motor.*`（电机控制）。
- `Core/Src/`：主入口与任务调度，如 `main.c`、`freertos.c`、`can.c`、`imu.c`。
- `Drivers/STM32F4xx_HAL_Driver/`：HAL 库，硬件抽象层。
- `MDK-ARM/`：Keil 工程文件，编译与调试配置。

## 控制与数据流
- IMU 数据采集通过定时器中断，实时更新姿态。
- 视觉任务与遥控输入分别提供目标偏差与期望角度。
- 云台控制任务（1kHz）实现双环 PID 控制，输出期望角速度与电机指令。
- 电机控制命令通过 CAN 总线发送，反馈数据用于闭环。

## 开发与调试流程
- **编译/调试**：推荐使用 Keil MDK-ARM，主工程文件为 `platform.uvprojx`。
- **任务周期**：通过 FreeRTOS `vTaskDelayUntil` 保证任务周期稳定。
- **调试接口**：可通过 USART/VOFA 实现数据输出与调试。
- **CAN 通信**：相关帧结构与协议见 `bsp_can.*`、`CAN_receive.*`。

## 项目约定与模式
- 双环控制：角度环（低带宽）+ 速度环（高带宽），均为 PID 实现。
- 任务优先级：IMU > 云台控制 > CAN 通信 > 视觉。
- 结构体与数据流定义集中于 `struct_typedef.h`。
- 所有硬件相关操作通过 HAL 库封装，禁止直接寄存器操作。
- 视觉与遥控输入可动态切换，详见 `remote_control.*`。

## 典型代码模式示例
```c
// IMU 姿态解算（定时器中断）
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
    // ...existing code...
    imu_update(); // 姿态解算与数据更新
}

// 云台控制任务（FreeRTOS）
void Gimbal_Task(void *argument) {
    // ...existing code...
    pid_angle(); // 角度环
    pid_speed(); // 速度环
    motor_set_current(); // 输出到电机
}
```

## 外部依赖与集成
- 依赖 STM32 HAL 库、FreeRTOS。
- 视觉模块需通过串口或 CAN 接入。
- 电机驱动需兼容 CAN 协议。

---
如需补充特殊约定、调试技巧或集成细节，请在此文档补充说明。
