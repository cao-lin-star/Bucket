# 桶体软件重构架构

## 1. 目标

本工程按“硬件驱动、业务逻辑、通信协议、RTOS 调度”分层。第一阶段只调整目录和调用入口，保持原有控制算法、任务周期和通信行为不变。

## 2. 目录职责

```text
Core/             CubeMX 生成的启动、外设初始化和中断入口
Hardware/         传感器、泵阀、电机、UV、灯光等硬件控制
Application/      温控、充电、系统监控、日志等业务逻辑
Communication/    Linux/基站通信、协议解析和状态组包
RTOS/             FreeRTOS 各任务的周期执行入口
Middlewares/      FreeRTOS 内核
Drivers/          STM32 HAL 与 CMSIS
```

## 3. 依赖方向

```text
RTOS
  -> Application / Communication
  -> Hardware
  -> Core / HAL
```

约束：

- Hardware 不得包含 Application、Communication 或 RTOS 的头文件。
- Application 可以调用 Hardware，但不得直接调用 GPIO、ADC、UART 等 HAL 接口。
- Communication 只负责收发、解析、转发和组包；后续不应直接控制加热、水泵、电机和 UV。
- RTOS 只负责调度，不保存业务状态，也不编写控制条件。
- `Core/Src/freertos.c` 只保留 CubeMX 任务创建和到 `RTOS/rtos_tasks.c` 的薄包装。

## 4. 当前迁移状态

### Hardware

- `sensor.*`
- `pump_valve.*`
- `motor_control.*`
- `uv_lamp.*`
- `color_light.*`

### Application

- `app_bucket.*`：所有桶体模块的统一初始化入口
- `temp_control.*`：加热、恒温和加热前 5 秒管道排空
- `power_manager.*`：基站连接条件下的充电状态机
- `system_monitor.*`：状态、定时和故障监控
- `log.*`：运行日志

### Communication

- `uart_comm.*`：Linux 与基站通信协议
- `uart_port_config.h`：串口角色与波特率配置

### RTOS

- `rtos_tasks.*`：保持原有 8 个任务及其执行周期

## 5. 下一阶段建议

1. 从 `power_manager` 中拆出纯 `charger` 硬件驱动，让业务代码不再直接操作 `DCIN_ON`。
2. 从 `temp_control` 中拆出纯 `heater` 硬件驱动。
3. 将 `uart_comm` 中直接控制设备的代码改为调用 Application 命令接口。
4. 将 `system_monitor` 拆成系统状态和安全保护两个业务模块。
5. 最后再评估是否需要把 8 个 FreeRTOS 任务合并，避免目录迁移和调度变化同时发生。

