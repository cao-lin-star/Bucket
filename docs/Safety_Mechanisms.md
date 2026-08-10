# 足浴桶安全机制实现说明

本文档基于当前工程代码整理，说明所有已实现的安全机制、AD 采样相关的电流/电压处理、对应代码位置，以及如果需要临时取消某项安全机制时应注释的代码块。

> 注意：取消安全机制会使加热、水泵、电机、UV、供电异常等风险失去软件保护，只建议在实验室调试环境短时间使用。

## 1. 安全机制总览

| 机制 | 触发条件 | 保护动作 | 主要代码 |
| --- | --- | --- | --- |
| AD 连续采样与滤波 | ADC1 6 通道循环 DMA；TIM3 水位脉冲捕获 | 生成温度、电池/输入电压、水泵/电机电流和水位快照 | `Hardware/sensor_acquisition.c`, `Application/sensor.c`, `Core/Src/adc.c` |
| 温度传感器异常 | NTC 温度无效或超出 0~85 C | 上报温度传感器错误；加热模块停机 | `Sensor_TaskProcess`, `Temp_Control_TaskProcess`, `SystemMonitor_TaskProcess` |
| 水位传感器异常 | TIM3 无脉冲超过 2 s，或频率不在 1000~40000 Hz | 水位和频率清零并上报水位传感器错误 | `Sensor_ProcessWaterCount`, `SystemMonitor_TaskProcess` |
| 缺水保护 | 加热/循环泵/UV 需要水但水位 < 1 L | 上报缺水；加热和 UV 会主动关闭 | `Temp_Control_TaskProcess`, `UV_TaskProcess`, `SystemMonitor_TaskProcess` |
| 超温保护 | 当前温度 >= 55 C | 加热停机，上报温度异常/加热模块故障 | `Temp_Control_TaskProcess`, `SystemMonitor_TaskProcess` |
| 加热超时保护 | 加热超过 15 min 仍低于目标回差温度 | 加热停机，上报加热模块故障 | `Temp_Control_TaskProcess` |
| 水泵过流保护 | 水泵运行时电流 > 3000 mA | 关闭水泵，上报水泵故障 | `PumpValve_TaskProcess` |
| 排水超时保护 | 排水超过 8 min 水位仍 > 0 L | 关闭水泵，上报水泵故障 | `PumpValve_TaskProcess` |
| 电机过流保护 | 电机运行时电流 > 2100 mA | 停止电机，上报电机故障 | `Motor_TaskProcess` |
| 电池电压保护 | 电池 < 20.0 V 或 > 26.0 V | 上报电池故障；低压时停止所有输出并进入低电状态 | `SystemMonitor_TaskProcess` |
| 主控通信超时保护 | 5 s 未收到主控合法帧 | 停止所有输出、清定时、进入待机 | `UART_Comm_HandleMainTimeout` |
| 定时关机保护 | 定时倒计时结束 | 停止所有输出、进入待机 | `SystemMonitor_TaskProcess` |
| 停止/待机/低功耗/复位命令 | 收到对应通信命令 | 停止所有输出，复位命令延时后重启 MCU | `UART_Comm_ProcessBucketCommand`, `UART_Comm_ProcessSystemCommand`, `SystemMonitor_TaskProcess` |
| UART 帧校验与 DMA 错误恢复 | 帧头/校验和错误或 UART DMA 异常 | 丢弃非法帧；DMA 错误时重启接收 | `UART_Comm_IsFrameValid`, `UART_Comm_ErrorCallback` |

## 2. 传感器采集与电流/电压处理

### 2.1 分层职责和 ADC 通道映射

`Core/Src/adc.c` 配置 ADC1 扫描、连续转换和 6 路循环 DMA。`Hardware/sensor_acquisition.c` 负责启动并滤波原始值，`Application/sensor.c` 负责业务换算和快照。

| DMA 下标 | Rank | ADC 通道 | 业务含义 |
| ---: | ---: | --- | --- |
| 0 | 1 | ADC_CHANNEL_4 / PA4 | DCIN 电压 |
| 1 | 2 | ADC_CHANNEL_5 / PA5 | 电池电压 |
| 2 | 3 | ADC_CHANNEL_7 / PA7 | 桶内 NTC |
| 3 | 4 | ADC_CHANNEL_8 / PB0 | 水泵电流 |
| 4 | 5 | ADC_CHANNEL_9 / PB1 | 电机电流 |
| 5 | 6 | ADC_CHANNEL_0 / PA0 | 电池 NTC |

水位不占用 ADC 通道，而是由 TIM3 通道 1 输入捕获统计脉冲。

### 2.2 采样启动和滤波

`Core/Src/main.c` 先执行 `MX_DMA_Init()`、`MX_ADC1_Init()`、`MX_TIM3_Init()`。随后 `Sensor_Init()` 调用 `SensorAcquisition_Init()`，完成 NTC 上电、ADC 校准、ADC DMA 启动和 TIM3 捕获启动。

`Sensor_TaskProcess()` 每 20 ms 调用采集层。六路 ADC 分别执行分母为 8 的一阶 IIR，首个样本直接装载：

```text
acc = acc - acc / 8 + raw
filtered_raw = acc / 8
```

应用层再把原始码换成 ADC 引脚毫伏：

```text
adc_mv = round(raw × SENSOR_ADC_VREF_MV / 4095)
```

### 2.3 水位、电压、电流和温度换算

水位脉冲按实际统计窗口换算为 Hz：

```text
frequency_hz = round(pulse_count × 1000 / elapsed_ms)
```

默认使用 1 s 窗口；2 s 未收到脉冲，或频率不在 1000~40000 Hz 时判为无效。有效频率再用分母为 8 的 IIR 滤波，并按空桶 26700 Hz、满桶 24700 Hz 线性换算为 0~20 L。

电池和 DCIN 按分压倍率还原，结果单位为 0.1 V：

```text
input_mv = adc_mv × divider_x100 / 100
decivolt = round(input_mv / 100)
```

水泵和电机电流按各自采样电阻及缩放参数换算为 mA。桶内和电池 NTC 使用 10 kΩ、B=3950、100 kΩ 上拉的 Beta 公式换算；ADC 原始值 `<= 5` 或 `>= 4090`，以及换算温度不在 0~85℃ 时判为无效。

更完整的公式和单位见 `docs/sensor_detection_principle.md`。

## 3. 分模块安全机制

### 3.1 加热安全

阈值定义在 `Application/temp_control.h`：

```c
#define TEMP_TARGET_MIN_C           35.0f
#define TEMP_TARGET_MAX_C           48.0f
#define TEMP_HIGH_CUTOFF_C          55.0f
#define TEMP_HEAT_TIMEOUT_MS        (15UL * 60UL * 1000UL)
```

目标温度会被限制在 35~48 C：

```c
temp_target_c = Temp_ClampTarget(target);
```

加热任务中的保护逻辑：

```c
if ((Sensor_IsTempSensorOk() == 0U) ||
    (current_temp >= TEMP_HIGH_CUTOFF_C) ||
    (Sensor_GetWaterLevelProtocol() < SENSOR_WATER_MIN_SAFE_LITERS))
{
  temp_fault = 1U;
  temp_enabled = 0U;
  Temp_SetHeatOutput(0U);
  return;
}
```

加热超时保护：

```c
if ((temp_heating != 0U) &&
    (temp_heat_start_tick != 0U) &&
    ((now - temp_heat_start_tick) > TEMP_HEAT_TIMEOUT_MS) &&
    (current_temp < (temp_target_c - TEMP_HYSTERESIS_LOW_C)))
{
  temp_fault = 1U;
  temp_enabled = 0U;
  Temp_SetHeatOutput(0U);
}
```

取消方式：

- 取消目标温度限制：注释 `Temp_SetTargetC()` 内的 `Temp_ClampTarget(target)`，改为直接赋值。
- 取消温度传感器/超温/缺水停加热：注释 `Temp_Control_TaskProcess()` 中第一段保护 `if (...) { ... return; }`。
- 取消加热超时：注释 `Temp_Control_TaskProcess()` 末尾的加热超时 `if (...) { ... }`。

### 3.2 水泵和阀安全

阈值定义在 `Hardware/pump_valve.h`：

```c
#define PUMP_CURRENT_MAX_MA          3000U
#define PUMP_VALVE_DRAIN_TIMEOUT_MS  (8UL * 60UL * 1000UL)
```

水泵过流保护：

```c
current_ma = Sensor_GetPumpCurrentMa();
if ((pump_mode != PUMP_VALVE_MODE_OFF) && (current_ma > PUMP_CURRENT_MAX_MA))
{
  pump_fault = 1U;
  PumpValve_SetMode(PUMP_VALVE_MODE_OFF);
  return;
}
```

排水超时保护：

```c
if ((pump_mode == PUMP_VALVE_MODE_DRAIN) &&
    ((HAL_GetTick() - pump_mode_start_tick) > PUMP_VALVE_DRAIN_TIMEOUT_MS) &&
    (Sensor_GetWaterLevelProtocol() > 0U))
{
  pump_fault = 1U;
  PumpValve_SetMode(PUMP_VALVE_MODE_OFF);
}
```

取消方式：

- 取消水泵过流：注释 `PumpValve_TaskProcess()` 中读取 `Sensor_GetPumpCurrentMa()` 后的过流 `if`。
- 取消排水超时：注释 `PumpValve_TaskProcess()` 中排水模式超时 `if`。

### 3.3 电机安全

阈值定义在 `Hardware/motor_control.h`：

```c
#define MOTOR_CURRENT_MAX_MA         2100U
```

电机过流保护：

```c
current_ma = Sensor_GetMotorCurrentMa();
if ((motor_running != 0U) && (current_ma > MOTOR_CURRENT_MAX_MA))
{
  motor_fault = 1U;
  Motor_Stop();
  return;
}
```

取消方式：

- 注释 `Motor_TaskProcess()` 中读取 `Sensor_GetMotorCurrentMa()` 后的过流 `if`。

### 3.4 UV 缺水保护

UV 运行时水位低于 1 L 会关闭 UV 并置故障：

```c
if ((uv_enabled != 0U) &&
    (Sensor_GetWaterLevelProtocol() < SENSOR_WATER_MIN_SAFE_LITERS))
{
  uv_fault = 1U;
  UV_Off();
}
```

取消方式：

- 注释 `UV_TaskProcess()` 中上述缺水保护 `if`。

### 3.5 系统监控与错误码

`SystemMonitor_TaskProcess()` 每 50 ms 运行一次，负责汇总错误码和部分全局保护。

错误码定义在 `Application/system_monitor.h`：

```c
#define BUCKET_ERR1_LACK_WATER       0x01U
#define BUCKET_ERR1_TEMP_ABNORMAL    0x04U
#define BUCKET_ERR1_TEMP_SENSOR      0x10U
#define BUCKET_ERR1_WATER_SENSOR     0x20U

#define BUCKET_ERR2_HEAT_MODULE      0x01U
#define BUCKET_ERR2_PUMP             0x02U
#define BUCKET_ERR2_MOTOR            0x08U
#define BUCKET_ERR2_UV               0x10U
#define BUCKET_ERR2_BATTERY          0x20U
```

缺水、水位传感器、温度传感器、超温错误上报：

```c
if ((active_needs_water != 0U) && (water_level < SENSOR_WATER_MIN_SAFE_LITERS))
{
  system_err1 |= BUCKET_ERR1_LACK_WATER;
}
if (Sensor_IsWaterSensorOk() == 0U)
{
  system_err1 |= BUCKET_ERR1_WATER_SENSOR;
}
if (Sensor_IsTempSensorOk() == 0U)
{
  system_err1 |= BUCKET_ERR1_TEMP_SENSOR;
}
if (Sensor_GetTemperatureC() >= TEMP_HIGH_CUTOFF_C)
{
  system_err1 |= BUCKET_ERR1_TEMP_ABNORMAL;
}
```

各执行器故障汇总：

```c
if (Temp_HasFault() != 0U)      system_err2 |= BUCKET_ERR2_HEAT_MODULE;
if (PumpValve_HasFault() != 0U) system_err2 |= BUCKET_ERR2_PUMP;
if (Motor_HasFault() != 0U)     system_err2 |= BUCKET_ERR2_MOTOR;
if (UV_HasFault() != 0U)        system_err2 |= BUCKET_ERR2_UV;
```

电池电压保护：

```c
if ((battery_dv != 0U) &&
    ((battery_dv < SYSTEM_BAT_LOW_DV) || (battery_dv > SYSTEM_BAT_HIGH_DV)))
{
  system_err2 |= BUCKET_ERR2_BATTERY;
  if (battery_dv < SYSTEM_BAT_LOW_DV)
  {
    system_main_status = BUCKET_STATUS_LOW_POWER;
    SystemMonitor_StopAllOutputs();
  }
}
```

定时关机：

```c
if ((system_timer_deadline_tick != 0UL) &&
    (SystemMonitor_GetTimerRemainingMin() == 0U))
{
  system_timer_deadline_tick = 0UL;
  SystemMonitor_StopAllOutputs();
  system_main_status = BUCKET_STATUS_STANDBY;
  system_sub_status = 0U;
}
```

取消方式：

- 只取消错误上报：注释对应的 `system_err1 |= ...` 或 `system_err2 |= ...` 行。
- 取消低电压自动停机：注释电池保护中 `if (battery_dv < SYSTEM_BAT_LOW_DV) { ... }`，保留 `BUCKET_ERR2_BATTERY` 可继续上报异常。
- 取消电池高低压上报：注释整个电池电压保护 `if ((battery_dv != 0U) && ...)`。
- 取消定时关机：注释定时关机 `if ((system_timer_deadline_tick != 0UL) && ...)`。

### 3.6 停止所有输出

所有全局停机都会调用：

```c
void SystemMonitor_StopAllOutputs(void)
{
  Temp_Enable(0U);
  Motor_Stop();
  PumpValve_SetMode(PUMP_VALVE_MODE_OFF);
  UV_Off();
}
```

该函数被低压保护、通信超时、停止/待机/低功耗命令、定时关机、复位命令等调用。

取消方式：

- 不建议直接注释 `SystemMonitor_StopAllOutputs()` 函数体，因为会影响所有停机路径。
- 如果只想取消某一路停机，应在对应调用处注释 `SystemMonitor_StopAllOutputs();`。

### 3.7 通信安全

主控 5 s 无合法命令帧时停机：

```c
if ((now - main_last_rx_tick) < UART_COMM_MAIN_TIMEOUT_MS)
{
  return;
}

SystemMonitor_StopAllOutputs();
SystemMonitor_SetBathTimer(0U);
SystemMonitor_SetCommand(UART_CMD_BUCKET_STOP);
SystemMonitor_SetMainStatus(BUCKET_STATUS_STANDBY, 0U);
```

帧校验：

```c
if ((frame[0] != UART_COMM_HEAD1) || (frame[1] != UART_COMM_HEAD2))
{
  return 0U;
}
return (UART_Comm_Checksum(frame) == frame[29]) ? 1U : 0U;
```

UART 错误恢复：

```c
UART_Comm_StopReceiveOne(&huart1);
UART_Comm_StartReceiveOne(&huart1, linux_rx_dma_buffer, &linux_rx_dma_pos);
```

取消方式：

- 取消主控通信超时停机：注释 `UART_Comm_TaskProcess()` 中的 `UART_Comm_HandleMainTimeout(now);`，或注释 `UART_Comm_HandleMainTimeout()` 内的停机代码。
- 不建议取消帧校验；如必须取消，修改 `UART_Comm_IsFrameValid()` 始终返回 `1U`，但这会让错误帧也执行命令。
- 不建议取消 UART 错误恢复，否则 DMA 接收异常后可能无法恢复通信。

## 4. 调度关系

`Core/Src/freertos.c` 中各安全任务周期如下：

```c
Sensor_TaskProcess();          // 20 ms
Temp_Control_TaskProcess();    // 100 ms
Motor_TaskProcess();           // 20 ms
PumpValve_TaskProcess();       // 50 ms
UV_TaskProcess();              // 500 ms
UART_Comm_TaskProcess();       // 50 ms
SystemMonitor_TaskProcess();   // 50 ms
```

如果要完全停止某模块的安全任务，可以注释对应任务中的 `xxx_TaskProcess()` 调用。但这样该模块的正常运行逻辑也可能一起停止，例如电机 PWM 刷新、通信状态上报、传感器数据更新等。

## 5. 推荐的取消策略

调试时建议按下面优先级处理：

1. 优先修改阈值，例如把 `MOTOR_CURRENT_MAX_MA`、`PUMP_CURRENT_MAX_MA`、`TEMP_HIGH_CUTOFF_C` 调高，而不是直接删除保护。
2. 如果必须取消，优先只注释某个保护 `if`，不要停整个任务。
3. 保留错误码上报，让上位机仍能看到异常。
4. 调试完成后恢复保护代码，并重点实测缺水、超温、过流、低压、通信超时五类场景。

