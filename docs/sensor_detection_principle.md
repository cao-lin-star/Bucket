# 传感器采集与计算原理

本文档对应当前拆分后的传感器模块：

- `Core/Src/adc.c`：CubeMX 生成的 ADC1 和 DMA 基础配置。
- `Hardware/sensor_acquisition.c/.h`：NTC 供电、ADC 校准与 DMA 启动、6 路 ADC 原始值滤波、TIM3 水位脉冲采集。
- `Application/sensor.c/.h`：把滤波后的原始值换算为水量、温度、电压和电流，并维护 `SensorSnapshot_t`。

`Sensor_*` 公共接口保持不变，其他业务模块不直接访问 ADC、DMA 或 TIM3。

## 1. 六路 ADC 扫描顺序

ADC1 使用扫描、连续转换和循环 DMA，DMA 中每个下标与 regular Rank 固定对应：

| DMA 下标 | ADC Rank | MCU 引脚 / ADC 通道 | 采集层枚举 | 最终用途 |
| ---: | ---: | --- | --- | --- |
| 0 | 1 | PA4 / ADC1_IN4 | `SENSOR_ACQUISITION_ADC_DCIN_VOLT` | DCIN 输入电压 |
| 1 | 2 | PA5 / ADC1_IN5 | `SENSOR_ACQUISITION_ADC_BATTERY_VOLT` | 电池电压 |
| 2 | 3 | PA7 / ADC1_IN7 | `SENSOR_ACQUISITION_ADC_NTC_TEMP` | 桶内 NTC 温度 |
| 3 | 4 | PB0 / ADC1_IN8 | `SENSOR_ACQUISITION_ADC_PUMP_CURRENT` | 水泵电流 |
| 4 | 5 | PB1 / ADC1_IN9 | `SENSOR_ACQUISITION_ADC_MOTOR_CURRENT` | 按摩电机电流 |
| 5 | 6 | PA0 / ADC1_IN0 | `SENSOR_ACQUISITION_ADC_BAT_NTC_TEMP` | 电池 NTC 温度 |

修改 `Core/Src/adc.c` 或 CubeMX Rank 后，必须同步检查 `SensorAcquisitionAdcChannel_t`。水位不是 ADC 通道，而是 TIM3 通道 1 的输入捕获脉冲。

## 2. 初始化和任务调用顺序

`Core/Src/main.c` 先执行 CubeMX 生成的 `MX_DMA_Init()`、`MX_ADC1_Init()` 和 `MX_TIM3_Init()`。之后 `Sensor_Init()` 清空应用状态并调用 `SensorAcquisition_Init()`：

1. 打开桶内 NTC 和电池 NTC 分压网络供电。
2. 校准 ADC1。
3. 以 6 个 half-word 为缓冲区启动 ADC1 循环 DMA。
4. 启动 TIM3 通道 1 输入捕获中断。

`RTOS/rtos_tasks.c` 每 20 ms 调用一次 `Sensor_TaskProcess()`。采集层滤波仍由这个 20 ms 周期驱动，没有移入 DMA 中断，因此重构前后的滤波时间特性一致。

应用层先在私有工作快照中完成所有字段计算，最后通过 `vTaskSuspendAll()` / `xTaskResumeAll()` 暂停任务调度并整体复制到发布快照；`Sensor_GetSnapshot()` 在读侧也用相同方式复制，避免其他任务读到新旧周期混合数据。该同步方式不关闭外设中断，因此 ADC、DMA、UART 和定时器中断仍可及时响应。初始化发生在调度器启动前且尚无并发访问，此时直接建立初始发布快照，不调用调度器控制接口。

## 3. ADC 原始值滤波

每个通道各自维护一个累加器，滤波分母为 8：

```text
首次：acc = raw × 8
后续：acc[n] = acc[n-1] - acc[n-1] / 8 + raw[n]
输出：filtered_raw[n] = acc[n] / 8
```

近似浮点形式为：

```text
y[n] = y[n-1] + (x[n] - y[n-1]) / 8
```

首次样本直接装载，避免系统启动后从 0 缓慢爬升。`SensorAcquisition_GetFilteredRawSnapshot()` 一次复制六路滤波结果，单位仍是 ADC 码，范围为 0~4095。

## 4. ADC 原始值转毫伏

STM32F1 ADC 为 12 位，参考电压宏默认是 3300 mV：

```text
adc_mv = round(raw × SENSOR_ADC_VREF_MV / 4095)
```

代码用 `+2047` 后再除以 4095 实现整数四舍五入。结果是 ADC 引脚电压，单位 mV，不是分压前的实际输入电压。

## 5. 水位脉冲和水量

### 5.1 脉冲频率

`HAL_TIM_IC_CaptureCallback()` 仅在 TIM3 通道 1 捕获时累计脉冲，并记录最近一次脉冲的 HAL Tick。应用层使用实际统计时长换算频率：

```text
frequency_hz = round(pulse_count × 1000 / elapsed_ms)
```

默认规则：

- 统计窗口：1000 ms。
- 无脉冲超时：2000 ms。
- 有效频率：1000~40000 Hz。
- 频率无效时：频率、水量清零，`water_sensor_ok = 0`。
- 有效频率再进行分母为 8 的一阶 IIR 滤波，首个有效值直接装载。

### 5.2 频率转水量

当前标定值：空桶 26700 Hz，满桶 24700 Hz，最大水量 20 L。频率越低表示水量越多：

```text
liters = (EMPTY_COUNT - frequency_hz) × MAX_LITERS
         / (EMPTY_COUNT - FULL_COUNT)
```

边界处理：

- `frequency_hz >= 26700`：0 L。
- `frequency_hz <= 24700`：20 L。
- 中间区间：线性插值，并限制在 0~20 L。

通信协议值由 `round(water_liters)` 得到整数升，再限制到 20 L。

## 6. NTC 温度

桶内温度和电池温度共用同一换算函数，但分别使用 DMA 下标 2 和 5，并分别维护温度滤波状态。

默认参数：

```text
R0 = 10000 Ω
B = 3950 K
T0 = 298.15 K（25℃）
Rpullup = 100000 Ω
```

分压网络按“上拉电阻接 VREF、NTC 接地”计算：

```text
ratio = raw / 4095
Rntc = Rpullup × (1 - ratio) / ratio
1 / T = 1 / T0 + ln(Rntc / R0) / B
temperature_c = T - 273.15
```

有效性规则：

- `raw <= 5` 或 `raw >= 4090`：视为开路或短路，返回 `-100.0℃`。
- 换算后仅接受 `0℃ < temperature < 85℃`。
- 有效温度使用分母为 16 的一阶 IIR 滤波。
- 桶内和电池 NTC 分别更新 `temp_sensor_ok` 与 `battery_temp_sensor_ok`。

`Sensor_TaskProcess()` 中的局部变量 `temp_c` 被顺序复用：先保存桶温临时值，处理完成后再保存电池温度临时值，并不是同一个温度被计算两次。

## 7. 电池和 DCIN 电压

先将 ADC 码转换为引脚毫伏，再按分压倍率还原。`divider_x100 = 1100` 表示实际输入约为 ADC 引脚电压的 11 倍：

```text
input_mv = adc_mv × divider_x100 / 100
decivolt = round(input_mv / 100)
```

最终单位为 0.1 V，例如 `245` 表示 24.5 V。电池和 DCIN 当前都使用 11 倍分压参数，实际产品应以电阻和实测值校准。

## 8. 水泵和电机电流

### 8.1 水泵电流

默认参数为 50 mΩ 采样电阻和额外的 100 倍缩放：

```text
sense_current_ma = round(adc_mv × 1000 / 50)
pump_current_ma = round(sense_current_ma / 100)
```

### 8.2 电机电流

默认采样电阻为 150 mΩ：

```text
motor_current_ma = round(adc_mv × 1000 / 150)
```

两路结果都限制在 `uint16_t` 范围 0~65535 mA。上述参数必须与实际放大、分压和采样链路匹配，不能只根据采样电阻标称值推断。

## 9. 最终快照及单位

| 字段 | 含义 | 单位 / 范围 |
| --- | --- | --- |
| `raw[6]` | 六路滤波后 ADC 码 | 0~4095 |
| `millivolt[6]` | 六路 ADC 引脚电压 | mV |
| `water_liters` | 桶内水量 | L |
| `water_frequency_hz` | 滤波后水位脉冲频率 | Hz |
| `temperature_c` | 桶内温度 | ℃，无效时 -100.0 |
| `battery_temperature_c` | 电池温度 | ℃，无效时 -100.0 |
| `battery_decivolt` | 电池实际电压 | 0.1 V |
| `dcin_decivolt` | DCIN 实际电压 | 0.1 V |
| `pump_current_ma` | 水泵电流 | mA |
| `motor_current_ma` | 电机电流 | mA |
| `water_sensor_ok` | 水位脉冲有效标志 | 0/1 |
| `temp_sensor_ok` | 桶内 NTC 有效标志 | 0/1 |
| `battery_temp_sensor_ok` | 电池 NTC 有效标志 | 0/1 |

## 10. 函数职责

| 文件 | 函数 | 职责 |
| --- | --- | --- |
| `Hardware/sensor_acquisition.c` | `SensorAcquisition_Init` | 启动 NTC 供电、ADC DMA 和 TIM3 捕获 |
| `Hardware/sensor_acquisition.c` | `SensorAcquisition_TaskProcess` | 对六路 DMA ADC 原始值执行 IIR 滤波 |
| `Hardware/sensor_acquisition.c` | `SensorAcquisition_GetFilteredRawSnapshot` | 复制六路滤波后 ADC 原始值 |
| `Hardware/sensor_acquisition.c` | `SensorAcquisition_TakeWaterPulseCount` | 原子读取并清零水位脉冲计数 |
| `Application/sensor.c` | `Sensor_Init` | 初始化应用状态并调用采集层初始化 |
| `Application/sensor.c` | `Sensor_TaskProcess` | 生成完整业务值并在暂停任务调度期间发布快照 |
| `Application/sensor.c` | `Sensor_CalcWaterLiters` | 水位频率转 L |
| `Application/sensor.c` | `Sensor_CalcNtcTemperature` | ADC 码转 ℃ |
| `Application/sensor.c` | `Sensor_CalcScaledDecivolt` | ADC 引脚电压转 0.1 V |
| `Application/sensor.c` | `Sensor_CalcPumpCurrentMa` | 水泵采样电压转 mA |
| `Application/sensor.c` | `Sensor_CalcMotorCurrentMa` | 电机采样电压转 mA |
| `Application/sensor.c` | `Sensor_Get*` | 向其他模块提供快照字段 |

## 11. 校准建议

1. 用万用表测量 VDDA，必要时调整 `SENSOR_ADC_VREF_MV`。
2. 用已知 DCIN 和电池电压核对 `SENSOR_DCIN_DIVIDER_X100`、`SENSOR_BAT_DIVIDER_X100`。
3. 用已知电流校准两路电流采样参数，并同时考虑运放或滤波网络增益。
4. 分别记录实际空桶和满桶的稳定脉冲频率，再调整 `SENSOR_WATER_EMPTY_COUNT`、`SENSOR_WATER_FULL_COUNT`。
5. 用标准温度计核对 NTC 的 R0、B 值和上拉电阻，避免仅凭器件名称设置参数。
