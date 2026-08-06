# sensor.c 检测计算原理与公式说明

本文档说明 `Hardware/sensor.c` 中所有检测量的计算过程、公式来源，以及原理图中电阻配比对结果的影响。

参考文件：

- 代码：`Hardware/sensor.c`、`Hardware/sensor.h`
- ADC 配置：`Core/Src/adc.c`、`Foot_bath_robot1.ioc`
- 原理图：`g:\work\SMT32电路图\P_SCH桶体电控_V1.0.pdf` / `.SchDoc`

## 1. ADC 采样通道对应关系

`sensor.c` 使用 ADC1 扫描 + DMA 循环采样，共 6 路。ADC 的扫描顺序必须和 `SensorAdcChannel_t` 枚举一致：

| DMA 下标 | 枚举名 | MCU 引脚 | ADC 通道 | 信号名 | 计算用途 |
|---:|---|---|---|---|---|
| 0 | `SENSOR_ADC_WATER_LEVEL` | PA0 | ADC1_IN0 | `W_LEVEL` | 水位电压 -> 水量 |
| 1 | `SENSOR_ADC_DCIN_VOLT` | PA4 | ADC1_IN4 | `AD_DCIN` | DC 输入电压 |
| 2 | `SENSOR_ADC_BATTERY_VOLT` | PA5 | ADC1_IN5 | `AD_BAT_V` / `AD_BAT` | 电池电压 |
| 3 | `SENSOR_ADC_NTC_TEMP` | PA7 | ADC1_IN7 | `AD_NTC` | NTC 温度 |
| 4 | `SENSOR_ADC_PUMP_CURRENT` | PB0 | ADC1_IN8 | `AD_PUMP_I` / `AD_PUMP` | 水泵电流 |
| 5 | `SENSOR_ADC_MOTOR_CURRENT` | PB1 | ADC1_IN9 | `AD_MOTOR_I` | 电机电流 |

原理图中可以看到 `AD_DCIN`、`AD_BAT`、`W_LEVEL`、`AD_NTC`、`AD_PUMP` 网络名；未在当前原理图文本中检索到 `AD_MOTOR` 网络名，代码仍保留了 PB1/ADC1_IN9 作为电机电流通道。

## 2. ADC 原始值到电压

STM32F103 的 ADC 为 12 位，右对齐输出范围为 0~4095。代码中参考电压：

```c
#define SENSOR_ADC_VREF_MV 3300U
```

所以原始 ADC 值 `raw` 到采样点电压 `Vadc` 的公式为：

```text
Vadc_mV = raw * Vref_mV / 4095
```

代码实现：

```c
mv = ((uint32_t)raw * SENSOR_ADC_VREF_MV + 2047U) / 4095U;
```

其中 `+2047` 是四舍五入补偿。因为除数是 4095，加入约半个除数后再整除，可使整数计算更接近真实小数结果。

ADC 电压分辨率约为：

```text
3300 / 4095 = 0.8059 mV/LSB
```

这意味着每 1 个 ADC 计数约等于 0.806 mV。

## 3. 一阶递推滤波

每一路 ADC 都先经过一阶平均滤波：

```c
#define SENSOR_FILTER_DIV 8U
acc = acc - acc / 8 + raw;
filtered = acc / 8;
```

令 `y[n] = acc[n] / 8`，`x[n] = raw[n]`，可近似写成：

```text
y[n] = 7/8 * y[n-1] + 1/8 * x[n]
```

含义：

- 新采样值只占 1/8 权重。
- 历史值占 7/8 权重。
- 优点是抑制 ADC 抖动、电机/水泵开关噪声。
- 代价是响应会变慢，突变信号大约需要多次任务周期才能稳定。

第一次运行时：

```c
acc = raw * 8
```

这样 `filtered = raw`，避免刚开机从 0 慢慢爬升。

## 4. 水位检测：电压线性映射为水量

代码配置：

```c
#define SENSOR_WATER_EMPTY_MV        300U
#define SENSOR_WATER_FULL_MV         3000U
#define SENSOR_WATER_MAX_LITERS      20U
```

代码假设水位传感器输出电压与水量近似线性：

```text
Vempty = 300 mV
Vfull  = 3000 mV
Lmax   = 20 L
```

分段公式：

```text
Vadc <= Vempty 时：Liters = 0
Vadc >= Vfull  时：Liters = Lmax
中间区间：
Liters = (Vadc - Vempty) * Lmax / (Vfull - Vempty)
```

代入当前参数：

```text
Liters = (Vadc_mV - 300) * 20 / (3000 - 300)
       = (Vadc_mV - 300) * 20 / 2700
```

换算斜率：

```text
20 L / 2700 mV = 0.007407 L/mV
1 L 对应 135 mV
```

协议层整数水位：

```c
liters = (uint8_t)(water_liters + 0.5f);
```

即四舍五入到整数升，并限制不超过 20 L。

电阻配比影响：

- 如果水位传感器直接输出 0.3~3.0 V，则代码参数可直接使用。
- 如果原理图在 `W_LEVEL` 前还有串联电阻、分压电阻或 RC 滤波，ADC 实际读到的是分压后的电压，应先换算回传感器输出电压。
- 若传感器输出为 `Vsensor`，上臂电阻为 `Rtop`，下臂电阻为 `Rbottom`，ADC 接在中点，则：

```text
Vadc = Vsensor * Rbottom / (Rtop + Rbottom)
Vsensor = Vadc * (Rtop + Rbottom) / Rbottom
```

此时 `SENSOR_WATER_EMPTY_MV` 和 `SENSOR_WATER_FULL_MV` 应填 ADC 点电压，而不是传感器原始输出电压；否则水位会按分压比例整体偏小或偏大。

## 5. NTC 温度检测

### 5.1 原理图与代码参数

原理图中 NTC 区域可见：

- `CON11` 标注：`NTC 100K`
- 文本标注：`NTC-100K 3950`
- `R22` 标注：`100K`
- 网络：`EN_NTC`、`AD_NTC`

代码中配置：

```c
#define SENSOR_NTC_R0_OHMS      10000.0f
#define SENSOR_NTC_BETA         3950.0f
#define SENSOR_NTC_T0_K         298.15f
#define SENSOR_NTC_PULLUP_OHMS  10000.0f
```

表面看代码是 10K NTC + 10K 上拉，但温度计算实际依赖的是 `Rntc / R0` 的比例。若硬件是 100K NTC + 100K 上拉，并且 NTC 的 B 值仍为 3950，则 `Rpullup` 和 `R0` 同比例放大 10 倍，计算结果仍然等价。

也就是说：

```text
代码模型：Rpullup=10K, R0=10K
硬件模型：Rpullup=100K, R0=100K
```

只要二者比例一致，`Rntc/R0` 不变，B 值公式得到的温度基本一致。

### 5.2 分压公式来源

代码使用的公式：

```c
voltage_ratio = raw / 4095.0f;
ntc_ohms = Rpullup * voltage_ratio / (1.0f - voltage_ratio);
```

这对应如下电路模型：

```text
EN_NTC/3.3V -- Rpullup -- AD_NTC -- Rntc -- GND
```

ADC 采样点在上拉电阻和 NTC 中间，因此：

```text
Vadc = Vcc * Rntc / (Rpullup + Rntc)
```

令：

```text
k = Vadc / Vcc = raw / 4095
```

则：

```text
k = Rntc / (Rpullup + Rntc)
k * (Rpullup + Rntc) = Rntc
k * Rpullup = Rntc * (1 - k)
Rntc = Rpullup * k / (1 - k)
```

这就是代码中 NTC 电阻计算公式的来源。

### 5.3 B 值温度公式来源

NTC 的 B 值模型为：

```text
R(T) = R0 * exp[B * (1/T - 1/T0)]
```

其中：

- `R(T)`：温度为 `T` 时的 NTC 电阻。
- `R0`：标称温度 `T0` 下的 NTC 电阻。
- `T`、`T0`：开尔文温度，单位 K。
- `B`：热敏电阻 B 值。
- `T0 = 25 + 273.15 = 298.15 K`。

两边取自然对数：

```text
ln(R/R0) = B * (1/T - 1/T0)
```

整理：

```text
1/T = 1/T0 + ln(R/R0) / B
T = 1 / [1/T0 + ln(R/R0) / B]
```

代码实现：

```c
inv_t = (1.0f / SENSOR_NTC_T0_K)
      + (logf(ntc_ohms / SENSOR_NTC_R0_OHMS) / SENSOR_NTC_BETA);
temp_k = 1.0f / inv_t;
temp_c = temp_k - 273.15f;
```

### 5.4 NTC 电阻配比对温度的影响

若实际上拉电阻为 `Rpullup_actual`，代码使用 `Rpullup_code`，则代码计算出的电阻为：

```text
Rntc_calc = Rpullup_code * Vadc / (Vcc - Vadc)
```

真实 NTC 电阻为：

```text
Rntc_actual = Rpullup_actual * Vadc / (Vcc - Vadc)
```

两者关系：

```text
Rntc_calc / Rntc_actual = Rpullup_code / Rpullup_actual
```

温度公式用的是 `Rntc_calc / R0_code`。如果满足：

```text
Rpullup_code / R0_code = Rpullup_actual / R0_actual
```

则：

```text
Rntc_calc / R0_code = Rntc_actual / R0_actual
```

温度计算不受绝对阻值缩放影响。

当前代码 10K/10K 与原理图 100K/100K 都是 1:1，因此温度曲线在理论上是匹配的。需要注意的是，100K 阻值更高，ADC 输入源阻抗更高，更容易受漏电、ADC 采样电容、PCB 污染、噪声影响；代码使用 71.5 个 ADC 周期采样，已经比很短采样时间更稳一些。

### 5.5 异常判断

代码认为 ADC 原始值接近两端电源轨时无效：

```c
if ((raw <= 5U) || (raw >= 4090U))
{
  return SENSOR_TEMP_INVALID_C;
}
```

含义：

- `raw <= 5`：采样点接近 0 V，可能 NTC 短路到地、上拉未使能、线路异常。
- `raw >= 4090`：采样点接近 Vref，可能 NTC 开路、下端断线。

最终温度有效范围：

```c
temp_sensor_ok = ((temp_c > 0.0f) && (temp_c < 85.0f)) ? 1U : 0U;
```

即 0~85 摄氏度内认为传感器正常。

## 6. 电池电压与 DC 输入电压检测

代码中电压还原函数：

```c
input_mv = adc_mv * divider_x100 / 100;
decivolt = (input_mv + 50) / 100;
```

其中：

```c
#define SENSOR_BAT_DIVIDER_X100   1100U
#define SENSOR_DCIN_DIVIDER_X100  1100U
```

`divider_x100 = 1100` 表示放大 11.00 倍，也就是代码假设 ADC 前级分压关系为：

```text
Vadc = Vin / 11
Vin  = Vadc * 11
```

### 6.1 通用分压公式

若原理图为：

```text
Vin -- Rtop -- ADC -- Rbottom -- GND
```

则：

```text
Vadc = Vin * Rbottom / (Rtop + Rbottom)
Vin = Vadc * (Rtop + Rbottom) / Rbottom
```

代码中的宏应设置为：

```text
divider_x100 = 100 * (Rtop + Rbottom) / Rbottom
```

例如：

| Rtop | Rbottom | Vin/Vadc | `divider_x100` |
|---:|---:|---:|---:|
| 100K | 10K | 11.0 | 1100 |
| 20K | 10K | 3.0 | 300 |
| 20K | 20K | 2.0 | 200 |
| 100K | 20K | 6.0 | 600 |

### 6.2 原理图电阻值对比

原理图中 `AD_DCIN` / `AD_BAT` 区域可检索到：

- `R2 = 20kΩ`
- `R7 = 10kΩ`
- `R8 = 20kΩ`
- 网络名：`AD_DCIN`、`AD_BAT`

当前代码默认 `SENSOR_BAT_DIVIDER_X100 = 1100`、`SENSOR_DCIN_DIVIDER_X100 = 1100`，等价于 100K:10K 这类 11 倍分压。

因此需要结合原理图连线确认：

- 如果某一路实际是 `100K + 10K` 分压，代码 11 倍正确。
- 如果某一路实际是 `20K + 10K` 分压，应改为 3 倍，即 `divider_x100 = 300`。
- 如果某一路实际是 `20K + 20K` 分压，应改为 2 倍，即 `divider_x100 = 200`。

配比错误的影响是线性的：

```text
Vin_calc / Vin_actual = divider_code / divider_actual
```

例如实际是 3 倍分压，但代码按 11 倍还原，则：

```text
Vin_calc = Vin_actual * 11 / 3 = 3.667 * Vin_actual
```

电压会被严重高估。

### 6.3 0.1V 输出单位

代码返回的是 0.1 V，即 decivolt：

```text
decivolt = round(input_mv / 100)
```

例如：

```text
input_mv = 24120 mV
decivolt = 241
表示 24.1 V
```

`+50` 是为了 mV 转 0.1V 时四舍五入。

## 7. 水泵电流与电机电流检测

代码中水泵和电机电流共用同一个换算函数：

```c
current_ma = (adc_mv - SENSOR_CURRENT_ZERO_MV)
           * SENSOR_CURRENT_MA_PER_MV_X10 / 10;
```

当前宏：

```c
#define SENSOR_CURRENT_ZERO_MV          0U
#define SENSOR_CURRENT_MA_PER_MV_X10    10U
```

所以实际公式为：

```text
I_mA = Vadc_mV * 10 / 10 = Vadc_mV
```

即代码假设：

```text
1 mV -> 1 mA
1 V  -> 1 A
```

### 7.1 分流电阻采样公式

若电流通过采样电阻 `Rsense`，ADC 直接测量采样电阻两端电压，基本物理公式来自欧姆定律：

```text
V = I * R
I = V / R
```

若 `Vadc` 单位为 mV，`I` 单位为 mA，则：

```text
I_mA = Vadc_mV / Rsense_ohm
```

因为：

```text
V = Vadc_mV / 1000
I_A = V / Rsense
I_mA = I_A * 1000 = Vadc_mV / Rsense
```

如果原理图中采样电阻为 `0.3Ω`：

```text
I_mA = Vadc_mV / 0.3 = 3.333 * Vadc_mV
```

换算成代码宏：

```text
SENSOR_CURRENT_MA_PER_MV_X10 = 33
```

即约 `3.3 mA/mV`。若需要更精确，可用 333/100 这类更高精度的系数，但当前代码接口是 `x10`，只能表达到 0.1 mA/mV。

### 7.2 放大器或滤波网络的影响

若采样电阻后面还有运放/电流检测芯片，设电压增益为 `G`，则：

```text
Vadc = I * Rsense * G
I = Vadc / (Rsense * G)
```

用 mV/mA 形式：

```text
I_mA = Vadc_mV / (Rsense_ohm * G)
```

代码宏应满足：

```text
SENSOR_CURRENT_MA_PER_MV_X10 = 10 / (Rsense_ohm * G)
```

如果有零点偏置，例如霍尔电流传感器 0A 输出 1.65V，则还要设置：

```text
SENSOR_CURRENT_ZERO_MV = 1650
```

然后：

```text
I_mA = (Vadc_mV - Vzero_mV) * scale
```

当前代码零点为 0，适用于低边采样电阻这类 0A 接近 0V 的方案。

### 7.3 原理图电阻值对比

原理图中 `AD_PUMP` 附近可检索到：

- `R51 = 0.3Ω`
- `R47 = 100kΩ`
- 网络名：`AD_PUMP`

另一区域还能看到：

- `R38 = 0.3Ω`
- `R41 = 0.3Ω`
- `R39 = 100kΩ`

当前代码按 `1 mA/mV` 计算。如果 `AD_PUMP` 是直接读取 `0.3Ω` 分流电阻电压，则代码会低估电流：

```text
I_code / I_actual = 1 / 3.333 = 0.3
```

也就是真实 1A 时，0.3Ω 上电压约 300mV，代码会算成 300mA。

只有在以下情况之一成立时，当前 `1 mA/mV` 才是正确的：

- 实际采样电阻等效为 1Ω；
- 0.3Ω 后级电路存在约 1/0.3 = 3.333 倍的等效放大；
- 硬件并不是直接采样该 0.3Ω 电阻，`AD_PUMP` 前还有其它比例网络。

因此电流宏建议按最终 ADC 点实测标定：

```text
SENSOR_CURRENT_MA_PER_MV_X10 = round(10 * I_actual_mA / Vadc_mV)
```

例如实测水泵电流 1000mA，ADC 点电压 300mV：

```text
scale_x10 = round(10 * 1000 / 300) = 33
```

## 8. 传感器正常性判断

### 8.1 水位传感器

```c
water_sensor_ok = raw > 5 && raw < 4090;
```

含义是只判断 ADC 是否明显短到地或顶到 Vref，并不判断水位是否合理。对应电压约为：

```text
raw = 5    -> 5 * 3300 / 4095 = 4.0 mV
raw = 4090 -> 3296 mV
```

所以它只是电气开短路粗判。

### 8.2 温度传感器

温度传感器有两层判断：

1. ADC 原始值不能接近 0 或 4095。
2. 计算温度必须在 0~85 摄氏度之间。

这比水位判断更严格，因为 NTC 可以通过公式反推出实际温度。

## 9. 当前代码参数与原理图的校准关注点

| 检测项 | 代码当前假设 | 原理图可见信息 | 影响 |
|---|---|---|---|
| ADC 参考电压 | 3300 mV | MCU VDD/VDDA 供电决定 | Vref 偏差会线性影响所有电压类计算 |
| 水位 | 300~3000 mV -> 0~20 L | `W_LEVEL` 网络 | 需按实际空桶/满桶 ADC 电压标定 |
| NTC | 10K/B3950/10K 上拉 | `NTC 100K`、`R22=100K`、`B=3950` | 由于上拉和 R0 同比例，理论温度曲线仍匹配 |
| 电池电压 | 11 倍分压 | `AD_BAT` 区域可见 20K/10K/20K | 若实际不是 11 倍，电压会按比例错误 |
| DC 输入 | 11 倍分压 | `AD_DCIN` 区域可见 20K/10K/20K | 同上 |
| 水泵电流 | 1 mA/mV | `AD_PUMP` 附近可见 0.3Ω | 若直接采样 0.3Ω，应约为 3.3 mA/mV |
| 电机电流 | 1 mA/mV | 当前原理图文本未检索到 `AD_MOTOR` | 需要确认 PB1/ADC1_IN9 的实际硬件连接 |

## 10. 建议的实测校准方法

1. ADC 基准校准：用万用表测 VDDA，若不是 3.300V，应调整 `SENSOR_ADC_VREF_MV`。
2. 水位校准：空桶记录 `W_LEVEL` mV，满 20L 记录 `W_LEVEL` mV，分别写入 `SENSOR_WATER_EMPTY_MV` 和 `SENSOR_WATER_FULL_MV`。
3. 电池/DC 输入校准：用可调电源输入已知电压，读取 ADC mV，计算：

```text
divider_x100 = round(100 * Vin_mV / Vadc_mV)
```

4. 电流校准：串联电流表测真实电流，同时读取 ADC mV，计算：

```text
SENSOR_CURRENT_MA_PER_MV_X10 = round(10 * I_mA / (Vadc_mV - Vzero_mV))
```

5. NTC 校准：在 25 摄氏度附近确认 ADC 是否约为半量程。若硬件确认为 100K NTC + 100K 上拉，代码可保持 10K/10K；若不是 1:1 配比，需要同步修改 `SENSOR_NTC_R0_OHMS` 与 `SENSOR_NTC_PULLUP_OHMS`。

