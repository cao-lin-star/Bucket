#include "sensor.h"
#include "sensor_acquisition.h"
#include "FreeRTOS.h"
#include "task.h"
#include <math.h>
#include <string.h>

/* 水位频率与温度使用独立的一阶 IIR 滤波参数。 */
#define SENSOR_WATER_FILTER_DIV    8U
#define SENSOR_TEMP_FILTER_DIV     16U

/* 10 kΩ、B=3950 NTC 配合 100 kΩ 上拉电阻，Beta 模型参考温度为 25℃。 */
#define SENSOR_NTC_R0_OHMS         10000.0f
#define SENSOR_NTC_BETA            3950.0f
#define SENSOR_NTC_T0_K            298.15f
#define SENSOR_NTC_PULLUP_OHMS     100000.0f

/* 水位换算和最终传感器快照仅由传感器任务更新。 */
static uint32_t sensor_water_count_filter_acc;
static uint32_t sensor_water_window_start_tick;
static uint8_t sensor_water_window_started;
static uint8_t sensor_water_filter_ready;

/* 工作快照仅由传感器任务更新；发布快照只在暂停调度器期间整体替换。 */
static SensorSnapshot_t sensor_working_snapshot;
static SensorSnapshot_t sensor_snapshot;
static float sensor_temp_filter_c;
static uint8_t sensor_temp_filter_ready;
static float sensor_battery_temp_filter_c;
static uint8_t sensor_battery_temp_filter_ready;

static void Sensor_PublishSnapshot(void)
{
  /* 仅阻止任务切换；外设中断继续响应，窗口内不调用任何阻塞接口。 */
  vTaskSuspendAll();
  sensor_snapshot = sensor_working_snapshot;
  (void)xTaskResumeAll();
}

/*
 * 功能：将 STM32F1 的 12 位 ADC 原始码转换为 ADC 引脚电压，单位 mV。
 * 公式：millivolt = round(raw * SENSOR_ADC_VREF_MV / 4095)。
 * 其中 raw 范围为 0~4095，SENSOR_ADC_VREF_MV 默认为 3300 mV。
 */
static uint16_t Sensor_RawToMv(uint16_t raw)
{
  uint32_t mv;

  /* 加半个除数实现整数四舍五入；STM32F1 ADC 为 12 bit。 */
  mv = ((uint32_t)raw * SENSOR_ADC_VREF_MV + 2047U) / 4095U;
  return (uint16_t)mv;
}

static float Sensor_ClampFloat(float value, float min_value, float max_value)
{
  if (value < min_value)
  {
    return min_value;
  }
  if (value > max_value)
  {
    return max_value;
  }
  return value;
}

/*
 * 功能：将水位传感器脉冲频率换算为桶内水量，单位 L。
 * 公式：liters = (EMPTY_COUNT - frequency) * MAX_LITERS /
 *                 (EMPTY_COUNT - FULL_COUNT)。
 * 频率 >= EMPTY_COUNT 时输出 0 L；频率 <= FULL_COUNT 时输出最大水量。
 */
static float Sensor_CalcWaterLiters(uint32_t count_per_second)
{
  float liters;

  /* 标定特性反向：空桶频率高，满桶频率低；区间外直接饱和。 */
  if (count_per_second >= SENSOR_WATER_EMPTY_COUNT)
  {
    return 0.0f;
  }
  if (count_per_second <= SENSOR_WATER_FULL_COUNT)
  {
    return (float)SENSOR_WATER_MAX_LITERS;
  }

  liters = ((float)(SENSOR_WATER_EMPTY_COUNT - count_per_second) *
            (float)SENSOR_WATER_MAX_LITERS) /
           (float)(SENSOR_WATER_EMPTY_COUNT - SENSOR_WATER_FULL_COUNT);

  return Sensor_ClampFloat(liters, 0.0f, (float)SENSOR_WATER_MAX_LITERS);
}

/*
 * 功能：将 NTC 通道 ADC 原始码换算为摄氏温度，可用于桶温和电池温度。
 * 分压公式：ratio = raw / 4095，Rntc = Rpullup * (1 - ratio) / ratio。
 * Beta 公式：1/T = 1/T0 + ln(Rntc/R0) / B，temperature_c = T - 273.15。
 * ADC 接近 0 或 4095 时视为开路/短路，返回 SENSOR_TEMP_INVALID_C。
 */
static float Sensor_CalcNtcTemperature(uint16_t raw)
{
  float voltage_ratio;
  float ntc_ohms;
  float inv_t;
  float temp_k;

  /* 接近 ADC 两端通常意味着 NTC 开路/短路，不能代入对数公式。 */
  if ((raw <= 5U) || (raw >= 4090U))
  {
    return SENSOR_TEMP_INVALID_C;
  }

  /* 上拉接 VREF、NTC 接地：Rntc=Rpullup*(1-ratio)/ratio。 */
  voltage_ratio = (float)raw / 4095.0f;
  ntc_ohms = SENSOR_NTC_PULLUP_OHMS * (1.0f - voltage_ratio) / voltage_ratio;
  if (ntc_ohms <= 1.0f)
  {
    return SENSOR_TEMP_INVALID_C;
  }

  inv_t = (1.0f / SENSOR_NTC_T0_K) + (logf(ntc_ohms / SENSOR_NTC_R0_OHMS) / SENSOR_NTC_BETA);
  if (inv_t <= 0.0f)
  {
    return SENSOR_TEMP_INVALID_C;
  }

  temp_k = 1.0f / inv_t;
  return temp_k - 273.15f;
}

/*
 * 功能：将分压后的 ADC 引脚电压还原为电池或 DCIN 实际电压，单位 0.1 V。
 * 公式：input_mv = adc_mv * divider_x100 / 100；
 *       decivolt = round(input_mv / 100)。
 * divider_x100=1100 表示实际输入电压约为 ADC 引脚电压的 11 倍。
 */
static uint16_t Sensor_CalcScaledDecivolt(uint16_t adc_mv, uint16_t divider_x100)
{
  uint32_t input_mv;

  input_mv = ((uint32_t)adc_mv * divider_x100) / 100U;
  return (uint16_t)((input_mv + 50U) / 100U);
}

/*
 * 功能：将水泵电流采样通道电压换算为水泵电流，单位 mA。
 * 公式：sense_current_ma = adc_mv * 1000 / sense_resistor_mohm；
 *       pump_current_ma = round(sense_current_ma / scale_div)。
 * 当前参数为 30 mΩ 采样电阻，并按硬件采样链路再缩小 100 倍。
 */
static uint16_t Sensor_CalcPumpCurrentMa(uint16_t adc_mv)
{
  uint32_t current_ma;

  if ((SENSOR_PUMP_CURRENT_SENSE_MOHM == 0U) ||
      (SENSOR_PUMP_CURRENT_SCALE_DIV == 0U))
  {
    return 0U;
  }

  /* R61=0.03Ω 低边采样；按采样电阻算出的值再缩小 100 倍为实际电流。 */
  current_ma = ((uint32_t)adc_mv * 1000U +
                (SENSOR_PUMP_CURRENT_SENSE_MOHM / 2U)) /
               SENSOR_PUMP_CURRENT_SENSE_MOHM;

  current_ma = (current_ma + (SENSOR_PUMP_CURRENT_SCALE_DIV / 2U)) /
               SENSOR_PUMP_CURRENT_SCALE_DIV;
  if (current_ma > 65535U)
  {
    current_ma = 65535U;
  }

  return (uint16_t)current_ma;
}

/*
 * 功能：将按摩电机电流采样通道电压换算为电机电流，单位 mA。
 * 公式：motor_current_ma = round(adc_mv * 1000 /
 *                                  SENSOR_MOTOR_CURRENT_SENSE_MOHM)。
 * 当前采样电阻为 150 mΩ，计算结果超过 uint16_t 范围时限制为 65535。
 */
static uint16_t Sensor_CalcMotorCurrentMa(uint16_t adc_mv)
{
  uint32_t current_ma;

  if (SENSOR_MOTOR_CURRENT_SENSE_MOHM == 0U)
  {
    return 0U;
  }

  current_ma = ((uint32_t)adc_mv * 1000U + (SENSOR_MOTOR_CURRENT_SENSE_MOHM / 2U)) /
               SENSOR_MOTOR_CURRENT_SENSE_MOHM;
  if (current_ma > 65535U)
  {
    current_ma = 65535U;
  }

  return (uint16_t)current_ma;
}

/*
 * 功能：把统计窗口内的水位脉冲数转换为 Hz，校验后滤波并计算水量。
 * 频率公式：frequency_hz = round(pulse_count * 1000 / elapsed_ms)。
 * IIR 公式：filtered += (sample - filtered) / SENSOR_WATER_FILTER_DIV，当前分母为 8。
 */
static void Sensor_ProcessWaterCount(uint32_t now_tick)
{
  uint32_t elapsed_ms;
  uint32_t pulse_count;
  uint32_t count_per_second;

  /* 第一次调用只建立时间基准，丢弃初始化前可能积累的脉冲。 */
  if (sensor_water_window_started == 0U)
  {
    (void)SensorAcquisition_TakeWaterPulseCount();
    sensor_water_window_start_tick = now_tick;
    sensor_water_window_started = 1U;
    return;
  }

  elapsed_ms = now_tick - sensor_water_window_start_tick;
  /* 窗口未结束时仍检查掉线，确保拔掉传感器后尽快报无效。 */
  if (elapsed_ms < SENSOR_WATER_COUNT_WINDOW_MS)
  {
    if ((SensorAcquisition_GetWaterLastPulseTick() == 0UL) ||
        ((now_tick - SensorAcquisition_GetWaterLastPulseTick()) >
         SENSOR_WATER_COUNT_TIMEOUT_MS))
    {
      sensor_water_filter_ready = 0U;
      sensor_water_count_filter_acc = 0UL;
      sensor_working_snapshot.water_frequency_hz = 0UL;
      sensor_working_snapshot.water_liters = 0.0f;
      sensor_working_snapshot.water_sensor_ok = 0U;
    }
    return;
  }

  pulse_count = SensorAcquisition_TakeWaterPulseCount();
  sensor_water_window_start_tick = now_tick;

  /* 按实际窗口长度归一化为 Hz，允许任务调度带来少量周期抖动。 */
  count_per_second = (pulse_count * 1000UL + (elapsed_ms / 2UL)) / elapsed_ms;
  if ((SensorAcquisition_GetWaterLastPulseTick() == 0UL) ||
      ((now_tick - SensorAcquisition_GetWaterLastPulseTick()) >
       SENSOR_WATER_COUNT_TIMEOUT_MS) ||
      (count_per_second < SENSOR_WATER_COUNT_MIN_VALID) ||
      (count_per_second > SENSOR_WATER_COUNT_MAX_VALID))
  {
    sensor_water_filter_ready = 0U;
    sensor_water_count_filter_acc = 0UL;
    sensor_working_snapshot.water_frequency_hz = 0UL;
    sensor_working_snapshot.water_liters = 0.0f;
    sensor_working_snapshot.water_sensor_ok = 0U;
    return;
  }

  /* 首个有效值直接装载，后续以 1/8 权重抑制水面晃动。 */
  if (sensor_water_filter_ready == 0U)
  {
    sensor_water_count_filter_acc = count_per_second * SENSOR_WATER_FILTER_DIV;
    sensor_water_filter_ready = 1U;
  }
  else
  {
    sensor_water_count_filter_acc = sensor_water_count_filter_acc -
                                    (sensor_water_count_filter_acc / SENSOR_WATER_FILTER_DIV) +
                                    count_per_second;
  }

  sensor_working_snapshot.water_frequency_hz = sensor_water_count_filter_acc / SENSOR_WATER_FILTER_DIV;
  sensor_working_snapshot.water_liters = Sensor_CalcWaterLiters(sensor_working_snapshot.water_frequency_hz);
  sensor_working_snapshot.water_sensor_ok = 1U;
}

/*
 * 功能：校验并滤波桶内 NTC 温度，生成最终 temperature_c 和有效标志。
 * IIR 公式：filtered += (temp_c - filtered) / SENSOR_TEMP_FILTER_DIV，
 * 当前分母为 16；仅接受 0℃ < temperature_c < 85℃。
 */
static void Sensor_ProcessTemperature(float temp_c)
{
  /* 0~85℃为认可工作区间；越界即清除滤波历史并报告故障。 */
  if ((temp_c > 0.0f) && (temp_c < 85.0f))
  {
    if (sensor_temp_filter_ready == 0U)
    {
      sensor_temp_filter_c = temp_c;
      sensor_temp_filter_ready = 1U;
    }
    else
    {
      sensor_temp_filter_c += (temp_c - sensor_temp_filter_c) / (float)SENSOR_TEMP_FILTER_DIV;
    }

    sensor_working_snapshot.temperature_c = sensor_temp_filter_c;
    sensor_working_snapshot.temp_sensor_ok = 1U;
  }
  else
  {
    sensor_temp_filter_ready = 0U;
    sensor_temp_filter_c = SENSOR_TEMP_INVALID_C;
    sensor_working_snapshot.temperature_c = SENSOR_TEMP_INVALID_C;
    sensor_working_snapshot.temp_sensor_ok = 0U;
  }
}

/*
 * 功能：校验并滤波电池 NTC 温度，生成 battery_temperature_c 和有效标志。
 * IIR 公式：filtered += (temp_c - filtered) / SENSOR_TEMP_FILTER_DIV，
 * 当前分母为 16；仅接受 0℃ < battery_temperature_c < 85℃。
 */
static void Sensor_ProcessBatteryTemperature(float temp_c)
{
  if ((temp_c > 0.0f) && (temp_c < 85.0f))
  {
    if (sensor_battery_temp_filter_ready == 0U)
    {
      sensor_battery_temp_filter_c = temp_c;
      sensor_battery_temp_filter_ready = 1U;
    }
    else
    {
      sensor_battery_temp_filter_c += (temp_c - sensor_battery_temp_filter_c) / (float)SENSOR_TEMP_FILTER_DIV;
    }

    sensor_working_snapshot.battery_temperature_c = sensor_battery_temp_filter_c;
    sensor_working_snapshot.battery_temp_sensor_ok = 1U;
  }
  else
  {
    sensor_battery_temp_filter_ready = 0U;
    sensor_battery_temp_filter_c = SENSOR_TEMP_INVALID_C;
    sensor_working_snapshot.battery_temperature_c = SENSOR_TEMP_INVALID_C;
    sensor_working_snapshot.battery_temp_sensor_ok = 0U;
  }
}

/* 初始化应用层滤波状态和最终值，再启动底层 ADC/DMA 与水位脉冲采集。 */
void Sensor_Init(void)
{
  memset(&sensor_working_snapshot, 0, sizeof(sensor_working_snapshot));
  memset(&sensor_snapshot, 0, sizeof(sensor_snapshot));

  sensor_working_snapshot.temperature_c = SENSOR_TEMP_INVALID_C;
  sensor_working_snapshot.battery_temperature_c = SENSOR_TEMP_INVALID_C;

  sensor_temp_filter_c = SENSOR_TEMP_INVALID_C;
  sensor_temp_filter_ready = 0U;
  sensor_battery_temp_filter_c = SENSOR_TEMP_INVALID_C;
  sensor_battery_temp_filter_ready = 0U;

  sensor_water_count_filter_acc = 0UL;
  sensor_water_window_start_tick = 0UL;
  sensor_water_window_started = 0U;
  sensor_water_filter_ready = 0U;

  /* 此时调度器尚未启动且不存在并发访问，直接建立初始发布快照。 */
  sensor_snapshot = sensor_working_snapshot;
  SensorAcquisition_Init();
}

/*
 * 功能：读取硬件采集层提供的 6 路滤波后 ADC 原始值，并生成本周期最终快照。
 * 原始值到毫伏公式：millivolt = round(raw * SENSOR_ADC_VREF_MV / 4095)；
 * 随后依次完成水位、实际电压、电流、桶温和电池温度换算。
 */
void Sensor_TaskProcess(void)
{
  uint8_t index;
  uint16_t mv;
  float temp_c;
  uint32_t now_tick;

  /* 采集层先刷新 6 路滤波结果，再一次性复制到本周期快照。 */
  SensorAcquisition_TaskProcess();
  SensorAcquisition_GetFilteredRawSnapshot(sensor_working_snapshot.raw);

  for (index = 0U; index < SENSOR_ADC_CHANNEL_COUNT; index++)
  {
    mv = Sensor_RawToMv(sensor_working_snapshot.raw[index]);
    sensor_working_snapshot.millivolt[index] = mv;
  }

  now_tick = HAL_GetTick();
  Sensor_ProcessWaterCount(now_tick);

  /* 将通用采样值转换为业务单位，结果在本轮任务末尾保持一致。 */
  sensor_working_snapshot.battery_decivolt = Sensor_CalcScaledDecivolt(sensor_working_snapshot.millivolt[SENSOR_ADC_BATTERY_VOLT], SENSOR_BAT_DIVIDER_X100);
  sensor_working_snapshot.dcin_decivolt = Sensor_CalcScaledDecivolt(sensor_working_snapshot.millivolt[SENSOR_ADC_DCIN_VOLT], SENSOR_DCIN_DIVIDER_X100);
  sensor_working_snapshot.pump_current_ma = Sensor_CalcPumpCurrentMa(sensor_working_snapshot.millivolt[SENSOR_ADC_PUMP_CURRENT]);
  sensor_working_snapshot.motor_current_ma = Sensor_CalcMotorCurrentMa(sensor_working_snapshot.millivolt[SENSOR_ADC_MOTOR_CURRENT]);

  /* 桶内 NTC 温度换算为摄氏度并滤波，更新 temperature_c 和 temp_sensor_ok。 */
  temp_c = Sensor_CalcNtcTemperature(sensor_working_snapshot.raw[SENSOR_ADC_NTC_TEMP]);
  Sensor_ProcessTemperature(temp_c);

  /* 电池 NTC 温度换算为摄氏度并滤波，更新 battery_temperature_c 和 battery_temp_sensor_ok。 */
  temp_c = Sensor_CalcNtcTemperature(sensor_working_snapshot.raw[SENSOR_ADC_BAT_NTC_TEMP]);
  Sensor_ProcessBatteryTemperature(temp_c);

  /* 全部字段计算完成后再一次性发布，读者不会看到半更新快照。 */
  Sensor_PublishSnapshot();
}

void Sensor_GetSnapshot(SensorSnapshot_t *snapshot)
{
  if (snapshot != NULL)
  {
    /* 暂停任务调度，避免发布任务在结构体复制中途切换进来。 */
    vTaskSuspendAll();
    *snapshot = sensor_snapshot;
    (void)xTaskResumeAll();
  }
}

uint16_t Sensor_GetRaw(SensorAdcChannel_t channel)
{
  if ((uint8_t)channel >= SENSOR_ADC_CHANNEL_COUNT)
  {
    return 0U;
  }
  return sensor_snapshot.raw[channel];
}

uint16_t Sensor_GetMilliVolt(SensorAdcChannel_t channel)
{
  if ((uint8_t)channel >= SENSOR_ADC_CHANNEL_COUNT)
  {
    return 0U;
  }
  return sensor_snapshot.millivolt[channel];
}

float Sensor_GetWaterLiters(void)
{
  return sensor_snapshot.water_liters;
}

/*
 * 功能：将浮点水量转换为通信协议使用的整数升。
 * 公式：protocol_liters = round(water_liters)，并限制在 0~MAX_LITERS。
 */
uint8_t Sensor_GetWaterLevelProtocol(void)
{
  uint8_t liters;

  /* 协议只传整数升，因此四舍五入并限制到允许的最大水量。 */
  liters = (uint8_t)(sensor_snapshot.water_liters + 0.5f);
  if (liters > SENSOR_WATER_MAX_LITERS)
  {
    liters = SENSOR_WATER_MAX_LITERS;
  }
  return liters;
}

uint32_t Sensor_GetWaterFrequencyHz(void)
{
  return sensor_snapshot.water_frequency_hz;
}

float Sensor_GetTemperatureC(void)
{
  return sensor_snapshot.temperature_c;
}

/*
 * 功能：将桶内温度转换为通信/显示使用的 0.1℃ 整数。
 * 公式：temperature_x10 = temperature_c * 10；传感器无效时返回 -1000。
 */
int16_t Sensor_GetTemperatureCx10(void)
{
  uint8_t sensor_ok;
  float temperature_c;

  /* 有效标志和温度必须来自同一次已发布快照。 */
  vTaskSuspendAll();
  sensor_ok = sensor_snapshot.temp_sensor_ok;
  temperature_c = sensor_snapshot.temperature_c;
  (void)xTaskResumeAll();

  if (sensor_ok == 0U)
  {
    return -1000;
  }
  return (int16_t)(temperature_c * 10.0f);
}

float Sensor_GetBatteryTemperatureC(void)
{
  return sensor_snapshot.battery_temperature_c;
}

/*
 * 功能：将电池温度转换为通信/显示使用的 0.1℃ 整数。
 * 公式：battery_temperature_x10 = battery_temperature_c * 10；无效时返回 -1000。
 */
int16_t Sensor_GetBatteryTemperatureCx10(void)
{
  uint8_t sensor_ok;
  float temperature_c;

  /* 有效标志和温度必须来自同一次已发布快照。 */
  vTaskSuspendAll();
  sensor_ok = sensor_snapshot.battery_temp_sensor_ok;
  temperature_c = sensor_snapshot.battery_temperature_c;
  (void)xTaskResumeAll();

  if (sensor_ok == 0U)
  {
    return -1000;
  }
  return (int16_t)(temperature_c * 10.0f);
}

uint16_t Sensor_GetBatteryDeciVolt(void)
{
  return sensor_snapshot.battery_decivolt;
}

uint16_t Sensor_GetDcinDeciVolt(void)
{
  return sensor_snapshot.dcin_decivolt;
}

uint16_t Sensor_GetPumpCurrentMa(void)
{
  return sensor_snapshot.pump_current_ma;
}

uint16_t Sensor_GetMotorCurrentMa(void)
{
  return sensor_snapshot.motor_current_ma;
}

uint8_t Sensor_IsWaterSensorOk(void)
{
  return sensor_snapshot.water_sensor_ok;
}

uint8_t Sensor_IsTempSensorOk(void)
{
  return sensor_snapshot.temp_sensor_ok;
}

uint8_t Sensor_IsBatteryTempSensorOk(void)
{
  return sensor_snapshot.battery_temp_sensor_ok;
}
