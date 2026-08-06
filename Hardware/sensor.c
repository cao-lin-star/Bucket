#include "sensor.h"
#include "adc.h"
#include "tim.h"
#include "main.h"
#include <math.h>
#include <string.h>

/* 一阶 IIR 滤波分母：输出 += (输入 - 输出) / DIV。 */
#define SENSOR_FILTER_DIV          8U
#define SENSOR_TEMP_FILTER_DIV     16U

/* 5 kΩ、B=3950 NTC 的 Beta 模型参数，参考温度为 25℃。 */
#define SENSOR_NTC_R0_OHMS         10000.0f
#define SENSOR_NTC_BETA            3950.0f
#define SENSOR_NTC_T0_K            298.15f
#define SENSOR_NTC_PULLUP_OHMS     100000.0f

/* DMA 和捕获中断会异步更新 volatile 数据；其余状态由传感器任务访问。 */
static volatile uint16_t sensor_adc_dma[SENSOR_ADC_CHANNEL_COUNT];
static uint32_t sensor_filter_acc[SENSOR_ADC_CHANNEL_COUNT];

static volatile uint32_t sensor_water_pulse_count;
static volatile uint32_t sensor_water_last_pulse_tick;
static uint32_t sensor_water_count_filter_acc;
static uint32_t sensor_water_window_start_tick;
static uint8_t sensor_water_window_started;
static uint8_t sensor_water_filter_ready;

static SensorSnapshot_t sensor_snapshot;
static uint8_t sensor_filter_ready;
static float sensor_temp_filter_c;
static uint8_t sensor_temp_filter_ready;
static float sensor_battery_temp_filter_c;
static uint8_t sensor_battery_temp_filter_ready;

static void Sensor_RestoreIrq(uint32_t primask)
{
  /* 仅当调用前中断开启时恢复，避免破坏上层临界区。 */
  if (primask == 0U)
  {
    __enable_irq();
  }
}

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

static uint16_t Sensor_CalcScaledDecivolt(uint16_t adc_mv, uint16_t divider_x100)
{
  uint32_t input_mv;

  input_mv = ((uint32_t)adc_mv * divider_x100) / 100U;
  return (uint16_t)((input_mv + 50U) / 100U);
}

static uint16_t Sensor_CalcPumpCurrentMa(uint16_t adc_mv)
{
  uint32_t current_ma;

  if ((SENSOR_PUMP_CURRENT_SENSE_MOHM == 0U) ||
      (SENSOR_PUMP_CURRENT_SCALE_DIV == 0U))
  {
    return 0U;
  }

  /* R61=0.05Ω 低边采样；按采样电阻算出的值再缩小 100 倍为实际电流。 */
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

static uint32_t Sensor_TakeWaterPulseSnapshot(void)
{
  uint32_t primask;
  uint32_t pulse_count;

  /* “读取并清零”必须原子完成，否则此间到来的捕获脉冲可能丢失。 */
  primask = __get_PRIMASK();
  __disable_irq();
  pulse_count = sensor_water_pulse_count;
  sensor_water_pulse_count = 0UL;
  Sensor_RestoreIrq(primask);

  return pulse_count;
}

static void Sensor_ProcessWaterCount(uint32_t now_tick)
{
  uint32_t elapsed_ms;
  uint32_t pulse_count;
  uint32_t count_per_second;

  /* 第一次调用只建立时间基准，丢弃初始化前可能积累的脉冲。 */
  if (sensor_water_window_started == 0U)
  {
    (void)Sensor_TakeWaterPulseSnapshot();
    sensor_water_window_start_tick = now_tick;
    sensor_water_window_started = 1U;
    return;
  }

  elapsed_ms = now_tick - sensor_water_window_start_tick;
  /* 窗口未结束时仍检查掉线，确保拔掉传感器后尽快报无效。 */
  if (elapsed_ms < SENSOR_WATER_COUNT_WINDOW_MS)
  {
    if ((sensor_water_last_pulse_tick == 0UL) ||
        ((now_tick - sensor_water_last_pulse_tick) > SENSOR_WATER_COUNT_TIMEOUT_MS))
    {
      sensor_water_filter_ready = 0U;
      sensor_water_count_filter_acc = 0UL;
      sensor_snapshot.water_frequency_hz = 0UL;
      sensor_snapshot.water_liters = 0.0f;
      sensor_snapshot.water_sensor_ok = 0U;
    }
    return;
  }

  pulse_count = Sensor_TakeWaterPulseSnapshot();
  sensor_water_window_start_tick = now_tick;

  /* 按实际窗口长度归一化为 Hz，允许任务调度带来少量周期抖动。 */
  count_per_second = (pulse_count * 1000UL + (elapsed_ms / 2UL)) / elapsed_ms;
  if ((sensor_water_last_pulse_tick == 0UL) ||
      ((now_tick - sensor_water_last_pulse_tick) > SENSOR_WATER_COUNT_TIMEOUT_MS) ||
      (count_per_second < SENSOR_WATER_COUNT_MIN_VALID) ||
      (count_per_second > SENSOR_WATER_COUNT_MAX_VALID))
  {
    sensor_water_filter_ready = 0U;
    sensor_water_count_filter_acc = 0UL;
    sensor_snapshot.water_frequency_hz = 0UL;
    sensor_snapshot.water_liters = 0.0f;
    sensor_snapshot.water_sensor_ok = 0U;
    return;
  }

  /* 首个有效值直接装载，后续以 1/8 权重抑制水面晃动。 */
  if (sensor_water_filter_ready == 0U)
  {
    sensor_water_count_filter_acc = count_per_second * SENSOR_FILTER_DIV;
    sensor_water_filter_ready = 1U;
  }
  else
  {
    sensor_water_count_filter_acc = sensor_water_count_filter_acc -
                                    (sensor_water_count_filter_acc / SENSOR_FILTER_DIV) +
                                    count_per_second;
  }

  sensor_snapshot.water_frequency_hz = sensor_water_count_filter_acc / SENSOR_FILTER_DIV;
  sensor_snapshot.water_liters = Sensor_CalcWaterLiters(sensor_snapshot.water_frequency_hz);
  sensor_snapshot.water_sensor_ok = 1U;
}

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

    sensor_snapshot.temperature_c = sensor_temp_filter_c;
    sensor_snapshot.temp_sensor_ok = 1U;
  }
  else
  {
    sensor_temp_filter_ready = 0U;
    sensor_temp_filter_c = SENSOR_TEMP_INVALID_C;
    sensor_snapshot.temperature_c = SENSOR_TEMP_INVALID_C;
    sensor_snapshot.temp_sensor_ok = 0U;
  }
}

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

    sensor_snapshot.battery_temperature_c = sensor_battery_temp_filter_c;
    sensor_snapshot.battery_temp_sensor_ok = 1U;
  }
  else
  {
    sensor_battery_temp_filter_ready = 0U;
    sensor_battery_temp_filter_c = SENSOR_TEMP_INVALID_C;
    sensor_snapshot.battery_temperature_c = SENSOR_TEMP_INVALID_C;
    sensor_snapshot.battery_temp_sensor_ok = 0U;
  }
}

void Sensor_Init(void)
{
  memset((void *)sensor_adc_dma, 0, sizeof(sensor_adc_dma));
  memset(sensor_filter_acc, 0, sizeof(sensor_filter_acc));
  memset(&sensor_snapshot, 0, sizeof(sensor_snapshot));

  sensor_snapshot.temperature_c = SENSOR_TEMP_INVALID_C;
  sensor_snapshot.battery_temperature_c = SENSOR_TEMP_INVALID_C;

  sensor_filter_ready = 0U;
  sensor_temp_filter_c = SENSOR_TEMP_INVALID_C;
  sensor_temp_filter_ready = 0U;
  sensor_battery_temp_filter_c = SENSOR_TEMP_INVALID_C;
  sensor_battery_temp_filter_ready = 0U;

  sensor_water_pulse_count = 0UL;
  sensor_water_last_pulse_tick = 0UL;
  sensor_water_count_filter_acc = 0UL;
  sensor_water_window_start_tick = 0UL;
  sensor_water_window_started = 0U;
  sensor_water_filter_ready = 0U;

  /* 先给 NTC 分压网络上电，再校准 ADC 并启动循环 DMA。 */
  HAL_GPIO_WritePin(EN_NTC_GPIO_Port, EN_NTC_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(EN_BAT_NTC_GPIO_Port, EN_BAT_NTC_Pin, GPIO_PIN_SET);

  HAL_ADCEx_Calibration_Start(&hadc1);

  if (HAL_ADC_Start_DMA(&hadc1, (uint32_t *)sensor_adc_dma, SENSOR_ADC_CHANNEL_COUNT) != HAL_OK)
  {
    Error_Handler();
  }

  if (HAL_TIM_IC_Start_IT(&htim3, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
}

void Sensor_TaskProcess(void)
{
  uint8_t index;
  uint16_t raw;
  uint16_t mv;
  float temp_c;
  uint32_t now_tick;

  /* 对每个 DMA 通道独立滤波，并保存 ADC 码与毫伏值供诊断。 */
  for (index = 0U; index < SENSOR_ADC_CHANNEL_COUNT; index++)
  {
    raw = sensor_adc_dma[index];
    if (sensor_filter_ready == 0U)
    {
      sensor_filter_acc[index] = (uint32_t)raw * SENSOR_FILTER_DIV;
    }
    else
    {
      sensor_filter_acc[index] = sensor_filter_acc[index] -
                                 (sensor_filter_acc[index] / SENSOR_FILTER_DIV) +
                                 raw;
    }

    sensor_snapshot.raw[index] = (uint16_t)(sensor_filter_acc[index] / SENSOR_FILTER_DIV);
    mv = Sensor_RawToMv(sensor_snapshot.raw[index]);
    sensor_snapshot.millivolt[index] = mv;
  }

  sensor_filter_ready = 1U;

  now_tick = HAL_GetTick();
  Sensor_ProcessWaterCount(now_tick);

  /* 将通用采样值转换为业务单位，结果在本轮任务末尾保持一致。 */
  sensor_snapshot.battery_decivolt = Sensor_CalcScaledDecivolt(sensor_snapshot.millivolt[SENSOR_ADC_BATTERY_VOLT], SENSOR_BAT_DIVIDER_X100);
  sensor_snapshot.dcin_decivolt = Sensor_CalcScaledDecivolt(sensor_snapshot.millivolt[SENSOR_ADC_DCIN_VOLT], SENSOR_DCIN_DIVIDER_X100);
  sensor_snapshot.pump_current_ma = Sensor_CalcPumpCurrentMa(sensor_snapshot.millivolt[SENSOR_ADC_PUMP_CURRENT]);
  sensor_snapshot.motor_current_ma = Sensor_CalcMotorCurrentMa(sensor_snapshot.millivolt[SENSOR_ADC_MOTOR_CURRENT]);

  temp_c = Sensor_CalcNtcTemperature(sensor_snapshot.raw[SENSOR_ADC_NTC_TEMP]);
  Sensor_ProcessTemperature(temp_c);

  temp_c = Sensor_CalcNtcTemperature(sensor_snapshot.raw[SENSOR_ADC_BAT_NTC_TEMP]);
  Sensor_ProcessBatteryTemperature(temp_c);
}

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
  /* HAL 共用回调：确认实例和活动通道后再累计水位脉冲。 */
  if ((htim == &htim3) && (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1))
  {
    sensor_water_pulse_count++;
    sensor_water_last_pulse_tick = HAL_GetTick();
  }
}

void Sensor_GetSnapshot(SensorSnapshot_t *snapshot)
{
  if (snapshot != NULL)
  {
    *snapshot = sensor_snapshot;
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

int16_t Sensor_GetTemperatureCx10(void)
{
  if (sensor_snapshot.temp_sensor_ok == 0U)
  {
    return -1000;
  }
  return (int16_t)(sensor_snapshot.temperature_c * 10.0f);
}

float Sensor_GetBatteryTemperatureC(void)
{
  return sensor_snapshot.battery_temperature_c;
}

int16_t Sensor_GetBatteryTemperatureCx10(void)
{
  if (sensor_snapshot.battery_temp_sensor_ok == 0U)
  {
    return -1000;
  }
  return (int16_t)(sensor_snapshot.battery_temperature_c * 10.0f);
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
