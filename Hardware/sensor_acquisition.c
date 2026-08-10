#include "sensor_acquisition.h"
#include "adc.h"
#include "tim.h"
#include "main.h"
#include <string.h>

/* 一阶 IIR：accumulator = accumulator - accumulator / 8 + sample。 */
#define SENSOR_ADC_FILTER_DIV 8U

/* DMA 按 ADC Rank 顺序循环写入：DCIN、电池、桶温、水泵电流、电机电流、电池温度。 */
static volatile uint16_t sensor_adc_dma[SENSOR_ACQUISITION_CHANNEL_COUNT];
static uint32_t sensor_filter_acc[SENSOR_ACQUISITION_CHANNEL_COUNT];
static uint16_t sensor_filtered_raw[SENSOR_ACQUISITION_CHANNEL_COUNT];
static uint8_t sensor_filter_ready;

/* TIM3 输入捕获中断更新水位脉冲计数和最后脉冲时间。 */
static volatile uint32_t sensor_water_pulse_count;
static volatile uint32_t sensor_water_last_pulse_tick;

static void SensorAcquisition_RestoreIrq(uint32_t primask)
{
  /* 仅恢复本函数进入前原本开启的中断，避免破坏调用者的临界区。 */
  if (primask == 0U)
  {
    __enable_irq();
  }
}

void SensorAcquisition_Init(void)
{
  memset((void *)sensor_adc_dma, 0, sizeof(sensor_adc_dma));
  memset(sensor_filter_acc, 0, sizeof(sensor_filter_acc));
  memset(sensor_filtered_raw, 0, sizeof(sensor_filtered_raw));

  sensor_filter_ready = 0U;
  sensor_water_pulse_count = 0UL;
  sensor_water_last_pulse_tick = 0UL;

  /* 先给两个 NTC 分压网络上电，再校准 ADC 并启动 6 路循环 DMA。 */
  HAL_GPIO_WritePin(EN_NTC_GPIO_Port, EN_NTC_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(EN_BAT_NTC_GPIO_Port, EN_BAT_NTC_Pin, GPIO_PIN_SET);

  HAL_ADCEx_Calibration_Start(&hadc1);

  if (HAL_ADC_Start_DMA(&hadc1,
                        (uint32_t *)sensor_adc_dma,
                        SENSOR_ACQUISITION_CHANNEL_COUNT) != HAL_OK)
  {
    Error_Handler();
  }

  if (HAL_TIM_IC_Start_IT(&htim3, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
}

/*
 * 功能：对 DMA 中的 6 路 ADC 原始样本分别执行一阶 IIR 滤波。
 * 公式：filtered += (raw - filtered) / SENSOR_ADC_FILTER_DIV。
 * 当前分母为 8；首次处理直接以本次原始值初始化，避免从 0 缓慢爬升。
 */
void SensorAcquisition_TaskProcess(void)
{
  uint8_t index;
  uint16_t raw;

  for (index = 0U; index < SENSOR_ACQUISITION_CHANNEL_COUNT; index++)
  {
    raw = sensor_adc_dma[index];
    if (sensor_filter_ready == 0U)
    {
      sensor_filter_acc[index] = (uint32_t)raw * SENSOR_ADC_FILTER_DIV;
    }
    else
    {
      sensor_filter_acc[index] = sensor_filter_acc[index] -
                                 (sensor_filter_acc[index] / SENSOR_ADC_FILTER_DIV) +
                                 raw;
    }

    sensor_filtered_raw[index] =
        (uint16_t)(sensor_filter_acc[index] / SENSOR_ADC_FILTER_DIV);
  }

  sensor_filter_ready = 1U;
}

void SensorAcquisition_GetFilteredRawSnapshot(
    uint16_t raw[SENSOR_ACQUISITION_CHANNEL_COUNT])
{
  if (raw != NULL)
  {
    memcpy(raw, sensor_filtered_raw, sizeof(sensor_filtered_raw));
  }
}

uint32_t SensorAcquisition_TakeWaterPulseCount(void)
{
  uint32_t primask;
  uint32_t pulse_count;

  /* “读取并清零”必须原子完成，否则此间到达的捕获脉冲可能丢失。 */
  primask = __get_PRIMASK();
  __disable_irq();
  pulse_count = sensor_water_pulse_count;
  sensor_water_pulse_count = 0UL;
  SensorAcquisition_RestoreIrq(primask);

  return pulse_count;
}

uint32_t SensorAcquisition_GetWaterLastPulseTick(void)
{
  return sensor_water_last_pulse_tick;
}

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
  /* HAL 共用回调：确认 TIM3 的通道 1 后再累计水位脉冲。 */
  if ((htim == &htim3) && (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1))
  {
    sensor_water_pulse_count++;
    sensor_water_last_pulse_tick = HAL_GetTick();
  }
}
