#ifndef SENSOR_ACQUISITION_H
#define SENSOR_ACQUISITION_H

#include "stm32f1xx_hal.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ADC1 DMA 下标与 Core/Src/adc.c 的 regular Rank 一一对应。 */
typedef enum
{
  SENSOR_ACQUISITION_ADC_DCIN_VOLT = 0, /* DMA 下标 0，Rank 1，PA4/ADC1_IN4：DCIN 输入电压。 */
  SENSOR_ACQUISITION_ADC_BATTERY_VOLT,  /* DMA 下标 1，Rank 2，PA5/ADC1_IN5：电池电压。 */
  SENSOR_ACQUISITION_ADC_NTC_TEMP,      /* DMA 下标 2，Rank 3，PA7/ADC1_IN7：桶内 NTC 温度。 */
  SENSOR_ACQUISITION_ADC_PUMP_CURRENT,  /* DMA 下标 3，Rank 4，PB0/ADC1_IN8：水泵电流。 */
  SENSOR_ACQUISITION_ADC_MOTOR_CURRENT, /* DMA 下标 4，Rank 5，PB1/ADC1_IN9：按摩电机电流。 */
  SENSOR_ACQUISITION_ADC_BAT_NTC_TEMP,  /* DMA 下标 5，Rank 6，PA0/ADC1_IN0：电池 NTC 温度。 */
  SENSOR_ACQUISITION_CHANNEL_COUNT      /* ADC 采集通道总数，必须始终放在枚举末尾。 */
} SensorAcquisitionAdcChannel_t;

/** 初始化 NTC 供电、ADC 校准、循环 DMA 和水位脉冲输入捕获。 */
void SensorAcquisition_Init(void);

/** 读取 DMA 当前样本并对 6 个 ADC 通道分别执行一阶 IIR 滤波。 */
void SensorAcquisition_TaskProcess(void);

/** 复制同一处理周期得到的 6 路滤波后 ADC 原始值，范围为 0~4095。 */
void SensorAcquisition_GetFilteredRawSnapshot(
    uint16_t raw[SENSOR_ACQUISITION_CHANNEL_COUNT]);

/** 原子读取并清零当前水位脉冲累计值。 */
uint32_t SensorAcquisition_TakeWaterPulseCount(void);

/** 返回最近一次水位脉冲到达时的 HAL Tick，单位 ms。 */
uint32_t SensorAcquisition_GetWaterLastPulseTick(void);

#ifdef __cplusplus
}
#endif

#endif /* SENSOR_ACQUISITION_H */
