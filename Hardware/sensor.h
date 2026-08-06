#ifndef SENSOR_H
#define SENSOR_H

#include "stm32f1xx_hal.h"
#include "cmsis_os.h"

#ifdef __cplusplus
extern "C" {
#endif

/*
 * 传感器模块说明：ADC1 使用扫描 DMA 连续采集 6 路模拟量；TIM3 输入
 * 捕获中断统计水位传感器脉冲。业务模块应使用本文件的 Get 接口，
 * 不要直接访问 DMA 缓冲区。
 */
/* ADC 扫描通道总数，必须与 DMA 缓冲区长度保持一致。 */
#define SENSOR_ADC_CHANNEL_COUNT        6U
#define SENSOR_WATER_MAX_LITERS         20U
#define SENSOR_WATER_MIN_SAFE_LITERS    1U
#define SENSOR_TEMP_INVALID_C           (-100.0f)

#ifndef SENSOR_ADC_VREF_MV
#define SENSOR_ADC_VREF_MV              3300U
#endif

#ifndef SENSOR_WATER_EMPTY_COUNT
/* 水位标定值：频率越低表示水量越多，两端之间采用线性插值。 */
#define SENSOR_WATER_EMPTY_COUNT        26700U
#endif

#ifndef SENSOR_WATER_FULL_COUNT
#define SENSOR_WATER_FULL_COUNT         24700U
#endif

#ifndef SENSOR_WATER_COUNT_WINDOW_MS
/* 脉冲统计窗口、合法频率范围以及无脉冲超时时间。 */
#define SENSOR_WATER_COUNT_WINDOW_MS    1000U
#endif

#ifndef SENSOR_WATER_COUNT_MIN_VALID
#define SENSOR_WATER_COUNT_MIN_VALID    1000U
#endif

#ifndef SENSOR_WATER_COUNT_MAX_VALID
#define SENSOR_WATER_COUNT_MAX_VALID    40000U
#endif

#ifndef SENSOR_WATER_COUNT_TIMEOUT_MS
#define SENSOR_WATER_COUNT_TIMEOUT_MS   2000U
#endif

#ifndef SENSOR_BAT_DIVIDER_X100
/* 电阻分压倍率乘以 100；1100 表示输入约为 ADC 引脚电压的 11 倍。 */
#define SENSOR_BAT_DIVIDER_X100         1100U
#endif

#ifndef SENSOR_DCIN_DIVIDER_X100
#define SENSOR_DCIN_DIVIDER_X100        1100U
#endif

#ifndef SENSOR_PUMP_CURRENT_SENSE_MOHM
/* 水泵低边采样电阻 R61=0.05Ω，单位 mΩ。 */
#define SENSOR_PUMP_CURRENT_SENSE_MOHM  50U
#endif

#ifndef SENSOR_PUMP_CURRENT_SCALE_DIV
/* 当前硬件采样链路计算值需缩小 100 倍才是实际水泵电流。 */
#define SENSOR_PUMP_CURRENT_SCALE_DIV   100U
#endif

#ifndef SENSOR_MOTOR_CURRENT_SENSE_MOHM
#define SENSOR_MOTOR_CURRENT_SENSE_MOHM 150U
#endif

typedef enum
{
  /* 枚举顺序必须与 CubeMX 配置的 ADC regular rank 顺序一致。 */
  SENSOR_ADC_DCIN_VOLT = 0,
  SENSOR_ADC_BATTERY_VOLT,
  SENSOR_ADC_NTC_TEMP,
  SENSOR_ADC_PUMP_CURRENT,
  SENSOR_ADC_MOTOR_CURRENT,
  SENSOR_ADC_BAT_NTC_TEMP
} SensorAdcChannel_t;

typedef struct
{
  /* 同一处理周期生成的一致性快照，避免上层重复换算采样值。 */
  uint16_t raw[SENSOR_ADC_CHANNEL_COUNT];
  uint16_t millivolt[SENSOR_ADC_CHANNEL_COUNT];
  float water_liters;
  uint32_t water_frequency_hz;
  float temperature_c;
  float battery_temperature_c;
  uint16_t battery_decivolt;
  uint16_t dcin_decivolt;
  uint16_t pump_current_ma;
  uint16_t motor_current_ma;
  uint8_t water_sensor_ok;
  uint8_t temp_sensor_ok;
  uint8_t battery_temp_sensor_ok;
} SensorSnapshot_t;

/** 初始化 ADC DMA、NTC 供电和水位脉冲输入捕获。 */
void Sensor_Init(void);
/** 周期处理入口：建议由传感器任务固定周期调用。 */
void Sensor_TaskProcess(void);
/** 复制完整快照；snapshot 为 NULL 时不执行任何操作。 */
void Sensor_GetSnapshot(SensorSnapshot_t *snapshot);

uint16_t Sensor_GetRaw(SensorAdcChannel_t channel);
uint16_t Sensor_GetMilliVolt(SensorAdcChannel_t channel);
float Sensor_GetWaterLiters(void);
uint8_t Sensor_GetWaterLevelProtocol(void);
uint32_t Sensor_GetWaterFrequencyHz(void);
float Sensor_GetTemperatureC(void);
/** 返回 0.1℃；温度传感器无效时返回 -1000。 */
int16_t Sensor_GetTemperatureCx10(void);
float Sensor_GetBatteryTemperatureC(void);
int16_t Sensor_GetBatteryTemperatureCx10(void);
uint16_t Sensor_GetBatteryDeciVolt(void);
uint16_t Sensor_GetDcinDeciVolt(void);
uint16_t Sensor_GetPumpCurrentMa(void);
uint16_t Sensor_GetMotorCurrentMa(void);
uint8_t Sensor_IsWaterSensorOk(void);
uint8_t Sensor_IsTempSensorOk(void);
uint8_t Sensor_IsBatteryTempSensorOk(void);

#ifdef __cplusplus
}
#endif

#endif
