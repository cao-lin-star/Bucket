#ifndef SENSOR_H
#define SENSOR_H

#include "stm32f1xx_hal.h"
#include "cmsis_os.h"
#include "sensor_acquisition.h"

#ifdef __cplusplus
extern "C" {
#endif

/*
 * 传感器应用层说明：读取硬件采集层提供的 6 路滤波后 ADC 原始值，
 * 统一换算为水量、温度、电压和电流。业务模块应使用本文件的 Get
 * 接口，不要直接访问 ADC、DMA 或输入捕获外设。
 */
/* 对外保留原有名称，通道总数由硬件采集层唯一维护。 */
#define SENSOR_ADC_CHANNEL_COUNT        SENSOR_ACQUISITION_CHANNEL_COUNT
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
/* 满桶时的水位标定值。 */
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
/* 水泵低边采样电阻 R61=0.03Ω，单位 mΩ。 */
#define SENSOR_PUMP_CURRENT_SENSE_MOHM  30U
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
  /* 公开枚举复用采集层 Rank，防止 DMA 下标和业务含义发生偏移。 */
  SENSOR_ADC_DCIN_VOLT = SENSOR_ACQUISITION_ADC_DCIN_VOLT,
  SENSOR_ADC_BATTERY_VOLT = SENSOR_ACQUISITION_ADC_BATTERY_VOLT,
  SENSOR_ADC_NTC_TEMP = SENSOR_ACQUISITION_ADC_NTC_TEMP,
  SENSOR_ADC_PUMP_CURRENT = SENSOR_ACQUISITION_ADC_PUMP_CURRENT,
  SENSOR_ADC_MOTOR_CURRENT = SENSOR_ACQUISITION_ADC_MOTOR_CURRENT,
  SENSOR_ADC_BAT_NTC_TEMP = SENSOR_ACQUISITION_ADC_BAT_NTC_TEMP
} SensorAdcChannel_t;

typedef struct
{
  /* 同一处理周期生成的一致性快照，避免上层重复换算采样值。 */
  uint16_t raw[SENSOR_ADC_CHANNEL_COUNT];        /* 6 路 ADC 滤波后的原始值，范围 0~4095。 */
  uint16_t millivolt[SENSOR_ADC_CHANNEL_COUNT]; /* 6 路 ADC 原始值换算后的引脚电压，单位 mV。 */
  float water_liters;                           /* 根据水位脉冲频率换算的水量，单位 L。 */
  uint32_t water_frequency_hz;                  /* 水位传感器滤波后的脉冲频率，单位 Hz。 */
  float temperature_c;                         /* 桶内 NTC 温度，单位 ℃；无效时为 -100.0。 */
  float battery_temperature_c;                 /* 电池 NTC 温度，单位 ℃；无效时为 -100.0。 */
  uint16_t battery_decivolt;                    /* 电池实际电压，单位 0.1 V；245 表示 24.5 V。 */
  uint16_t dcin_decivolt;                       /* 基站/DCIN 输入电压，单位 0.1 V。 */
  uint16_t pump_current_ma;                     /* 水泵工作电流，单位 mA。 */
  uint16_t motor_current_ma;                    /* 按摩电机工作电流，单位 mA。 */
  uint8_t water_sensor_ok;                      /* 水位传感器状态：1 正常，0 无脉冲或频率异常。 */
  uint8_t temp_sensor_ok;                       /* 桶内温度传感器状态：1 正常，0 开路/短路或越界。 */
  uint8_t battery_temp_sensor_ok;               /* 电池温度传感器状态：1 正常，0 开路/短路或越界。 */
} SensorSnapshot_t;

/** 初始化传感器应用状态，并调用硬件采集层完成 ADC/DMA 等初始化。 */
void Sensor_Init(void);
/** 周期处理入口：由调度器启动后的传感器任务固定周期调用。 */
void Sensor_TaskProcess(void);
/** 仅限任务上下文：暂停任务调度并复制最近一次完整发布的快照；外设中断保持响应。 */
void Sensor_GetSnapshot(SensorSnapshot_t *snapshot);

uint16_t Sensor_GetRaw(SensorAdcChannel_t channel);
uint16_t Sensor_GetMilliVolt(SensorAdcChannel_t channel);
float Sensor_GetWaterLiters(void);
uint8_t Sensor_GetWaterLevelProtocol(void);
uint32_t Sensor_GetWaterFrequencyHz(void);
float Sensor_GetTemperatureC(void);
/** 仅限任务上下文：返回 0.1℃；温度传感器无效时返回 -1000。 */
int16_t Sensor_GetTemperatureCx10(void);
float Sensor_GetBatteryTemperatureC(void);
/** 仅限任务上下文：返回电池温度 0.1℃；传感器无效时返回 -1000。 */
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
