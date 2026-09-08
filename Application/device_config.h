#ifndef DEVICE_CONFIG_H
#define DEVICE_CONFIG_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/*
 * 桶体设备标定参数。
 * 水位传感器频率越低表示水量越多，因此空桶频率必须大于满桶频率。
 */
typedef struct
{
  uint32_t water_empty_frequency_hz; /* 空桶标定频率，单位 Hz。 */
  uint32_t water_full_frequency_hz;  /* 满桶标定频率，单位 Hz。 */
  uint16_t water_max_liters;         /* 满桶标定水量，单位 L。 */
  uint16_t reserved;                 /* 保留字段，当前版本不参与业务判断。 */
} DeviceConfig_t;

typedef enum
{
  DEVICE_CONFIG_STATUS_OK = 0,
  DEVICE_CONFIG_STATUS_INVALID_ARGUMENT,
  DEVICE_CONFIG_STATUS_INVALID_VALUE,
  DEVICE_CONFIG_STATUS_FLASH_UNAVAILABLE,
  DEVICE_CONFIG_STATUS_FLASH_ERASE_FAILED,
  DEVICE_CONFIG_STATUS_FLASH_PROGRAM_FAILED,
  DEVICE_CONFIG_STATUS_FLASH_VERIFY_FAILED,
  DEVICE_CONFIG_STATUS_DEFAULTS_USED,
  DEVICE_CONFIG_STATUS_BUSY
} DeviceConfigStatus_t;

/* 在 FreeRTOS 调度器启动前调用，直接从 Flash 装载，失败时使用编译默认值。 */
void DeviceConfig_Init(void);

/* 仅限任务上下文：使用短调度器锁复制当前完整配置快照。 */
void DeviceConfig_GetSnapshot(DeviceConfig_t *config);

/* 校验参数范围，不访问 Flash，也不修改当前配置。 */
DeviceConfigStatus_t DeviceConfig_Validate(const DeviceConfig_t *config);

/* 将合法配置应用到 RAM；成功后标记为待保存。 */
DeviceConfigStatus_t DeviceConfig_Apply(const DeviceConfig_t *config);

/* 将当前 RAM 配置追加保存到双页 Flash 日志。 */
DeviceConfigStatus_t DeviceConfig_Save(void);

/* 恢复编译默认值并标记为待保存。 */
void DeviceConfig_LoadDefaults(void);

/* 重新扫描 Flash；没有有效记录时装载默认值并返回 DEFAULTS_USED。 */
DeviceConfigStatus_t DeviceConfig_Reload(void);

uint8_t DeviceConfig_IsLoadedFromFlash(void);
uint8_t DeviceConfig_IsDirty(void);
uint32_t DeviceConfig_GetSequence(void);

#ifdef __cplusplus
}
#endif

#endif /* DEVICE_CONFIG_H */
