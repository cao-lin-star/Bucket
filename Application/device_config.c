#include "device_config.h"

#include "sensor.h"
#include "stm32f1xx_hal.h"
#include "stm32f1xx_hal_flash_ex.h"
#include "FreeRTOS.h"
#include "task.h"
#include <stddef.h>
#include <string.h>

/*
 * STM32F103C8 官方容量为 64 KiB，末尾两个 1 KiB 页专用于配置日志。
 * 链接配置必须同步把应用 IROM1 大小限制为 0xF800，避免代码覆盖参数页。
 */
#define DEVICE_CONFIG_FLASH_PAGE_A_ADDRESS   0x0800F800UL
#define DEVICE_CONFIG_FLASH_PAGE_B_ADDRESS   0x0800FC00UL
#define DEVICE_CONFIG_FLASH_PAGE_SIZE        0x00000400UL
#define DEVICE_CONFIG_FLASH_MIN_KB           64U
#define DEVICE_CONFIG_FLASH_ALT_KB           128U

#define DEVICE_CONFIG_RECORD_SIZE            32UL
#define DEVICE_CONFIG_RECORDS_PER_PAGE         (DEVICE_CONFIG_FLASH_PAGE_SIZE / DEVICE_CONFIG_RECORD_SIZE)

#define DEVICE_CONFIG_RECORD_MAGIC           0x47464344UL /* 内存字节序为 "DCFG"。 */
#define DEVICE_CONFIG_RECORD_VERSION         1U
#define DEVICE_CONFIG_RECORD_COMMIT          0xA55AU

#define DEVICE_CONFIG_FREQUENCY_MIN_HZ       1000UL
#define DEVICE_CONFIG_FREQUENCY_MAX_HZ       40000UL
#define DEVICE_CONFIG_FREQUENCY_MIN_SPAN_HZ  100UL
#define DEVICE_CONFIG_WATER_MIN_LITERS       1U
#define DEVICE_CONFIG_WATER_MAX_LITERS       20U

#if (FLASH_PAGE_SIZE != DEVICE_CONFIG_FLASH_PAGE_SIZE)
#error "DeviceConfig Flash page size does not match STM32F103xB"
#endif

typedef struct
{
  uint32_t magic;
  uint16_t version;
  uint16_t payload_length;
  uint32_t sequence;
  DeviceConfig_t payload;
  uint32_t crc32;
  uint16_t record_reserved;
  uint16_t commit;
} DeviceConfigFlashRecord_t;

typedef char DeviceConfigPayloadSizeCheck[
  (sizeof(DeviceConfig_t) == 12U) ? 1 : -1];
typedef char DeviceConfigRecordSizeCheck[
  (sizeof(DeviceConfigFlashRecord_t) == DEVICE_CONFIG_RECORD_SIZE) ? 1 : -1];
typedef char DeviceConfigCrcOffsetCheck[
  (offsetof(DeviceConfigFlashRecord_t, crc32) == 24U) ? 1 : -1];
typedef char DeviceConfigCommitOffsetCheck[
  (offsetof(DeviceConfigFlashRecord_t, commit) == 30U) ? 1 : -1];

typedef struct
{
  uint8_t has_valid_record;
  uint32_t first_erased_address;
  uint32_t latest_address;
  DeviceConfigFlashRecord_t latest_record;
} DeviceConfigPageScan_t;

typedef struct
{
  DeviceConfigPageScan_t page_a;
  DeviceConfigPageScan_t page_b;
  uint8_t has_latest_record;
  uint32_t latest_address;
  DeviceConfigFlashRecord_t latest_record;
} DeviceConfigFlashScan_t;

static DeviceConfig_t device_config_current;
static uint8_t device_config_loaded_from_flash;
static uint8_t device_config_dirty;
static uint8_t device_config_flash_operation_active;
static uint32_t device_config_sequence;

static DeviceConfig_t DeviceConfig_MakeDefaults(void)
{
  DeviceConfig_t defaults;

  defaults.water_empty_frequency_hz = (uint32_t)SENSOR_WATER_EMPTY_COUNT;
  defaults.water_full_frequency_hz = (uint32_t)SENSOR_WATER_FULL_COUNT;
  defaults.water_max_liters = (uint16_t)SENSOR_WATER_MAX_LITERS;
  defaults.reserved = 0U;
  return defaults;
}

/* 调度器启动前直接访问；启动后使用短调度器锁保护共享状态。 */
static uint8_t DeviceConfig_LockScheduler(void)
{
  if (xTaskGetSchedulerState() != taskSCHEDULER_NOT_STARTED)
  {
    vTaskSuspendAll();
    return 1U;
  }

  return 0U;
}

static void DeviceConfig_UnlockScheduler(uint8_t locked)
{
  if (locked != 0U)
  {
    (void)xTaskResumeAll();
  }
}

static uint8_t DeviceConfig_IsFlashAvailableInternal(void)
{
  uint16_t flash_size_kb;

  flash_size_kb = *((volatile const uint16_t *)FLASH_SIZE_DATA_REGISTER);
  return ((flash_size_kb == DEVICE_CONFIG_FLASH_MIN_KB) ||
          (flash_size_kb == DEVICE_CONFIG_FLASH_ALT_KB)) ? 1U : 0U;
}

static uint32_t DeviceConfig_Crc32(const uint8_t *data, uint32_t length)
{
  uint32_t crc;
  uint32_t index;
  uint8_t bit;

  crc = 0xFFFFFFFFUL;
  for (index = 0UL; index < length; index++)
  {
    crc ^= (uint32_t)data[index];
    for (bit = 0U; bit < 8U; bit++)
    {
      if ((crc & 1UL) != 0UL)
      {
        crc = (crc >> 1U) ^ 0xEDB88320UL;
      }
      else
      {
        crc >>= 1U;
      }
    }
  }

  return crc ^ 0xFFFFFFFFUL;
}

static uint8_t DeviceConfig_IsSequenceNewer(uint32_t candidate,
                                             uint32_t reference)
{
  uint32_t delta;

  delta = candidate - reference;
  return ((delta != 0UL) && (delta < 0x80000000UL)) ? 1U : 0U;
}

static uint8_t DeviceConfig_IsSlotErased(uint32_t address)
{
  uint32_t word_index;

  for (word_index = 0UL;
       word_index < (DEVICE_CONFIG_RECORD_SIZE / sizeof(uint32_t));
       word_index++)
  {
    if (*((volatile const uint32_t *)(address +
          (word_index * sizeof(uint32_t)))) != 0xFFFFFFFFUL)
    {
      return 0U;
    }
  }

  return 1U;
}

static void DeviceConfig_ReadRecord(uint32_t address,
                                    DeviceConfigFlashRecord_t *record)
{
  uint32_t word_index;
  uint32_t *destination;

  destination = (uint32_t *)record;
  for (word_index = 0UL;
       word_index < (DEVICE_CONFIG_RECORD_SIZE / sizeof(uint32_t));
       word_index++)
  {
    destination[word_index] =
      *((volatile const uint32_t *)(address +
        (word_index * sizeof(uint32_t))));
  }
}

static uint8_t DeviceConfig_IsRecordValid(
  const DeviceConfigFlashRecord_t *record)
{
  uint32_t crc;

  if ((record->commit != DEVICE_CONFIG_RECORD_COMMIT) ||
      (record->magic != DEVICE_CONFIG_RECORD_MAGIC) ||
      (record->version != DEVICE_CONFIG_RECORD_VERSION) ||
      (record->payload_length != sizeof(DeviceConfig_t)))
  {
    return 0U;
  }

  if (DeviceConfig_Validate(&record->payload) != DEVICE_CONFIG_STATUS_OK)
  {
    return 0U;
  }

  crc = DeviceConfig_Crc32(
    (const uint8_t *)record,
    (uint32_t)offsetof(DeviceConfigFlashRecord_t, crc32));
  return (crc == record->crc32) ? 1U : 0U;
}

static void DeviceConfig_ScanPage(uint32_t page_address,
                                  DeviceConfigPageScan_t *scan)
{
  uint32_t slot_index;
  uint32_t address;
  DeviceConfigFlashRecord_t record;

  memset(scan, 0, sizeof(*scan));
  for (slot_index = 0UL;
       slot_index < DEVICE_CONFIG_RECORDS_PER_PAGE;
       slot_index++)
  {
    address = page_address + (slot_index * DEVICE_CONFIG_RECORD_SIZE);
    if (DeviceConfig_IsSlotErased(address) != 0U)
    {
      if (scan->first_erased_address == 0UL)
      {
        scan->first_erased_address = address;
      }
      continue;
    }

    DeviceConfig_ReadRecord(address, &record);
    if (DeviceConfig_IsRecordValid(&record) == 0U)
    {
      /* 掉电留下的未提交记录永久跳过，下一次使用后续空槽。 */
      continue;
    }

    if ((scan->has_valid_record == 0U) ||
        (DeviceConfig_IsSequenceNewer(record.sequence,
                                      scan->latest_record.sequence) != 0U))
    {
      scan->has_valid_record = 1U;
      scan->latest_address = address;
      scan->latest_record = record;
    }
  }
}

static DeviceConfigStatus_t DeviceConfig_ScanFlash(
  DeviceConfigFlashScan_t *scan)
{
  if (scan == NULL)
  {
    return DEVICE_CONFIG_STATUS_INVALID_ARGUMENT;
  }

  memset(scan, 0, sizeof(*scan));
  if (DeviceConfig_IsFlashAvailableInternal() == 0U)
  {
    return DEVICE_CONFIG_STATUS_FLASH_UNAVAILABLE;
  }

  DeviceConfig_ScanPage(DEVICE_CONFIG_FLASH_PAGE_A_ADDRESS, &scan->page_a);
  DeviceConfig_ScanPage(DEVICE_CONFIG_FLASH_PAGE_B_ADDRESS, &scan->page_b);

  if (scan->page_a.has_valid_record != 0U)
  {
    scan->has_latest_record = 1U;
    scan->latest_address = scan->page_a.latest_address;
    scan->latest_record = scan->page_a.latest_record;
  }

  if ((scan->page_b.has_valid_record != 0U) &&
      ((scan->has_latest_record == 0U) ||
       (DeviceConfig_IsSequenceNewer(scan->page_b.latest_record.sequence,
                                     scan->latest_record.sequence) != 0U)))
  {
    scan->has_latest_record = 1U;
    scan->latest_address = scan->page_b.latest_address;
    scan->latest_record = scan->page_b.latest_record;
  }

  return DEVICE_CONFIG_STATUS_OK;
}

static uint8_t DeviceConfig_IsPageErased(uint32_t page_address)
{
  uint32_t offset;

  for (offset = 0UL;
       offset < DEVICE_CONFIG_FLASH_PAGE_SIZE;
       offset += sizeof(uint32_t))
  {
    if (*((volatile const uint32_t *)(page_address + offset)) !=
        0xFFFFFFFFUL)
    {
      return 0U;
    }
  }

  return 1U;
}

static DeviceConfigStatus_t DeviceConfig_ErasePage(uint32_t page_address)
{
  FLASH_EraseInitTypeDef erase;
  uint32_t page_error;

  erase.TypeErase = FLASH_TYPEERASE_PAGES;
  erase.PageAddress = page_address;
  erase.NbPages = 1UL;
  page_error = 0xFFFFFFFFUL;

  if ((HAL_FLASHEx_Erase(&erase, &page_error) != HAL_OK) ||
      (page_error != 0xFFFFFFFFUL))
  {
    return DEVICE_CONFIG_STATUS_FLASH_ERASE_FAILED;
  }

  if (DeviceConfig_IsPageErased(page_address) == 0U)
  {
    return DEVICE_CONFIG_STATUS_FLASH_VERIFY_FAILED;
  }

  return DEVICE_CONFIG_STATUS_OK;
}

static DeviceConfigStatus_t DeviceConfig_ProgramHalfword(uint32_t address,
                                                          uint16_t value)
{
  if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD,
                        address,
                        (uint64_t)value) != HAL_OK)
  {
    return DEVICE_CONFIG_STATUS_FLASH_PROGRAM_FAILED;
  }

  if (*((volatile const uint16_t *)address) != value)
  {
    return DEVICE_CONFIG_STATUS_FLASH_VERIFY_FAILED;
  }

  return DEVICE_CONFIG_STATUS_OK;
}

static DeviceConfigStatus_t DeviceConfig_ProgramRecord(
  uint32_t address,
  const DeviceConfigFlashRecord_t *record)
{
  uint32_t offset;
  uint16_t value;
  DeviceConfigStatus_t status;
  DeviceConfigFlashRecord_t verify_record;

  /*
   * commit 字段位于最后一个半字。先写其余30字节，全部成功后再提交，
   * 掉电时未提交槽不会被启动扫描接受。
   */
  for (offset = 0UL;
       offset < (uint32_t)offsetof(DeviceConfigFlashRecord_t, commit);
       offset += sizeof(uint16_t))
  {
    memcpy(&value, ((const uint8_t *)record) + offset, sizeof(value));
    status = DeviceConfig_ProgramHalfword(address + offset, value);
    if (status != DEVICE_CONFIG_STATUS_OK)
    {
      return status;
    }
  }

  status = DeviceConfig_ProgramHalfword(
    address + (uint32_t)offsetof(DeviceConfigFlashRecord_t, commit),
    record->commit);
  if (status != DEVICE_CONFIG_STATUS_OK)
  {
    return status;
  }

  DeviceConfig_ReadRecord(address, &verify_record);
  if ((DeviceConfig_IsRecordValid(&verify_record) == 0U) ||
      (verify_record.sequence != record->sequence) ||
      (memcmp(&verify_record.payload,
              &record->payload,
              sizeof(DeviceConfig_t)) != 0))
  {
    return DEVICE_CONFIG_STATUS_FLASH_VERIFY_FAILED;
  }

  return DEVICE_CONFIG_STATUS_OK;
}

static void DeviceConfig_SelectWriteLocation(
  const DeviceConfigFlashScan_t *scan,
  uint32_t *write_address,
  uint32_t *erase_page_address)
{
  const DeviceConfigPageScan_t *latest_page;
  const DeviceConfigPageScan_t *other_page;
  uint32_t latest_page_address;
  uint32_t other_page_address;

  *write_address = 0UL;
  *erase_page_address = 0UL;

  if (scan->has_latest_record == 0U)
  {
    if (scan->page_a.first_erased_address != 0UL)
    {
      *write_address = scan->page_a.first_erased_address;
    }
    else if (scan->page_b.first_erased_address != 0UL)
    {
      *write_address = scan->page_b.first_erased_address;
    }
    else
    {
      *erase_page_address = DEVICE_CONFIG_FLASH_PAGE_A_ADDRESS;
      *write_address = DEVICE_CONFIG_FLASH_PAGE_A_ADDRESS;
    }
    return;
  }

  if (scan->latest_address < DEVICE_CONFIG_FLASH_PAGE_B_ADDRESS)
  {
    latest_page = &scan->page_a;
    other_page = &scan->page_b;
    latest_page_address = DEVICE_CONFIG_FLASH_PAGE_A_ADDRESS;
    other_page_address = DEVICE_CONFIG_FLASH_PAGE_B_ADDRESS;
  }
  else
  {
    latest_page = &scan->page_b;
    other_page = &scan->page_a;
    latest_page_address = DEVICE_CONFIG_FLASH_PAGE_B_ADDRESS;
    other_page_address = DEVICE_CONFIG_FLASH_PAGE_A_ADDRESS;
  }

  if (latest_page->first_erased_address != 0UL)
  {
    *write_address = latest_page->first_erased_address;
  }
  else if (other_page->first_erased_address != 0UL)
  {
    *write_address = other_page->first_erased_address;
  }
  else
  {
    /*
     * 两页都满时只擦除不含最新记录的页。直到新记录完成提交，
     * latest_page_address 所在页始终保留一份有效旧配置。
     */
    (void)latest_page_address;
    *erase_page_address = other_page_address;
    *write_address = other_page_address;
  }
}

static void DeviceConfig_FinishFlashOperation(
  DeviceConfigStatus_t status,
  const DeviceConfig_t *saved_config,
  uint32_t saved_sequence)
{
  uint8_t locked;

  locked = DeviceConfig_LockScheduler();
  if ((status == DEVICE_CONFIG_STATUS_OK) && (saved_config != NULL))
  {
    device_config_sequence = saved_sequence;
    if (memcmp(&device_config_current,
               saved_config,
               sizeof(DeviceConfig_t)) == 0)
    {
      device_config_loaded_from_flash = 1U;
      device_config_dirty = 0U;
    }
    else
    {
      /* 写入期间RAM配置发生变化时，保留dirty，等待下一次保存。 */
      device_config_loaded_from_flash = 0U;
      device_config_dirty = 1U;
    }
  }
  device_config_flash_operation_active = 0U;
  DeviceConfig_UnlockScheduler(locked);
}

DeviceConfigStatus_t DeviceConfig_Validate(const DeviceConfig_t *config)
{
  if (config == NULL)
  {
    return DEVICE_CONFIG_STATUS_INVALID_ARGUMENT;
  }

  if ((config->water_empty_frequency_hz <
       DEVICE_CONFIG_FREQUENCY_MIN_HZ) ||
      (config->water_empty_frequency_hz >
       DEVICE_CONFIG_FREQUENCY_MAX_HZ) ||
      (config->water_full_frequency_hz <
       DEVICE_CONFIG_FREQUENCY_MIN_HZ) ||
      (config->water_full_frequency_hz >
       DEVICE_CONFIG_FREQUENCY_MAX_HZ))
  {
    return DEVICE_CONFIG_STATUS_INVALID_VALUE;
  }

  if ((config->water_empty_frequency_hz <=
       config->water_full_frequency_hz) ||
      ((config->water_empty_frequency_hz -
        config->water_full_frequency_hz) <
       DEVICE_CONFIG_FREQUENCY_MIN_SPAN_HZ))
  {
    return DEVICE_CONFIG_STATUS_INVALID_VALUE;
  }

  if ((config->water_max_liters < DEVICE_CONFIG_WATER_MIN_LITERS) ||
      (config->water_max_liters > DEVICE_CONFIG_WATER_MAX_LITERS))
  {
    return DEVICE_CONFIG_STATUS_INVALID_VALUE;
  }

  return DEVICE_CONFIG_STATUS_OK;
}

void DeviceConfig_Init(void)
{
  DeviceConfigFlashScan_t scan;
  DeviceConfigStatus_t status;

  device_config_flash_operation_active = 0U;
  status = DeviceConfig_ScanFlash(&scan);
  if ((status == DEVICE_CONFIG_STATUS_OK) &&
      (scan.has_latest_record != 0U))
  {
    device_config_current = scan.latest_record.payload;
    device_config_loaded_from_flash = 1U;
    device_config_dirty = 0U;
    device_config_sequence = scan.latest_record.sequence;
  }
  else
  {
    device_config_current = DeviceConfig_MakeDefaults();
    device_config_loaded_from_flash = 0U;
    device_config_dirty = 1U;
    device_config_sequence = 0UL;
  }
}

void DeviceConfig_GetSnapshot(DeviceConfig_t *config)
{
  uint8_t locked;

  if (config == NULL)
  {
    return;
  }

  locked = DeviceConfig_LockScheduler();
  *config = device_config_current;
  DeviceConfig_UnlockScheduler(locked);
}

DeviceConfigStatus_t DeviceConfig_Apply(const DeviceConfig_t *config)
{
  DeviceConfigStatus_t status;
  uint8_t locked;

  status = DeviceConfig_Validate(config);
  if (status != DEVICE_CONFIG_STATUS_OK)
  {
    return status;
  }

  locked = DeviceConfig_LockScheduler();
  if (device_config_flash_operation_active != 0U)
  {
    DeviceConfig_UnlockScheduler(locked);
    return DEVICE_CONFIG_STATUS_BUSY;
  }

  if (memcmp(&device_config_current, config, sizeof(DeviceConfig_t)) != 0)
  {
    device_config_current = *config;
    device_config_loaded_from_flash = 0U;
    device_config_dirty = 1U;
  }
  DeviceConfig_UnlockScheduler(locked);
  return DEVICE_CONFIG_STATUS_OK;
}

DeviceConfigStatus_t DeviceConfig_Save(void)
{
  DeviceConfig_t config_to_save;
  DeviceConfigFlashScan_t scan;
  DeviceConfigFlashRecord_t record;
  DeviceConfigStatus_t status;
  uint8_t locked;
  uint32_t write_address;
  uint32_t erase_page_address;
  uint32_t next_sequence;

  locked = DeviceConfig_LockScheduler();
  if (device_config_flash_operation_active != 0U)
  {
    DeviceConfig_UnlockScheduler(locked);
    return DEVICE_CONFIG_STATUS_BUSY;
  }
  if (device_config_dirty == 0U)
  {
    DeviceConfig_UnlockScheduler(locked);
    return DEVICE_CONFIG_STATUS_OK;
  }

  device_config_flash_operation_active = 1U;
  config_to_save = device_config_current;
  DeviceConfig_UnlockScheduler(locked);

  status = DeviceConfig_Validate(&config_to_save);
  if (status != DEVICE_CONFIG_STATUS_OK)
  {
    DeviceConfig_FinishFlashOperation(status, NULL, 0UL);
    return status;
  }

  status = DeviceConfig_ScanFlash(&scan);
  if (status != DEVICE_CONFIG_STATUS_OK)
  {
    DeviceConfig_FinishFlashOperation(status, NULL, 0UL);
    return status;
  }

  next_sequence = (scan.has_latest_record != 0U) ?
                  (scan.latest_record.sequence + 1UL) : 1UL;
  DeviceConfig_SelectWriteLocation(&scan,
                                   &write_address,
                                   &erase_page_address);

  memset(&record, 0, sizeof(record));
  record.magic = DEVICE_CONFIG_RECORD_MAGIC;
  record.version = DEVICE_CONFIG_RECORD_VERSION;
  record.payload_length = (uint16_t)sizeof(DeviceConfig_t);
  record.sequence = next_sequence;
  record.payload = config_to_save;
  record.record_reserved = 0U;
  record.commit = DEVICE_CONFIG_RECORD_COMMIT;
  record.crc32 = DeviceConfig_Crc32(
    (const uint8_t *)&record,
    (uint32_t)offsetof(DeviceConfigFlashRecord_t, crc32));

  if (HAL_FLASH_Unlock() != HAL_OK)
  {
    (void)HAL_FLASH_Lock();
    status = DEVICE_CONFIG_STATUS_FLASH_UNAVAILABLE;
    DeviceConfig_FinishFlashOperation(status, NULL, 0UL);
    return status;
  }

  __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_EOP |
                         FLASH_FLAG_PGERR |
                         FLASH_FLAG_WRPERR);

  if (erase_page_address != 0UL)
  {
    status = DeviceConfig_ErasePage(erase_page_address);
  }
  else
  {
    status = DEVICE_CONFIG_STATUS_OK;
  }

  if (status == DEVICE_CONFIG_STATUS_OK)
  {
    status = DeviceConfig_ProgramRecord(write_address, &record);
  }

  /* 无论擦写结果如何都重新锁定Flash控制器。 */
  (void)HAL_FLASH_Lock();
  DeviceConfig_FinishFlashOperation(status,
                                    (status == DEVICE_CONFIG_STATUS_OK) ?
                                    &config_to_save : NULL,
                                    next_sequence);
  return status;
}

void DeviceConfig_LoadDefaults(void)
{
  DeviceConfig_t defaults;
  uint8_t locked;

  defaults = DeviceConfig_MakeDefaults();
  locked = DeviceConfig_LockScheduler();
  device_config_current = defaults;
  device_config_loaded_from_flash = 0U;
  device_config_dirty = 1U;
  DeviceConfig_UnlockScheduler(locked);
}

DeviceConfigStatus_t DeviceConfig_Reload(void)
{
  DeviceConfigFlashScan_t scan;
  DeviceConfig_t loaded_config;
  DeviceConfigStatus_t status;
  uint8_t loaded_from_flash;
  uint8_t locked;
  uint32_t loaded_sequence;

  locked = DeviceConfig_LockScheduler();
  if (device_config_flash_operation_active != 0U)
  {
    DeviceConfig_UnlockScheduler(locked);
    return DEVICE_CONFIG_STATUS_BUSY;
  }
  device_config_flash_operation_active = 1U;
  DeviceConfig_UnlockScheduler(locked);

  status = DeviceConfig_ScanFlash(&scan);
  if ((status == DEVICE_CONFIG_STATUS_OK) &&
      (scan.has_latest_record != 0U))
  {
    loaded_config = scan.latest_record.payload;
    loaded_from_flash = 1U;
    loaded_sequence = scan.latest_record.sequence;
  }
  else
  {
    loaded_config = DeviceConfig_MakeDefaults();
    loaded_from_flash = 0U;
    loaded_sequence = 0UL;
    if (status == DEVICE_CONFIG_STATUS_OK)
    {
      status = DEVICE_CONFIG_STATUS_DEFAULTS_USED;
    }
  }

  locked = DeviceConfig_LockScheduler();
  device_config_current = loaded_config;
  device_config_loaded_from_flash = loaded_from_flash;
  device_config_dirty = (loaded_from_flash != 0U) ? 0U : 1U;
  device_config_sequence = loaded_sequence;
  device_config_flash_operation_active = 0U;
  DeviceConfig_UnlockScheduler(locked);
  return status;
}

uint8_t DeviceConfig_IsLoadedFromFlash(void)
{
  uint8_t value;
  uint8_t locked;

  locked = DeviceConfig_LockScheduler();
  value = device_config_loaded_from_flash;
  DeviceConfig_UnlockScheduler(locked);
  return value;
}

uint8_t DeviceConfig_IsDirty(void)
{
  uint8_t value;
  uint8_t locked;

  locked = DeviceConfig_LockScheduler();
  value = device_config_dirty;
  DeviceConfig_UnlockScheduler(locked);
  return value;
}

uint32_t DeviceConfig_GetSequence(void)
{
  uint32_t value;
  uint8_t locked;

  locked = DeviceConfig_LockScheduler();
  value = device_config_sequence;
  DeviceConfig_UnlockScheduler(locked);
  return value;
}
