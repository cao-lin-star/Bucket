#include "config_console.h"

#include "device_config.h"
#include "log.h"
#include "motor_control.h"
#include "pump_valve.h"
#include "sensor.h"
#include "temp_control.h"
#include "uart_port_config.h"
#include "FreeRTOS.h"
#include "task.h"
#include "uv_lamp.h"

#include <string.h>

/* HAL 每次接收一小段文本；该缓冲区只由 UART HAL 和接收回调访问。 */
#define CONFIG_CONSOLE_RX_CHUNK_LEN       64U

/* ISR 为唯一生产者、任务为唯一消费者；实际可保存 255 个字节。 */
#define CONFIG_CONSOLE_RX_RING_LEN        256U

/* 不含结尾 '\0'；第 96 个普通字符会触发超长行丢弃。 */
#define CONFIG_CONSOLE_LINE_MAX_LEN       95U
#define CONFIG_CONSOLE_LINE_BUFFER_LEN    (CONFIG_CONSOLE_LINE_MAX_LEN + 1U)
#define CONFIG_CONSOLE_MAX_TOKENS         4U

typedef enum
{
  CONFIG_CONSOLE_DISCARD_NONE = 0,
  CONFIG_CONSOLE_DISCARD_NON_ASCII,
  CONFIG_CONSOLE_DISCARD_LINE_TOO_LONG,
  CONFIG_CONSOLE_DISCARD_RX_OVERFLOW
} ConfigConsoleDiscardReason_t;

static uint8_t config_console_rx_chunk[CONFIG_CONSOLE_RX_CHUNK_LEN];
static uint8_t config_console_rx_ring[CONFIG_CONSOLE_RX_RING_LEN];
static volatile uint16_t config_console_rx_head;
static volatile uint16_t config_console_rx_tail;
static volatile uint8_t config_console_rx_overflow;
static volatile uint8_t config_console_rx_armed;
static uint8_t config_console_initialized;

static char config_console_line[CONFIG_CONSOLE_LINE_BUFFER_LEN];
static uint16_t config_console_line_length;
static ConfigConsoleDiscardReason_t config_console_discard_reason;
static uint8_t config_console_periodic_log_enabled;

/* 恢复进入短临界区之前的中断状态，避免错误地打开原本已关闭的中断。 */
static void ConfigConsole_RestoreIrq(uint32_t primask)
{
  if (primask == 0U)
  {
    __enable_irq();
  }
}

static uint16_t ConfigConsole_NextRingIndex(uint16_t index)
{
  index++;
  if (index >= CONFIG_CONSOLE_RX_RING_LEN)
  {
    index = 0U;
  }
  return index;
}

/* ISR 单生产者写入：数据写完后再发布 head，任务不会读到半写入字节。 */
static void ConfigConsole_PushRxByteFromIsr(uint8_t byte)
{
  uint16_t head;
  uint16_t next_head;

  head = config_console_rx_head;
  next_head = ConfigConsole_NextRingIndex(head);
  if (next_head == config_console_rx_tail)
  {
    config_console_rx_overflow = 1U;
    return;
  }

  config_console_rx_ring[head] = byte;
  __DMB();
  config_console_rx_head = next_head;
}

/* 任务单消费者读取：tail 只由任务更新，因此正常收发不需要关闭中断。 */
static uint8_t ConfigConsole_PopRxByte(uint8_t *byte)
{
  uint16_t tail;

  if (byte == NULL)
  {
    return 0U;
  }

  tail = config_console_rx_tail;
  if (tail == config_console_rx_head)
  {
    return 0U;
  }

  __DMB();
  *byte = config_console_rx_ring[tail];
  __DMB();
  config_console_rx_tail = ConfigConsole_NextRingIndex(tail);
  return 1U;
}

/*
 * 环形缓冲区溢出后无法判断具体丢失位置，因此清空现存字节，并让组行器
 * 一直丢弃到后续换行符，保证残缺文本绝不会被当作有效命令执行。
 */
static uint8_t ConfigConsole_TakeAndFlushRxOverflow(void)
{
  uint32_t primask;
  uint8_t overflow;

  if (config_console_rx_overflow == 0U)
  {
    return 0U;
  }

  primask = __get_PRIMASK();
  __disable_irq();
  overflow = config_console_rx_overflow;
  if (overflow != 0U)
  {
    config_console_rx_overflow = 0U;
    config_console_rx_tail = config_console_rx_head;
  }
  ConfigConsole_RestoreIrq(primask);
  return overflow;
}

static HAL_StatusTypeDef ConfigConsole_StartReceive(void)
{
  HAL_StatusTypeDef status;

  if (UART_PORT_LOGGING->RxState != HAL_UART_STATE_READY)
  {
    config_console_rx_armed = 0U;
    return HAL_BUSY;
  }

  status = HAL_UARTEx_ReceiveToIdle_IT(UART_PORT_LOGGING,
                                      config_console_rx_chunk,
                                      CONFIG_CONSOLE_RX_CHUNK_LEN);
  config_console_rx_armed = (status == HAL_OK) ? 1U : 0U;
  return status;
}

static const char *ConfigConsole_StatusText(DeviceConfigStatus_t status)
{
  switch (status)
  {
    case DEVICE_CONFIG_STATUS_OK:
      return "ok";
    case DEVICE_CONFIG_STATUS_INVALID_ARGUMENT:
      return "invalid_argument";
    case DEVICE_CONFIG_STATUS_INVALID_VALUE:
      return "invalid_value";
    case DEVICE_CONFIG_STATUS_FLASH_UNAVAILABLE:
      return "flash_unavailable";
    case DEVICE_CONFIG_STATUS_FLASH_ERASE_FAILED:
      return "flash_erase_failed";
    case DEVICE_CONFIG_STATUS_FLASH_PROGRAM_FAILED:
      return "flash_program_failed";
    case DEVICE_CONFIG_STATUS_FLASH_VERIFY_FAILED:
      return "flash_verify_failed";
    case DEVICE_CONFIG_STATUS_DEFAULTS_USED:
      return "defaults_used";
    case DEVICE_CONFIG_STATUS_BUSY:
      return "busy";
    default:
      return "unknown_status";
  }
}

static void ConfigConsole_PrintStatusError(const char *operation,
                                           DeviceConfigStatus_t status)
{
  Logging_Printf("[CFG] ERR %s %s\r\n",
                 (operation != NULL) ? operation : "operation",
                 ConfigConsole_StatusText(status));
}

/* 仅接受至少一个十进制数字；禁止符号、前后缀和溢出。 */
static uint8_t ConfigConsole_ParseUnsigned(const char *text, uint32_t *value)
{
  uint32_t result;
  uint32_t digit;

  if ((text == NULL) || (value == NULL) || (*text == '\0'))
  {
    return 0U;
  }

  result = 0UL;
  while (*text != '\0')
  {
    if ((*text < '0') || (*text > '9'))
    {
      return 0U;
    }
    digit = (uint32_t)(*text - '0');
    if (result > ((0xFFFFFFFFUL - digit) / 10UL))
    {
      return 0U;
    }
    result = (result * 10UL) + digit;
    text++;
  }

  *value = result;
  return 1U;
}

/* 只把普通 ASCII 空格视为分隔符；Tab 等控制字符会在组行阶段拒绝。 */
static uint8_t ConfigConsole_Tokenize(char *line, char **tokens,
                                     uint8_t max_tokens, uint8_t *token_count)
{
  char *cursor;
  uint8_t count;

  if ((line == NULL) || (tokens == NULL) || (token_count == NULL) ||
      (max_tokens == 0U))
  {
    return 0U;
  }

  cursor = line;
  count = 0U;
  while (*cursor != '\0')
  {
    while (*cursor == ' ')
    {
      cursor++;
    }
    if (*cursor == '\0')
    {
      break;
    }
    if (count >= max_tokens)
    {
      return 0U;
    }

    tokens[count++] = cursor;
    while ((*cursor != '\0') && (*cursor != ' '))
    {
      cursor++;
    }
    if (*cursor == ' ')
    {
      *cursor = '\0';
      cursor++;
    }
  }

  *token_count = count;
  return 1U;
}

static void ConfigConsole_PrintCurrent(uint8_t include_details)
{
  DeviceConfig_t config;
  const char *source;

  DeviceConfig_GetSnapshot(&config);
  if (DeviceConfig_IsLoadedFromFlash() != 0U)
  {
    source = "flash";
  }
  else if ((config.water_empty_frequency_hz ==
            (uint32_t)SENSOR_WATER_EMPTY_COUNT) &&
           (config.water_full_frequency_hz ==
            (uint32_t)SENSOR_WATER_FULL_COUNT) &&
           (config.water_max_liters == (uint16_t)SENSOR_WATER_MAX_LITERS))
  {
    source = "defaults";
  }
  else
  {
    source = "ram";
  }

  Logging_Printf("[CFG] active 0L=%luHz full=%luHz max_l=%uL "
                 "(water_12l_hz=water_full_hz)\r\n",
                 (unsigned long)config.water_empty_frequency_hz,
                 (unsigned long)config.water_full_frequency_hz,
                 (unsigned int)config.water_max_liters);
  Logging_Printf("[CFG] measure water=%luHz sensor_ok=%u\r\n",
                 (unsigned long)Sensor_GetWaterFrequencyHz(),
                 (unsigned int)Sensor_IsWaterSensorOk());
  Logging_Printf("[CFG] source=%s dirty=%u seq=%lu; compile defaults "
                 "0L=%luHz full=%luHz max_l=%uL\r\n",
                 source,
                 (unsigned int)DeviceConfig_IsDirty(),
                 (unsigned long)DeviceConfig_GetSequence(),
                 (unsigned long)SENSOR_WATER_EMPTY_COUNT,
                 (unsigned long)SENSOR_WATER_FULL_COUNT,
                 (unsigned int)SENSOR_WATER_MAX_LITERS);

  if (include_details != 0U)
  {
    Logging_Print("[CFG] future(unavailable): water curve/filter/timeout; ADC voltage/current "
                  "calibration; temperature/charge/protection thresholds\r\n");
  }
}

static uint8_t ConfigConsole_IsSaveSafe(void)
{
  if ((Temp_IsEnabled() != 0U) ||
      (Motor_GetLevel() != 0U) ||
      (PumpValve_GetMode() != PUMP_VALVE_MODE_OFF) ||
      (UV_IsOn() != 0U))
  {
    return 0U;
  }
  return 1U;
}

static void ConfigConsole_HandleSet(char **tokens, uint8_t token_count)
{
  DeviceConfig_t config;
  DeviceConfigStatus_t status;
  uint32_t value;

  if ((token_count != 3U) ||
      (ConfigConsole_ParseUnsigned(tokens[2], &value) == 0U))
  {
    Logging_Print("[CFG] ERR set syntax/value\r\n");
    return;
  }

  DeviceConfig_GetSnapshot(&config);
  if (strcmp(tokens[1], "water_0l_hz") == 0)
  {
    config.water_empty_frequency_hz = value;
  }
  else if ((strcmp(tokens[1], "water_12l_hz") == 0) ||
           (strcmp(tokens[1], "water_full_hz") == 0))
  {
    config.water_full_frequency_hz = value;
  }
  else if (strcmp(tokens[1], "water_max_l") == 0)
  {
    if (value > 0xFFFFUL)
    {
      Logging_Print("[CFG] ERR set invalid_value\r\n");
      return;
    }
    config.water_max_liters = (uint16_t)value;
  }
  else
  {
    Logging_Print("[CFG] ERR set unknown_key\r\n");
    return;
  }

  status = DeviceConfig_Validate(&config);
  if (status != DEVICE_CONFIG_STATUS_OK)
  {
    ConfigConsole_PrintStatusError("set", status);
    return;
  }

  status = DeviceConfig_Apply(&config);
  if (status != DEVICE_CONFIG_STATUS_OK)
  {
    ConfigConsole_PrintStatusError("set", status);
    return;
  }

  Logging_Printf("[CFG] OK set %s=%lu dirty=%u\r\n",
                 tokens[1],
                 (unsigned long)value,
                 (unsigned int)DeviceConfig_IsDirty());
}

static void ConfigConsole_HandleSave(void)
{
  DeviceConfig_t config;
  DeviceConfigStatus_t status;
  uint8_t save_safe;

  DeviceConfig_GetSnapshot(&config);
  status = DeviceConfig_Validate(&config);
  if (status != DEVICE_CONFIG_STATUS_OK)
  {
    ConfigConsole_PrintStatusError("save", status);
    return;
  }

  /*
   * 将安全检查和实际保存置于同一个调度器锁中，防止其他任务在两者之间
   * 重新开启输出。这里只暂停任务切换，UART、定时器等外设中断仍保持响应。
   */
  vTaskSuspendAll();
  save_safe = ConfigConsole_IsSaveSafe();
  if (save_safe != 0U)
  {
    status = DeviceConfig_Save();
  }
  (void)xTaskResumeAll();

  if (save_safe == 0U)
  {
    Logging_Print("[CFG] ERR save unsafe: stop heat/motor/pump/UV\r\n");
    return;
  }

  if (status != DEVICE_CONFIG_STATUS_OK)
  {
    ConfigConsole_PrintStatusError("save", status);
    return;
  }

  Logging_Printf("[CFG] OK save seq=%lu\r\n",
                 (unsigned long)DeviceConfig_GetSequence());
}

static void ConfigConsole_HandleDefaults(void)
{
  DeviceConfig_t config;
  DeviceConfigStatus_t status;

  DeviceConfig_LoadDefaults();
  DeviceConfig_GetSnapshot(&config);
  status = DeviceConfig_Validate(&config);
  if (status != DEVICE_CONFIG_STATUS_OK)
  {
    ConfigConsole_PrintStatusError("defaults", status);
    return;
  }

  Logging_Print("[CFG] OK defaults active (not saved)\r\n");
}

static void ConfigConsole_HandleReload(void)
{
  DeviceConfigStatus_t status;

  status = DeviceConfig_Reload();
  if (status == DEVICE_CONFIG_STATUS_OK)
  {
    Logging_Print("[CFG] OK reload source=flash\r\n");
  }
  else if (status == DEVICE_CONFIG_STATUS_DEFAULTS_USED)
  {
    Logging_Print("[CFG] OK reload source=defaults\r\n");
  }
  else
  {
    ConfigConsole_PrintStatusError("reload", status);
  }
}

static void ConfigConsole_ProcessLine(char *line)
{
  char *tokens[CONFIG_CONSOLE_MAX_TOKENS];
  uint8_t token_count;

  token_count = 0U;
  if ((ConfigConsole_Tokenize(line, tokens, CONFIG_CONSOLE_MAX_TOKENS,
                              &token_count) == 0U) ||
      (token_count == 0U))
  {
    Logging_Print("[CFG] ERR syntax\r\n");
    return;
  }

  if (strcmp(tokens[0], "config") == 0)
  {
    if (token_count != 1U)
    {
      Logging_Print("[CFG] ERR config syntax\r\n");
      return;
    }
    config_console_periodic_log_enabled = 0U;
    ConfigConsole_PrintCurrent(1U);
  }
  else if (strcmp(tokens[0], "show") == 0)
  {
    if (token_count != 1U)
    {
      Logging_Print("[CFG] ERR show syntax\r\n");
      return;
    }
    ConfigConsole_PrintCurrent(0U);
  }
  else if (strcmp(tokens[0], "set") == 0)
  {
    ConfigConsole_HandleSet(tokens, token_count);
  }
  else if (strcmp(tokens[0], "save") == 0)
  {
    if (token_count != 1U)
    {
      Logging_Print("[CFG] ERR save syntax\r\n");
      return;
    }
    ConfigConsole_HandleSave();
  }
  else if (strcmp(tokens[0], "defaults") == 0)
  {
    if (token_count != 1U)
    {
      Logging_Print("[CFG] ERR defaults syntax\r\n");
      return;
    }
    ConfigConsole_HandleDefaults();
  }
  else if (strcmp(tokens[0], "reload") == 0)
  {
    if (token_count != 1U)
    {
      Logging_Print("[CFG] ERR reload syntax\r\n");
      return;
    }
    ConfigConsole_HandleReload();
  }
  else if (strcmp(tokens[0], "log") == 0)
  {
    if (token_count != 1U)
    {
      Logging_Print("[CFG] ERR log syntax\r\n");
      return;
    }
    config_console_periodic_log_enabled = 1U;
    Logging_Print("[CFG] OK periodic log enabled\r\n");
  }
  else if (strcmp(tokens[0], "help") == 0)
  {
    if (token_count != 1U)
    {
      Logging_Print("[CFG] ERR help syntax\r\n");
      return;
    }
    Logging_Print("[CFG] config|show|set water_0l_hz N|set water_12l_hz N|"
                  "set water_max_l N\r\n");
    Logging_Print("[CFG] save|defaults|reload|log|help; alias: water_full_hz\r\n");
  }
  else
  {
    Logging_Print("[CFG] ERR unknown command; type help\r\n");
  }
}

static void ConfigConsole_ReportDiscard(ConfigConsoleDiscardReason_t reason)
{
  switch (reason)
  {
    case CONFIG_CONSOLE_DISCARD_NON_ASCII:
      Logging_Print("[CFG] ERR non_ascii/control_character\r\n");
      break;
    case CONFIG_CONSOLE_DISCARD_LINE_TOO_LONG:
      Logging_Print("[CFG] ERR line_too_long max=95\r\n");
      break;
    case CONFIG_CONSOLE_DISCARD_RX_OVERFLOW:
      Logging_Print("[CFG] ERR rx_overflow line_discarded\r\n");
      break;
    case CONFIG_CONSOLE_DISCARD_NONE:
    default:
      break;
  }
}

static void ConfigConsole_ProcessByte(uint8_t byte)
{
  if ((byte == (uint8_t)'\r') || (byte == (uint8_t)'\n'))
  {
    if (config_console_discard_reason != CONFIG_CONSOLE_DISCARD_NONE)
    {
      ConfigConsole_ReportDiscard(config_console_discard_reason);
    }
    else if (config_console_line_length > 0U)
    {
      config_console_line[config_console_line_length] = '\0';
      ConfigConsole_ProcessLine(config_console_line);
    }

    config_console_line_length = 0U;
    config_console_discard_reason = CONFIG_CONSOLE_DISCARD_NONE;
    return;
  }

  if (config_console_discard_reason != CONFIG_CONSOLE_DISCARD_NONE)
  {
    return;
  }

  /* 命令只允许可打印 7-bit ASCII；Tab、退格和所有扩展字节均拒绝。 */
  if ((byte < 0x20U) || (byte > 0x7EU))
  {
    config_console_line_length = 0U;
    config_console_discard_reason = CONFIG_CONSOLE_DISCARD_NON_ASCII;
    return;
  }

  if (config_console_line_length >= CONFIG_CONSOLE_LINE_MAX_LEN)
  {
    config_console_line_length = 0U;
    config_console_discard_reason = CONFIG_CONSOLE_DISCARD_LINE_TOO_LONG;
    return;
  }

  config_console_line[config_console_line_length++] = (char)byte;
}

void ConfigConsole_Init(void)
{
  config_console_rx_head = 0U;
  config_console_rx_tail = 0U;
  config_console_rx_overflow = 0U;
  config_console_rx_armed = 0U;
  config_console_line_length = 0U;
  config_console_discard_reason = CONFIG_CONSOLE_DISCARD_NONE;
  config_console_periodic_log_enabled = 1U;
  config_console_initialized = 1U;

  if (ConfigConsole_StartReceive() != HAL_OK)
  {
    Logging_Print("[CFG] ERR receive_init\r\n");
  }
}

void ConfigConsole_TaskProcess(void)
{
  uint8_t byte;

  if (config_console_initialized == 0U)
  {
    return;
  }

  /* 回调重启失败时由任务兜底重试，但不会打断仍在进行的 HAL 接收。 */
  if ((config_console_rx_armed == 0U) &&
      (UART_PORT_LOGGING->RxState == HAL_UART_STATE_READY))
  {
    (void)ConfigConsole_StartReceive();
  }

  if (ConfigConsole_TakeAndFlushRxOverflow() != 0U)
  {
    config_console_line_length = 0U;
    config_console_discard_reason = CONFIG_CONSOLE_DISCARD_RX_OVERFLOW;
  }

  while (ConfigConsole_PopRxByte(&byte) != 0U)
  {
    ConfigConsole_ProcessByte(byte);
    if (ConfigConsole_TakeAndFlushRxOverflow() != 0U)
    {
      config_console_line_length = 0U;
      config_console_discard_reason = CONFIG_CONSOLE_DISCARD_RX_OVERFLOW;
    }
  }
}

uint8_t ConfigConsole_IsPeriodicLogEnabled(void)
{
  return config_console_periodic_log_enabled;
}

void ConfigConsole_RxEventCallback(UART_HandleTypeDef *huart, uint16_t size)
{
  uint16_t index;

  if ((huart != UART_PORT_LOGGING) || (config_console_initialized == 0U))
  {
    return;
  }

  config_console_rx_armed = 0U;
  if (size > CONFIG_CONSOLE_RX_CHUNK_LEN)
  {
    size = CONFIG_CONSOLE_RX_CHUNK_LEN;
    config_console_rx_overflow = 1U;
  }

  for (index = 0U; index < size; index++)
  {
    ConfigConsole_PushRxByteFromIsr(config_console_rx_chunk[index]);
  }

  /* HAL 在 IDLE/接收完成回调前已恢复 RxState，立即续接下一段。 */
  (void)ConfigConsole_StartReceive();
}

void ConfigConsole_ErrorCallback(UART_HandleTypeDef *huart)
{
  if ((huart != UART_PORT_LOGGING) || (config_console_initialized == 0U))
  {
    return;
  }

  /* RX 非阻塞噪声错误时接收仍在继续；只有 READY 才允许重新启动。 */
  if (huart->RxState == HAL_UART_STATE_READY)
  {
    config_console_rx_armed = 0U;
    (void)ConfigConsole_StartReceive();
  }
}
