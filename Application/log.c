#include "log.h"
#include "motor_control.h"
#include "power_manager.h"
#include "pump_valve.h"
#include "sensor.h"
#include "system_monitor.h"
#include "temp_control.h"
#include "uart_comm.h"
#include "uart_port_config.h"
#include "usart.h"
#include "uv_lamp.h"
#include <stdarg.h>
#include <stdio.h>
#include <string.h>


// Ring buffer length.
#ifndef LOGGING_TX_BUFFER_LEN
#define LOGGING_TX_BUFFER_LEN       512U
#endif

// DMA 每次发送的最大字节数
#ifndef LOGGING_TX_DMA_CHUNK_LEN
#define LOGGING_TX_DMA_CHUNK_LEN    64U
#endif

// printf 格式化缓冲区大小
#ifndef LOGGING_PRINTF_BUFFER_LEN
#define LOGGING_PRINTF_BUFFER_LEN   288U
#endif

// Current debug output switch.
#ifndef LOGGING_ENABLE_CURRENT_DEBUG
#define LOGGING_ENABLE_CURRENT_DEBUG  1U
#endif

// Water count debug output. Set to 0U to hide WC=xxxx from status logs.
#ifndef LOGGING_ENABLE_WATER_COUNT_DEBUG
#define LOGGING_ENABLE_WATER_COUNT_DEBUG  1U
#endif

#if (LOGGING_ENABLE_WATER_COUNT_DEBUG != 0U)
#define LOGGING_WATER_COUNT_FMT       "  WC:%luHz"
#define LOGGING_WATER_COUNT_ARG       , Sensor_GetWaterFrequencyHz()
#else
#define LOGGING_WATER_COUNT_FMT
#define LOGGING_WATER_COUNT_ARG
#endif

// Retarget printf to UART.
#if defined(__CC_ARM)
#pragma import(__use_no_semihosting)
struct __FILE
{
  int handle;
};
FILE __stdout;

void _sys_exit(int x)
{
  (void)x;
}
#endif


static volatile HAL_StatusTypeDef logging_last_status = HAL_OK;
static uint8_t logging_tx_buffer[LOGGING_TX_BUFFER_LEN];
static uint8_t logging_tx_dma_buffer[LOGGING_TX_DMA_CHUNK_LEN];
static volatile uint16_t logging_tx_head;
static volatile uint16_t logging_tx_tail;                   //环形缓冲区尾索引
static volatile uint8_t logging_dma_busy;                   //DMA 发送忙标志

// Restore interrupt state.
static void Logging_RestoreIrq(uint32_t primask)
{
  if (primask == 0U)
  {
    __enable_irq();
  }
}

// Compute next ring-buffer index.
static uint16_t Logging_NextIndex(uint16_t index)
{
  index++;
  if (index >= LOGGING_TX_BUFFER_LEN)
  {
    index = 0U;
  }
  return index;
}

// Start DMA log transmission.
static void Logging_StartTxDma(void)
{
  HAL_StatusTypeDef status;
  uint32_t primask;
  uint16_t length;
  uint16_t next_tail;

  length = 0U;
  primask = __get_PRIMASK();
  __disable_irq();
  if ((logging_dma_busy != 0U) || (logging_tx_head == logging_tx_tail))
  {
    Logging_RestoreIrq(primask);
    return;
  }

  logging_dma_busy = 1U;
  next_tail = logging_tx_tail;
  while ((next_tail != logging_tx_head) && (length < LOGGING_TX_DMA_CHUNK_LEN))
  {
    logging_tx_dma_buffer[length] = logging_tx_buffer[next_tail];
    next_tail = Logging_NextIndex(next_tail);
    length++;
  }
  Logging_RestoreIrq(primask);

  if (length == 0U)
  {
    primask = __get_PRIMASK();
    __disable_irq();
    logging_dma_busy = 0U;
    Logging_RestoreIrq(primask);
    return;
  }

  status = HAL_UART_Transmit_DMA(UART_PORT_LOGGING, logging_tx_dma_buffer, length);
  logging_last_status = status;
  primask = __get_PRIMASK();
  __disable_irq();
  if (status == HAL_OK)
  {
    logging_tx_tail = next_tail;
  }
  else
  {
    logging_dma_busy = 0U;
  }
  Logging_RestoreIrq(primask);
}

// Write data into the ring buffer and start DMA.
static void Logging_WriteBuffer(const uint8_t *data, uint16_t length)
{
  uint16_t next_head;
  uint16_t index;
  uint32_t primask;
  uint8_t overflow;

  if ((data == NULL) || (length == 0U))
  {
    return;
  }

  overflow = 0U;
  primask = __get_PRIMASK();
  __disable_irq();
  for (index = 0U; index < length; index++)
  {
    next_head = Logging_NextIndex(logging_tx_head);
    if (next_head == logging_tx_tail)
    {
      overflow = 1U;
      break;
    }

    logging_tx_buffer[logging_tx_head] = data[index];
    logging_tx_head = next_head;
  }
  logging_last_status = (overflow == 0U) ? HAL_OK : HAL_BUSY;
  Logging_RestoreIrq(primask);

  Logging_StartTxDma();
}

// Retarget fputc for printf.
int fputc(int ch, FILE *f)
{
  uint8_t data;

  (void)f;
  data = (uint8_t)ch;
  Logging_WriteBuffer(&data, 1U);
  return ch;
}

// Initialize logging module.
void Logging_Init(void)
{
  uint32_t primask;

  primask = __get_PRIMASK();
  __disable_irq();
  logging_tx_head = 0U;
  logging_tx_tail = 0U;
  logging_dma_busy = 0U;
  logging_last_status = HAL_OK;
  Logging_RestoreIrq(primask);
  UART_PORT_LOGGING->Init.BaudRate = UART_PORT_LOGGING_BAUD;
  (void)HAL_UART_Init(UART_PORT_LOGGING);

  //调试信息，表明日志模块已初始化完成，UART3 可用
  Logging_Print("LOG is ready\r\n");
}

// Print a string.
void Logging_Print(const char *msg)
{
  size_t length;
  uint16_t chunk;

  if (msg != NULL)
  {
    length = strlen(msg);
    while (length > 0U)
    {
      chunk = (length > 0xFFFFU) ? 0xFFFFU : (uint16_t)length;
      Logging_WriteBuffer((const uint8_t *)msg, chunk);
      msg += chunk;
      length -= chunk;
    }
  }
}


//格式化打印，类似于printf函数
void Logging_Printf(const char *fmt, ...)
{
  va_list args;
  char buffer[LOGGING_PRINTF_BUFFER_LEN];
  int length;
  uint8_t truncated;

  if (fmt == NULL)
  {
    return;
  }

  truncated = 0U;
  va_start(args, fmt);
  length = vsnprintf(buffer, sizeof(buffer), fmt, args);
  va_end(args);

  if (length < 0)
  {
    logging_last_status = HAL_ERROR;
    return;
  }
  if ((size_t)length >= sizeof(buffer))
  {
    length = (int)(sizeof(buffer) - 1U);
    truncated = 1U;
  }

  Logging_WriteBuffer((const uint8_t *)buffer, (uint16_t)length);
  if (truncated != 0U)
  {
    logging_last_status = HAL_BUSY;
  }
}

// Periodically print system status.
void Logging_TaskProcess(void)
{
  int16_t temp_x10;
  int16_t battery_temp_x10;
  char sign;
  char battery_temp_sign;
  uint16_t battery_dv;
  uint16_t dcin_dv;
  uint8_t base_connected;
  uint8_t charging_enabled;
  temp_x10 = Sensor_GetTemperatureCx10();
  sign = '+';
  if (temp_x10 < 0)
  {
    sign = '-';
    temp_x10 = (int16_t)(-temp_x10);
  }
  battery_temp_x10 = Sensor_GetBatteryTemperatureCx10();
  battery_temp_sign = '+';
  if (battery_temp_x10 < 0)
  {
    battery_temp_sign = '-';
    battery_temp_x10 = (int16_t)(-battery_temp_x10);
  }

  battery_dv = Sensor_GetBatteryDeciVolt();
  dcin_dv = Sensor_GetDcinDeciVolt();
  base_connected = UART_Comm_IsBaseConnected();
  charging_enabled = PowerManager_IsChargingEnabled();
#if (LOGGING_ENABLE_CURRENT_DEBUG != 0U)
  Logging_Printf("[BUCKET] T:%c%d.%uC BAT_NTC:%c%d.%uC  W:%uL" LOGGING_WATER_COUNT_FMT "  BAT:%u.%uV DCIN:%u.%uV BASE:%u CHG:%u | ADC MOT:%umV PUMP:%umV | OUT M:%u P:%u H:%u UV:%u | ST:%u/%u TS:%lus CMD:%02X | I MOT:%umA PUMP:%umA | ERR:%02X/%02X\r\n",
                 sign,
                 temp_x10 / 10,
                 (uint8_t)(temp_x10 % 10),
                 battery_temp_sign,
                 battery_temp_x10 / 10,
                 (uint8_t)(battery_temp_x10 % 10),
                 Sensor_GetWaterLevelProtocol()
                 LOGGING_WATER_COUNT_ARG,
                 battery_dv / 10U,
                 battery_dv % 10U,
                 dcin_dv / 10U,
                 dcin_dv % 10U,
                 base_connected,
                 charging_enabled,
                 Sensor_GetMilliVolt(SENSOR_ADC_MOTOR_CURRENT),
                 Sensor_GetMilliVolt(SENSOR_ADC_PUMP_CURRENT),
                 Motor_GetLevel(),
                 (uint8_t)PumpValve_GetMode(),
                 Temp_IsHeating(),
                 UV_IsOn(),
                 SystemMonitor_GetMainStatus(),
                 SystemMonitor_GetSubStatus(),
                 SystemMonitor_GetTimerRemainingSec(),
                 SystemMonitor_GetCommand(),
                 Sensor_GetMotorCurrentMa(),
                 Sensor_GetPumpCurrentMa(),
                 SystemMonitor_GetErrCode1(),
                 SystemMonitor_GetErrCode2());
#else
  Logging_Printf("[BUCKET] T:%c%d.%uC BAT_NTC:%c%d.%uC  W:%uL" LOGGING_WATER_COUNT_FMT "  BAT:%u.%uV DCIN:%u.%uV BASE:%u CHG:%u | ADC MOT:%umV PUMP:%umV | OUT M:%u P:%u H:%u UV:%u | ST:%u/%u TS:%lus CMD:%02X | ERR:%02X/%02X\r\n",
                 sign,
                 temp_x10 / 10,
                 (uint8_t)(temp_x10 % 10),
                 battery_temp_sign,
                 battery_temp_x10 / 10,
                 (uint8_t)(battery_temp_x10 % 10),
                 Sensor_GetWaterLevelProtocol()
                 LOGGING_WATER_COUNT_ARG,
                 battery_dv / 10U,
                 battery_dv % 10U,
                 dcin_dv / 10U,
                 dcin_dv % 10U,
                 base_connected,
                 charging_enabled,
                 Sensor_GetMilliVolt(SENSOR_ADC_MOTOR_CURRENT),
                 Sensor_GetMilliVolt(SENSOR_ADC_PUMP_CURRENT),
                 Motor_GetLevel(),
                 (uint8_t)PumpValve_GetMode(),
                 Temp_IsHeating(),
                 UV_IsOn(),
                 SystemMonitor_GetMainStatus(),
                 SystemMonitor_GetSubStatus(),
                 SystemMonitor_GetTimerRemainingSec(),
                 SystemMonitor_GetCommand(),
                 SystemMonitor_GetErrCode1(),
                 SystemMonitor_GetErrCode2());
#endif
}

// Get last logging status.
HAL_StatusTypeDef Logging_GetLastStatus(void)
{
  return logging_last_status;
}

// DMA transmit complete callback.
void Logging_TxCpltCallback(UART_HandleTypeDef *huart)
{
  uint32_t primask;

  if (huart != UART_PORT_LOGGING)
  {
    return;
  }

  primask = __get_PRIMASK();
  __disable_irq();
  logging_dma_busy = 0U;
  Logging_RestoreIrq(primask);

  Logging_StartTxDma();
}

//串口错误回调
void Logging_ErrorCallback(UART_HandleTypeDef *huart)
{
  uint32_t primask;

  if (huart != UART_PORT_LOGGING)
  {
    return;
  }

  logging_last_status = HAL_ERROR;
  ATOMIC_CLEAR_BIT(huart->Instance->CR3, USART_CR3_DMAT);
  ATOMIC_CLEAR_BIT(huart->Instance->CR1, USART_CR1_TCIE);
  if (huart->hdmatx != NULL)
  {
    (void)HAL_DMA_Abort(huart->hdmatx);
  }
  huart->gState = HAL_UART_STATE_READY;

  primask = __get_PRIMASK();
  __disable_irq();
  logging_dma_busy = 0U;
  Logging_RestoreIrq(primask);

  Logging_StartTxDma();
}
