#include "uart_comm.h"
#include "log.h"
#include "main.h"
#include "motor_control.h"
#include "pump_valve.h"
#include "sensor.h"
#include "system_monitor.h"
#include "temp_control.h"
#include "uart_port_config.h"
#include "usart.h"
#include "uv_lamp.h"
#include <string.h>

// ͨ��״̬�ϱ����ڣ���λ ms
#ifndef UART_COMM_STATUS_PERIOD_MS
#define UART_COMM_STATUS_PERIOD_MS       1000UL
#endif

// �Լ�״̬����ʱ�䣬�ڼ�״̬֡ data[12] ���� 0x01
#ifndef UART_COMM_SELF_CHECK_DURATION_MS
#define UART_COMM_SELF_CHECK_DURATION_MS 5000UL
#endif

// Main controller command timeout. Stop bucket outputs if no valid command arrives.
#ifndef UART_COMM_MAIN_TIMEOUT_MS
#define UART_COMM_MAIN_TIMEOUT_MS        5000UL
#endif

// Base station link timeout. Used for LINK_STATUS in status frames.
#ifndef UART_COMM_BASE_TIMEOUT_MS
#define UART_COMM_BASE_TIMEOUT_MS        3000UL
#endif

#ifndef UART_COMM_BASE_DCIN_CONNECTED_DV
/* AD_DCIN above 13.0V indicates bucket/base power connection. Unit: 0.1V. */
#define UART_COMM_BASE_DCIN_CONNECTED_DV 130U
#endif
// ǿ��Э�鴮�ڲ�����Ϊ 115200
#ifndef UART_COMM_FORCE_PROTOCOL_BAUD
#define UART_COMM_FORCE_PROTOCOL_BAUD    1U
#endif

// ��������أ�1=���� data[3]=0xB4 ��ˮ�ú���ͨ����0=�ر�
#ifndef UART_COMM_ENABLE_DEBUG_B4_PUMP_VALVE
#define UART_COMM_ENABLE_DEBUG_B4_PUMP_VALVE  1U
#endif

// ���� DMA ����������
#ifndef UART_COMM_RX_DMA_BUFFER_LEN
#define UART_COMM_RX_DMA_BUFFER_LEN      128U
#endif

#define UART_CMD_IDLE                    0x00U      // ����/����
#define UART_CMD_BUCKET_OFF              0xA0U      // �ر�
#define UART_CMD_BUCKET_STANDBY          0xA1U      // ����
#define UART_CMD_BUCKET_TEMP_ON          0xA2U      // �������
#define UART_CMD_BUCKET_TEMP_OFF         0xA3U      // �رպ���
#define UART_CMD_BUCKET_MOTOR            0xA4U      // ��Ħ���
#define UART_CMD_BUCKET_UV               0xA5U      // UV ��
#define UART_CMD_BUCKET_TIMER            0xA6U      // ��ʱ
#define UART_CMD_BUCKET_STOP             0xA7U      // ֹͣ����
#define UART_CMD_BUCKET_SELF_CHECK       0xA8U      // �Լ�
#define UART_CMD_BUCKET_LOW_POWER        0xA9U      // �͹���
#define UART_CMD_BUCKET_AUTO_FILL        0xB2U      // �Զ���ˮ����վִ�м���/��ˮ��Ͱ�巴��ˮλ�¶�
#define UART_CMD_DEBUG_PUMP_VALVE_ON     0xB4U      // ���ԣ�ˮ�ÿ� + ��ͨ����
#define UART_CMD_SYSTEM_RESET            0xC0U      // ϵͳ��λ

#define UART_BASE_STATUS_CLEAN_DRAIN1    0x08U
#define UART_BASE_STATUS_CLEAR_DRAIN2    0x0AU
#define UART_BASE_STATUS_FORCE_DRAIN      0x0CU
#define UART_BASE_STATUS_CLEAN_SPRAY     0x07U
#define UART_BASE_STATUS_CLEAR_SPRAY     0x09U

// �������ṹ�壺���ջ����� + ��ǰд��λ��
typedef struct
{
  uint8_t buffer[UART_COMM_FRAME_LEN];
  uint8_t index;
} UartParser_t;

// ֡���ղۣ���������֡�;�����־
typedef struct
{
  uint8_t frame[UART_COMM_FRAME_LEN];
  volatile uint8_t ready;
} UartFrameSlot_t;

// ���Ͳۣ���ǰ����֡��������֡��æ״̬
typedef struct
{
  UART_HandleTypeDef *huart;
  uint8_t active_frame[UART_COMM_FRAME_LEN];
  uint8_t pending_frame[UART_COMM_FRAME_LEN];
  volatile uint8_t busy;
  volatile uint8_t pending;
} UartTxSlot_t;

// �������ڽ�������Linux ���غͻ�վ
static UartParser_t linux_parser;
static UartParser_t base_parser;
static UartFrameSlot_t linux_rx_slot;
static UartFrameSlot_t base_rx_slot;

// DMA ���ջ�����
static uint8_t linux_rx_dma_buffer[UART_COMM_RX_DMA_BUFFER_LEN];
static uint8_t base_rx_dma_buffer[UART_COMM_RX_DMA_BUFFER_LEN];
static uint16_t linux_rx_dma_pos;
static uint16_t base_rx_dma_pos;

// ���Ϳ���
static UartTxSlot_t linux_tx_slot;
static UartTxSlot_t base_tx_slot;

// ״̬�ϱ������س�ʱ����
static uint8_t base_data[13];
static uint32_t base_last_rx_tick;
static uint32_t main_last_rx_tick;
static uint32_t last_status_tx_tick;
static uint8_t last_link_mode = UART_COMM_DEFAULT_LINK_MODE;
static uint8_t main_timeout_handled;
static uint8_t self_check_pending;
static uint32_t self_check_start_tick;

// ϵͳ��λ����
static void UART_Comm_RestoreIrq(uint32_t primask)
{
  if (primask == 0U)
  {
    __enable_irq();
  }
}

static uint8_t UART_Comm_IsTransitMode(uint8_t link_mode)
{
  return (link_mode == UART_COMM_LINK_MODE_TRANSIT) ? 1U : 0U;
}

static uint8_t UART_Comm_IsValidLinkMode(uint8_t link_mode)
{
  if ((link_mode == UART_COMM_LINK_MODE_BUCKET) ||
      (link_mode == UART_COMM_LINK_MODE_TRANSIT) ||
      (link_mode == UART_COMM_LINK_MODE_DIRECT))
  {
    return 1U;
  }
  return 0U;
}

static void UART_Comm_RecordMainFrame(const uint8_t *frame)
{
  if (frame == NULL)
  {
    return;
  }

  last_link_mode = frame[2];
  main_last_rx_tick = HAL_GetTick();
  main_timeout_handled = 0U;
}

static void UART_Comm_HandleMainTimeout(uint32_t now)
{
  if (main_timeout_handled != 0U)
  {
    return;
  }

  if ((now - main_last_rx_tick) < UART_COMM_MAIN_TIMEOUT_MS)
  {
    return;
  }

  SystemMonitor_StopAllOutputs();   // ����ͨ�ų�ʱ��ֹͣ�������
  SystemMonitor_SetBathTimer(0U);   // ����ͨ�ų�ʱ�������ʱ
  SystemMonitor_SetCommand(UART_CMD_BUCKET_STOP);
  SystemMonitor_SetMainStatus(BUCKET_STATUS_STANDBY, 0U);
  main_timeout_handled = 1U;
}

// ����֡У���
uint8_t UART_Comm_Checksum(const uint8_t *frame)
{
  uint16_t sum;
  uint8_t index;

  sum = 0U;
  if (frame == NULL)
  {
    return 0U;
  }
  for (index = 2U; index <= 28U; index++)
  {
    sum = (uint16_t)(sum + frame[index]);
  }
  return (uint8_t)(sum & 0xFFU);
}

// ��֤֡ͷ��У���
uint8_t UART_Comm_IsFrameValid(const uint8_t *frame)
{
  if (frame == NULL)
  {
    return 0U;
  }
  if ((frame[0] != UART_COMM_HEAD1) || (frame[1] != UART_COMM_HEAD2))
  {
    return 0U;
  }
  return (UART_Comm_Checksum(frame) == frame[29]) ? 1U : 0U;
}

// ������յ�������֡����λ
static void UART_Comm_StoreFrame(UartFrameSlot_t *slot, const uint8_t *frame)
{
  if ((slot == NULL) || (frame == NULL))
  {
    return;
  }
  memcpy(slot->frame, frame, UART_COMM_FRAME_LEN);
  slot->ready = 1U;
}

// ��������������ֽڣ��Զ�ʶ��֡ͷ����װ����֡
static void UART_Comm_ParserPush(UartParser_t *parser, UartFrameSlot_t *slot, uint8_t byte)
{
  if ((parser == NULL) || (slot == NULL))
  {
    return;
  }

  if (parser->index == 0U)
  {
    if (byte != UART_COMM_HEAD1)
    {
      return;
    }
    parser->buffer[parser->index++] = byte;
    return;
  }

  if (parser->index == 1U)
  {
    if (byte != UART_COMM_HEAD2)
    {
      parser->index = (byte == UART_COMM_HEAD1) ? 1U : 0U;
      parser->buffer[0] = UART_COMM_HEAD1;
      return;
    }
    parser->buffer[parser->index++] = byte;
    return;
  }

  parser->buffer[parser->index++] = byte;
  if (parser->index >= UART_COMM_FRAME_LEN)
  {
    if (UART_Comm_IsFrameValid(parser->buffer) != 0U)
    {
      UART_Comm_StoreFrame(slot, parser->buffer);
    }
    parser->index = 0U;
  }
}

// ���ԴӲ�λȡ������֡���ɹ����� 1�����򷵻� 0
static uint8_t UART_Comm_FetchFrame(UartFrameSlot_t *slot, uint8_t *out_frame)
{
  uint8_t ready;
  uint32_t primask;

  if ((slot == NULL) || (out_frame == NULL))
  {
    return 0U;
  }

  primask = __get_PRIMASK();
  __disable_irq();
  ready = slot->ready;
  if (ready != 0U)
  {
    memcpy(out_frame, slot->frame, UART_COMM_FRAME_LEN);
    slot->ready = 0U;
  }
  UART_Comm_RestoreIrq(primask);

  return ready;
}

// ���� UART �����ȡ��Ӧ���Ͳ�
static UartTxSlot_t *UART_Comm_GetTxSlot(UART_HandleTypeDef *huart)
{
  if (huart == UART_PORT_LINUX)
  {
    return &linux_tx_slot;
  }
  if (huart == UART_PORT_BASE)
  {
    return &base_tx_slot;
  }
  return NULL;
}

// �����· DMA ���գ��ɹ����� HAL_OK
static HAL_StatusTypeDef UART_Comm_StartReceiveOne(UART_HandleTypeDef *huart, uint8_t *buffer, uint16_t *old_pos)
{
  HAL_StatusTypeDef status;

  if ((huart == NULL) || (buffer == NULL) || (old_pos == NULL))
  {
    return HAL_ERROR;
  }

  *old_pos = 0U;
  status = HAL_UARTEx_ReceiveToIdle_DMA(huart, buffer, UART_COMM_RX_DMA_BUFFER_LEN);
  if ((status == HAL_OK) && (huart->hdmarx != NULL))
  {
    __HAL_DMA_DISABLE_IT(huart->hdmarx, DMA_IT_HT);
  }
  return status;
}

// ֹͣ��· DMA ����
static void UART_Comm_StopReceiveOne(UART_HandleTypeDef *huart)
{
  if (huart == NULL)
  {
    return;
  }

  ATOMIC_CLEAR_BIT(huart->Instance->CR3, USART_CR3_DMAR);
  if (huart->hdmarx != NULL)
  {
    (void)HAL_DMA_Abort(huart->hdmarx);
  }
  ATOMIC_CLEAR_BIT(huart->Instance->CR1, (USART_CR1_PEIE | USART_CR1_IDLEIE));
  ATOMIC_CLEAR_BIT(huart->Instance->CR3, USART_CR3_EIE);
  huart->RxState = HAL_UART_STATE_READY;
  huart->ReceptionType = HAL_UART_RECEPTION_STANDARD;
}

// ��� Linux �ͻ�վ�� DMA ����
static void UART_Comm_StartReceive(void)
{
  memset(linux_rx_dma_buffer, 0, sizeof(linux_rx_dma_buffer));
  memset(base_rx_dma_buffer, 0, sizeof(base_rx_dma_buffer));
  (void)UART_Comm_StartReceiveOne(UART_PORT_LINUX, linux_rx_dma_buffer, &linux_rx_dma_pos);
  (void)UART_Comm_StartReceiveOne(UART_PORT_BASE, base_rx_dma_buffer, &base_rx_dma_pos);
}

// ���� DMA �����������ݣ����������
static void UART_Comm_ProcessRxRange(UartParser_t *parser, UartFrameSlot_t *slot,
                                     const uint8_t *buffer, uint16_t start, uint16_t end)
{
  uint16_t index;

  for (index = start; index < end; index++)
  {
    UART_Comm_ParserPush(parser, slot, buffer[index]);
  }
}

// ���� DMA �������ݣ����ݻ��λ������
static void UART_Comm_ProcessDmaRx(UartParser_t *parser, UartFrameSlot_t *slot,
                                   const uint8_t *buffer, uint16_t *old_pos, uint16_t pos)
{
  if ((parser == NULL) || (slot == NULL) || (buffer == NULL) || (old_pos == NULL))
  {
    return;
  }
  if (pos > UART_COMM_RX_DMA_BUFFER_LEN)
  {
    return;
  }

  if (pos == *old_pos)
  {
    return;
  }

  if (pos > *old_pos)
  {
    UART_Comm_ProcessRxRange(parser, slot, buffer, *old_pos, pos);
  }
  else
  {
    UART_Comm_ProcessRxRange(parser, slot, buffer, *old_pos, UART_COMM_RX_DMA_BUFFER_LEN);
    UART_Comm_ProcessRxRange(parser, slot, buffer, 0U, pos);
  }
  *old_pos = pos;
}

// ����������ͣ�������ڷ��ͣ����������֡
static void UART_Comm_TryStartTx(UartTxSlot_t *slot)
{
  HAL_StatusTypeDef status;
  uint32_t primask;
  uint8_t should_start;

  if ((slot == NULL) || (slot->huart == NULL))
  {
    return;
  }

  should_start = 0U;
  primask = __get_PRIMASK();
  __disable_irq();
  if ((slot->busy == 0U) && (slot->pending != 0U))
  {
    memcpy(slot->active_frame, slot->pending_frame, UART_COMM_FRAME_LEN);
    slot->pending = 0U;
    slot->busy = 1U;
    should_start = 1U;
  }
  UART_Comm_RestoreIrq(primask);

  if (should_start != 0U)
  {
    status = HAL_UART_Transmit_DMA(slot->huart, slot->active_frame, UART_COMM_FRAME_LEN);
    if (status != HAL_OK)
    {
      primask = __get_PRIMASK();
      __disable_irq();
      memcpy(slot->pending_frame, slot->active_frame, UART_COMM_FRAME_LEN);
      slot->pending = 1U;
      slot->busy = 0U;
      UART_Comm_RestoreIrq(primask);
    }
  }
}

// ����һ֡���ݣ���������æʱ���Ǵ�����֡
static void UART_Comm_SendFrame(UART_HandleTypeDef *huart, const uint8_t *frame)
{
  UartTxSlot_t *slot;
  uint32_t primask;

  if ((huart == NULL) || (frame == NULL))
  {
    return;
  }

  slot = UART_Comm_GetTxSlot(huart);
  if (slot == NULL)
  {
    return;
  }

  primask = __get_PRIMASK();
  __disable_irq();
  memcpy(slot->pending_frame, frame, UART_COMM_FRAME_LEN);
  slot->pending = 1U;
  UART_Comm_RestoreIrq(primask);

  UART_Comm_TryStartTx(slot);
}

static uint8_t UART_Comm_GetTemperatureProtocol(void)
{
  float temp_c;

  temp_c = Sensor_GetTemperatureC();
  if (Sensor_IsTempSensorOk() != 0U)
  {
    return (uint8_t)(temp_c + 0.5f);
  }
  return 0U;
}

// �� Linux ֡ת������վ��0x00 ����֡��Я��Ͱ��ʵʱˮλ���¶�
static void UART_Comm_ForwardToBase(const uint8_t *frame)
{
  if (frame != NULL)
  {
    uint8_t forward_frame[UART_COMM_FRAME_LEN];

    if (frame[3] == UART_CMD_IDLE)
    {
      memcpy(forward_frame, frame, sizeof(forward_frame));
      forward_frame[5] = Sensor_GetWaterLevelProtocol();
      forward_frame[6] = UART_Comm_GetTemperatureProtocol();
      forward_frame[29] = UART_Comm_Checksum(forward_frame);
      UART_Comm_SendFrame(UART_PORT_BASE, forward_frame);
      return;
    }

    UART_Comm_SendFrame(UART_PORT_BASE, frame);
  }
}

// �����Զ���ˮ���� B2
static void UART_Comm_ProcessAutoFillCommand(const uint8_t *frame)
{
  uint8_t is_transit;

  is_transit = UART_Comm_IsTransitMode(frame[2]);

  Temp_Enable(0U);
  SystemMonitor_SetCommand(UART_CMD_BUCKET_AUTO_FILL);
  SystemMonitor_SetMainStatus(BUCKET_STATUS_RUNNING, 0U);

  if (is_transit != 0U)
  {
    UART_Comm_ForwardToBase(frame);
  }
}

static void UART_Comm_ProcessBucketCommand(const uint8_t *frame)
{
  uint8_t cmd;

  cmd = frame[3];

  // �Լ��ڼ��������Ͱ����������������رպ��Լ�״̬��
  if ((self_check_pending != 0U) && (cmd != UART_CMD_BUCKET_SELF_CHECK))
  {
    return;
  }


  switch (cmd)
  {
    case UART_CMD_IDLE:
      break;

    case UART_CMD_BUCKET_OFF:
      SystemMonitor_StopAllOutputs();
      SystemMonitor_SetBathTimer(0U);
      SystemMonitor_SetMainStatus(BUCKET_STATUS_OFF, 0U);
      break;

    case UART_CMD_BUCKET_STANDBY:
      SystemMonitor_StopAllOutputs();
      SystemMonitor_SetBathTimer(0U);
      SystemMonitor_SetMainStatus(BUCKET_STATUS_STANDBY, 0U);
      break;

    case UART_CMD_BUCKET_TEMP_ON:
      Temp_SetTargetC((float)frame[6]);
      Temp_Enable(1U);
      PumpValve_SetMode(PUMP_VALVE_MODE_CIRCULATION);
      SystemMonitor_SetMainStatus(BUCKET_STATUS_RUNNING, frame[9]);
      break;

    case UART_CMD_BUCKET_TEMP_OFF:
      Temp_Enable(0U);
      if (UV_IsOn() == 0U)
      {
        PumpValve_SetMode(PUMP_VALVE_MODE_OFF);
      }
      SystemMonitor_SetMainStatus(BUCKET_STATUS_RUNNING, frame[9]);
      break;

    case UART_CMD_BUCKET_MOTOR:
      Motor_SetLevel(frame[7]);
      SystemMonitor_SetMainStatus((frame[7] == 0U) ? BUCKET_STATUS_STANDBY : BUCKET_STATUS_RUNNING, frame[9]);
      break;

    case UART_CMD_BUCKET_UV:
      UV_Set(frame[8]);
 
      if (frame[8] != 0U)
      {
        PumpValve_SetMode(PUMP_VALVE_MODE_CIRCULATION);
      }
      else if (Temp_IsEnabled() == 0U)
      {
        PumpValve_SetMode(PUMP_VALVE_MODE_OFF);
      }
      SystemMonitor_SetMainStatus((frame[8] == 0U) ? BUCKET_STATUS_STANDBY : BUCKET_STATUS_RUNNING, frame[9]);
      break;

    case UART_CMD_BUCKET_TIMER:
      SystemMonitor_SetBathTimer(frame[9]);
      SystemMonitor_SetMainStatus(BUCKET_STATUS_RUNNING, SystemMonitor_GetTimerRemainingMin());
      break;

    case UART_CMD_BUCKET_STOP:
      SystemMonitor_StopAllOutputs();
      SystemMonitor_SetBathTimer(0U);
      SystemMonitor_SetMainStatus(BUCKET_STATUS_STANDBY, 0U);
      break;

    case UART_CMD_BUCKET_SELF_CHECK:
      SystemMonitor_StopAllOutputs();
      SystemMonitor_ClearErrors();
      SystemMonitor_SetMainStatus(BUCKET_STATUS_SELF_CHECK, 0U);
      self_check_start_tick = HAL_GetTick();
      self_check_pending = 1U;
      break;

    case UART_CMD_BUCKET_LOW_POWER:
      SystemMonitor_StopAllOutputs();
      SystemMonitor_SetMainStatus(BUCKET_STATUS_LOW_POWER, 0U);
      break;

    default:
      break;
  }
}

// ����ϵͳ����֡
static void UART_Comm_ProcessSystemCommand(const uint8_t *frame)
{
  SystemMonitor_SetCommand(frame[3]);
  if (frame[3] == UART_CMD_SYSTEM_RESET)
  {
    SystemMonitor_StopAllOutputs();
    SystemMonitor_RequestReset();
  }
}

#if (UART_COMM_ENABLE_DEBUG_B4_PUMP_VALVE != 0U)
// �������data[3]=0xB4 ʱֱ�Ӵ�ˮ�ú���ͨ��
static void UART_Comm_ProcessDebugPumpValveCommand(void)
{
  PumpValve_SetMode(PUMP_VALVE_MODE_DRAIN);
  SystemMonitor_SetCommand(UART_CMD_DEBUG_PUMP_VALVE_ON);
  SystemMonitor_SetMainStatus(BUCKET_STATUS_RUNNING, 0U);
}
#endif

// ���� Linux ����֡�������Ƿ�Ϊ��Чҵ��֡
static uint8_t UART_Comm_ProcessLinuxFrame(const uint8_t *frame)
{
  uint8_t cmd;
  uint8_t is_transit;

  if (frame == NULL)
  {
    return 0U;
  }

  if (UART_Comm_IsValidLinkMode(frame[2]) == 0U)
  {
    return 0U;
  }

  UART_Comm_RecordMainFrame(frame);
  cmd = frame[3];
  is_transit = UART_Comm_IsTransitMode(frame[2]);

  if ((self_check_pending == 0U) || (cmd == UART_CMD_BUCKET_SELF_CHECK))
  {
    if (cmd != UART_CMD_IDLE)
    {
      SystemMonitor_SetCommand(cmd);
    }
  }

#if (UART_COMM_ENABLE_DEBUG_B4_PUMP_VALVE != 0U)
  if (cmd == UART_CMD_DEBUG_PUMP_VALVE_ON)
  {
    if (is_transit != 0U)
    {
      UART_Comm_ForwardToBase(frame);
    }
    UART_Comm_ProcessDebugPumpValveCommand();
    return 1U;
  }
#endif

  if (cmd == UART_CMD_BUCKET_AUTO_FILL)
  {
    UART_Comm_ProcessAutoFillCommand(frame);
    return 1U;
  }

  if (is_transit != 0U)
  {
    UART_Comm_ForwardToBase(frame);
    if ((cmd >= 0xB0U) && (cmd <= 0xBFU))
    {
      return 1U;
    }
  }

  if ((cmd == UART_CMD_IDLE) || ((cmd >= 0xA0U) && (cmd <= 0xAFU)))
  {
    UART_Comm_ProcessBucketCommand(frame);
    return 1U;
  }
  else if ((is_transit == 0U) && (cmd >= 0xC0U) && (cmd <= 0xCFU))
  {
    UART_Comm_ProcessSystemCommand(frame);
    return 1U;
  }

  return 0U;
}

static uint8_t UART_Comm_HandleSelfCheckCompletion(uint32_t now)
{
  if ((self_check_pending == 0U) ||
      ((now - self_check_start_tick) < UART_COMM_SELF_CHECK_DURATION_MS))
  {
    return 0U;
  }

  self_check_pending = 0U;
  SystemMonitor_SetMainStatus(BUCKET_STATUS_POWER_ON, 0U);
  return 1U;
}

// �����վ״̬֡�������վ���ݲ�ͬ��Ͱ��÷�״̬
static uint8_t UART_Comm_IsBaseDrainStatus(uint8_t status)
{
  return ((status == UART_BASE_STATUS_CLEAN_DRAIN1) ||
          (status == UART_BASE_STATUS_CLEAR_DRAIN2) ||
          (status == UART_BASE_STATUS_FORCE_DRAIN)) ? 1U : 0U;
}

static uint8_t UART_Comm_IsBaseCirculationStatus(uint8_t status)
{
  return ((status == UART_BASE_STATUS_CLEAN_SPRAY) ||
          (status == UART_BASE_STATUS_CLEAR_SPRAY)) ? 1U : 0U;
}

static void UART_Comm_SyncPumpValveFromBaseStatus(uint8_t status)
{
  if (UART_Comm_IsBaseDrainStatus(status) != 0U)
  {
    PumpValve_SetMode(PUMP_VALVE_MODE_DRAIN);
  }
  else if (UART_Comm_IsBaseCirculationStatus(status) != 0U)
  {
    PumpValve_SetMode(PUMP_VALVE_MODE_CIRCULATION);
  }
  else if (PumpValve_GetMode() == PUMP_VALVE_MODE_DRAIN)
  {
    PumpValve_SetMode(PUMP_VALVE_MODE_OFF);
  }
  else if ((PumpValve_GetMode() == PUMP_VALVE_MODE_CIRCULATION) &&
           (Temp_IsEnabled() == 0U) &&
           (UV_IsOn() == 0U))
  {
    PumpValve_SetMode(PUMP_VALVE_MODE_OFF);
  }
}

static uint8_t UART_Comm_ProcessBaseFrame(const uint8_t *frame)
{
  uint32_t now;

  if (frame == NULL)
  {
    return 0U;
  }

  if (UART_Comm_IsTransitMode(frame[2]) == 0U)
  {
    return 0U;
  }

  memcpy(base_data, &frame[16], sizeof(base_data));
  UART_Comm_SyncPumpValveFromBaseStatus(frame[23]);
  now = HAL_GetTick();
  base_last_rx_tick = now;
  return 1U;
}

uint8_t UART_Comm_BuildStatusFrame(uint8_t *frame)
{
  uint16_t battery_dv;
  uint8_t base_connected;

  if (frame == NULL)
  {
    return 0U;
  }

  memset(frame, 0, UART_COMM_FRAME_LEN);
  frame[0] = UART_COMM_HEAD1;
  frame[1] = UART_COMM_HEAD2;
  frame[2] = last_link_mode;
  frame[3] = UART_CMD_IDLE;
  base_connected = UART_Comm_IsBaseConnected();
  frame[4] = base_connected;
  frame[5] = Sensor_GetWaterLevelProtocol();
  frame[6] = UART_Comm_GetTemperatureProtocol();
  frame[7] = Motor_GetLevel();
  frame[8] = UV_IsOn();
  frame[9] = SystemMonitor_GetTimerRemainingMin();

  battery_dv = Sensor_GetBatteryDeciVolt();
  frame[10] = (uint8_t)((battery_dv >> 8) & 0xFFU);
  frame[11] = (uint8_t)(battery_dv & 0xFFU);
  frame[12] = SystemMonitor_GetMainStatus();
  frame[13] = SystemMonitor_GetSubStatus();
  frame[14] = SystemMonitor_GetErrCode1();
  frame[15] = SystemMonitor_GetErrCode2();

  if (base_connected != 0U)
  {
    memcpy(&frame[16], base_data, sizeof(base_data));
  }
  frame[29] = UART_Comm_Checksum(frame);
  return UART_COMM_FRAME_LEN;
}

// �жϻ�վ�Ƿ����ߣ��������һ�κϷ���վ֡ʱ��
uint8_t UART_Comm_IsBaseConnected(void)
{
  if (Sensor_GetDcinDeciVolt() <= UART_COMM_BASE_DCIN_CONNECTED_DV)
  {
    return 0U;
  }
  if (base_last_rx_tick == 0UL)
  {
    return 0U;
  }
  if ((HAL_GetTick() - base_last_rx_tick) < UART_COMM_BASE_TIMEOUT_MS)
  {
    return 1U;
  }
  return 0U;
}

// �ⲿ�ӿڣ�����һ֡�������ݲ�ִ������
void UART_ParseFrame(uint8_t *data, uint8_t len)
{
  if ((data == NULL) || (len != UART_COMM_FRAME_LEN))
  {
    return;
  }
  if (UART_Comm_IsFrameValid(data) == 0U)
  {
    return;
  }
  (void)UART_Comm_ProcessLinuxFrame(data);
}

// �ⲿ�ӿڣ���ʼ��ͨ��ģ�鲢�������
void UART_Comm_Init(void)
{
  memset(&linux_parser, 0, sizeof(linux_parser));
  memset(&base_parser, 0, sizeof(base_parser));
  memset(&linux_rx_slot, 0, sizeof(linux_rx_slot));
  memset(&base_rx_slot, 0, sizeof(base_rx_slot));
  memset(&linux_tx_slot, 0, sizeof(linux_tx_slot));
  memset(&base_tx_slot, 0, sizeof(base_tx_slot));
  memset(base_data, 0, sizeof(base_data));
  linux_tx_slot.huart = UART_PORT_LINUX;
  base_tx_slot.huart = UART_PORT_BASE;
  base_last_rx_tick = 0UL;
  main_last_rx_tick = HAL_GetTick();
  last_status_tx_tick = 0UL;
  last_link_mode = UART_COMM_DEFAULT_LINK_MODE;
  main_timeout_handled = 0U;
  self_check_pending = 0U;
  self_check_start_tick = 0UL;

#if UART_COMM_FORCE_PROTOCOL_BAUD
  UART_PORT_LINUX->Init.BaudRate = UART_PORT_PROTOCOL_BAUD;
  UART_PORT_BASE->Init.BaudRate = UART_PORT_PROTOCOL_BAUD;
  HAL_UART_Init(UART_PORT_LINUX);
  HAL_UART_Init(UART_PORT_BASE);
#endif

  UART_Comm_StartReceive();
}

// �ⲿ�ӿڣ�ͨ�����񣬴����շ�������ִ�к�����״̬�ϱ�
void UART_Comm_TaskProcess(void)
{
  uint8_t frame[UART_COMM_FRAME_LEN];
  uint32_t now;

  if (UART_Comm_FetchFrame(&linux_rx_slot, frame) != 0U)
  {
    if ((UART_Comm_ProcessLinuxFrame(frame) != 0U) &&
        (frame[3] == UART_CMD_BUCKET_SELF_CHECK))
    {
      now = HAL_GetTick();
      last_status_tx_tick = now;
      UART_Comm_BuildStatusFrame(frame);
      frame[3] = UART_CMD_BUCKET_SELF_CHECK;
      frame[29] = UART_Comm_Checksum(frame);
      UART_Comm_SendFrame(UART_PORT_LINUX, frame);
    }
  }

  if (UART_Comm_FetchFrame(&base_rx_slot, frame) != 0U)
  {
    (void)UART_Comm_ProcessBaseFrame(frame);
  }

  now = HAL_GetTick();
  UART_Comm_HandleMainTimeout(now);
  if (UART_Comm_HandleSelfCheckCompletion(now) != 0U)
  {
    last_status_tx_tick = now;
    UART_Comm_BuildStatusFrame(frame);
    UART_Comm_SendFrame(UART_PORT_LINUX, frame);
  }
  if ((now - last_status_tx_tick) >= UART_COMM_STATUS_PERIOD_MS)
  {
    last_status_tx_tick = now;
    UART_Comm_BuildStatusFrame(frame);
    UART_Comm_SendFrame(UART_PORT_LINUX, frame);
  }
}

// DMA ������ɻص�������������ݲ����������
void UART_Comm_RxEventCallback(UART_HandleTypeDef *huart, uint16_t pos)
{
  if (huart == UART_PORT_LINUX)
  {
    UART_Comm_ProcessDmaRx(&linux_parser, &linux_rx_slot,
                           linux_rx_dma_buffer, &linux_rx_dma_pos, pos);
  }
  else if (huart == UART_PORT_BASE)
  {
    UART_Comm_ProcessDmaRx(&base_parser, &base_rx_slot,
                           base_rx_dma_buffer, &base_rx_dma_pos, pos);
  }
}

// DMA ������ɻص�����Ƿ�����ɲ����Է�����һ֡
void UART_Comm_TxCpltCallback(UART_HandleTypeDef *huart)
{
  UartTxSlot_t *slot;
  uint32_t primask;

  slot = UART_Comm_GetTxSlot(huart);
  if (slot == NULL)
  {
    return;
  }

  primask = __get_PRIMASK();
  __disable_irq();
  slot->busy = 0U;
  UART_Comm_RestoreIrq(primask);

  UART_Comm_TryStartTx(slot);
}

// DMA/���ڴ���ص���������ղ��ָ�����״̬
void UART_Comm_ErrorCallback(UART_HandleTypeDef *huart)
{
  UartTxSlot_t *slot;
  uint32_t primask;

  if (huart == UART_PORT_LINUX)
  {
    UART_Comm_StopReceiveOne(UART_PORT_LINUX);
    (void)UART_Comm_StartReceiveOne(UART_PORT_LINUX, linux_rx_dma_buffer, &linux_rx_dma_pos);
  }
  else if (huart == UART_PORT_BASE)
  {
    UART_Comm_StopReceiveOne(UART_PORT_BASE);
    (void)UART_Comm_StartReceiveOne(UART_PORT_BASE, base_rx_dma_buffer, &base_rx_dma_pos);
  }

  slot = UART_Comm_GetTxSlot(huart);
  if ((slot != NULL) && (huart->gState == HAL_UART_STATE_READY))
  {
    primask = __get_PRIMASK();
    __disable_irq();
    slot->busy = 0U;
    UART_Comm_RestoreIrq(primask);
    UART_Comm_TryStartTx(slot);
  }
}
