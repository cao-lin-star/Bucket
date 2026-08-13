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

/* 桶体状态主动上报周期，单位 ms。 */
#ifndef UART_COMM_STATUS_PERIOD_MS
#define UART_COMM_STATUS_PERIOD_MS       1000UL
#endif

/* 自检保持时间；期间状态帧 data[12]（主状态）保持为 0x01。 */
#ifndef UART_COMM_SELF_CHECK_DURATION_MS
#define UART_COMM_SELF_CHECK_DURATION_MS 5000UL
#endif

/* 主控命令超时：超时未收到合法帧时，桶体进入安全停机。 */
#ifndef UART_COMM_MAIN_TIMEOUT_MS
#define UART_COMM_MAIN_TIMEOUT_MS        5000UL
#endif

/* 基站链路超时：用于计算状态帧中的 LINK_STATUS。 */
#ifndef UART_COMM_BASE_TIMEOUT_MS
#define UART_COMM_BASE_TIMEOUT_MS        3000UL
#endif

#ifndef UART_COMM_BASE_DCIN_CONNECTED_DV
/* AD_DCIN 高于 13.0 V 才认为桶体已接入基站电源，单位 0.1 V。 */
#define UART_COMM_BASE_DCIN_CONNECTED_DV 130U
#endif
/* 两个协议串口统一强制为 115200 baud。 */
#ifndef UART_COMM_FORCE_PROTOCOL_BAUD
#define UART_COMM_FORCE_PROTOCOL_BAUD    1U
#endif

/* 调试命令开关：1 允许 B4 直接开启水泵和排水通路，0 禁止。 */
#ifndef UART_COMM_ENABLE_DEBUG_B4_PUMP_VALVE
#define UART_COMM_ENABLE_DEBUG_B4_PUMP_VALVE  1U
#endif

/* 单路 UART 循环 DMA 接收缓冲区长度。 */
#ifndef UART_COMM_RX_DMA_BUFFER_LEN
#define UART_COMM_RX_DMA_BUFFER_LEN      128U
#endif

#define UART_CMD_IDLE                    0x00U      /* 空闲/心跳 */
#define UART_CMD_BUCKET_OFF              0xA0U      /* 关闭桶体 */
#define UART_CMD_BUCKET_STANDBY          0xA1U      /* 进入待机 */
#define UART_CMD_BUCKET_TEMP_ON          0xA2U      /* 开启恒温 */
#define UART_CMD_BUCKET_TEMP_OFF         0xA3U      /* 关闭恒温 */
#define UART_CMD_BUCKET_MOTOR            0xA4U      /* 设置按摩档位 */
#define UART_CMD_BUCKET_UV               0xA5U      /* 设置 UV 灯 */
#define UART_CMD_BUCKET_TIMER            0xA6U      /* 设置定时 */
#define UART_CMD_BUCKET_STOP             0xA7U      /* 停止全部功能 */
#define UART_CMD_BUCKET_SELF_CHECK       0xA8U      /* 启动自检 */
#define UART_CMD_BUCKET_LOW_POWER        0xA9U      /* 进入低功耗 */
#define UART_CMD_BUCKET_AUTO_FILL        0xB2U      /* 自动补水：基站执行加/排水，桶体反馈水位和温度 */
#define UART_CMD_DEBUG_PUMP_VALVE_ON     0xB4U      /* 调试：水泵开启并切到排水通路 */
#define UART_CMD_SYSTEM_RESET            0xC0U      /* 系统复位 */
/* 仅以下基站状态码会接管桶体泵阀模式。 */
#define UART_BASE_STATUS_CLEAN_DRAIN1    0x08U
#define UART_BASE_STATUS_CLEAR_DRAIN2    0x0AU
#define UART_BASE_STATUS_FORCE_DRAIN      0x0CU
#define UART_BASE_STATUS_CLEAN_SPRAY     0x07U
#define UART_BASE_STATUS_CLEAR_SPRAY     0x09U

/* 字节流解析器：保存正在组装的固定长度协议帧。 */
typedef struct
{
  uint8_t buffer[UART_COMM_FRAME_LEN];
  uint8_t index;
} UartParser_t;

/* 单帧接收槽：中断写入完整帧，任务原子取走 ready 帧。 */
typedef struct
{
  uint8_t frame[UART_COMM_FRAME_LEN];
  volatile uint8_t ready;
} UartFrameSlot_t;

/* DMA 发送槽：保留正在发送的帧，以及一帧“最新待发送”数据。 */
typedef struct
{
  UART_HandleTypeDef *huart;
  uint8_t active_frame[UART_COMM_FRAME_LEN];
  uint8_t pending_frame[UART_COMM_FRAME_LEN];
  volatile uint8_t busy;
  volatile uint8_t pending;
} UartTxSlot_t;

/* Linux 主控和基站各自维护解析器与接收槽，避免串口状态互相污染。 */
static UartParser_t linux_parser;
static UartParser_t base_parser;
static UartFrameSlot_t linux_rx_slot;
static UartFrameSlot_t base_rx_slot;

/* 两路 UART 的循环 DMA 接收缓冲区及上次消费位置。 */
static uint8_t linux_rx_dma_buffer[UART_COMM_RX_DMA_BUFFER_LEN];
static uint8_t base_rx_dma_buffer[UART_COMM_RX_DMA_BUFFER_LEN];
static uint16_t linux_rx_dma_pos;
static uint16_t base_rx_dma_pos;

/* 两路 UART 独立发送状态。 */
static UartTxSlot_t linux_tx_slot;
static UartTxSlot_t base_tx_slot;

/* 状态上报、链路看门狗和自检窗口所需的跨周期状态。 */
static uint8_t base_data[13];
static uint32_t base_last_rx_tick;
static uint32_t main_last_rx_tick;
static uint32_t last_status_tx_tick;
static uint8_t last_link_mode = UART_COMM_DEFAULT_LINK_MODE;
static uint8_t main_timeout_handled;
static uint8_t self_check_pending;
static uint32_t self_check_start_tick;

/* 临界区退出时只恢复调用前已开启的中断状态。 */
static void UART_Comm_RestoreIrq(uint32_t primask)
{
  if (primask == 0U)
  {
    __enable_irq();
  }
}
/* 链路模式决定命令由桶体执行、向基站透传，还是两者兼有。 */
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
/* 只有通过格式校验的主控帧才能刷新通信看门狗。 */
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
/* 主控掉线保护只执行一次；收到下一帧后才重新装载看门狗。 */
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

  SystemMonitor_StopAllOutputs();   /* 主控通信超时后立即关闭全部输出。 */
  SystemMonitor_SetBathTimer(0U);   /* 同时取消定时，防止恢复通信后误触发。 */
  SystemMonitor_SetCommand(UART_CMD_BUCKET_STOP);
  SystemMonitor_SetMainStatus(BUCKET_STATUS_STANDBY, 0U);
  main_timeout_handled = 1U;
}

/* 校验和为 data[2..28] 累加结果的低 8 位。 */
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

/* 协议帧必须同时通过帧头和校验和检查。 */
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

/* 保存最新完整帧；处理不及时期间允许新帧覆盖旧帧。 */
static void UART_Comm_StoreFrame(UartFrameSlot_t *slot, const uint8_t *frame)
{
  if ((slot == NULL) || (frame == NULL))
  {
    return;
  }
  memcpy(slot->frame, frame, UART_COMM_FRAME_LEN);
  slot->ready = 1U;
}

/* 从连续字节流中寻找 55 AA 帧头，并组装固定 30 字节帧。 */
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

/* 在关中断临界区内取走完整帧，避免回调写入时读到半帧。 */
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

/* 根据 UART 句柄选择对应的独立发送槽。 */
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

/* 启动单路循环 DMA 接收；关闭半传输中断以减少无效中断。 */
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

/* 停止单路 DMA 接收，并把 HAL 接收状态恢复为可重新启动。 */
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

/* 清空缓冲区后，同时启动 Linux 主控和基站两路 DMA 接收。 */
static void UART_Comm_StartReceive(void)
{
  memset(linux_rx_dma_buffer, 0, sizeof(linux_rx_dma_buffer));
  memset(base_rx_dma_buffer, 0, sizeof(base_rx_dma_buffer));
  (void)UART_Comm_StartReceiveOne(UART_PORT_LINUX, linux_rx_dma_buffer, &linux_rx_dma_pos);
  (void)UART_Comm_StartReceiveOne(UART_PORT_BASE, base_rx_dma_buffer, &base_rx_dma_pos);
}

/* 把 DMA 缓冲区指定区间逐字节送入协议解析器。 */
static void UART_Comm_ProcessRxRange(UartParser_t *parser, UartFrameSlot_t *slot,
                                     const uint8_t *buffer, uint16_t start, uint16_t end)
{
  uint16_t index;

  for (index = start; index < end; index++)
  {
    UART_Comm_ParserPush(parser, slot, buffer[index]);
  }
}

/* 消费循环 DMA 新数据；写指针回绕时分成尾部和头部两段处理。 */
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

/* 发送器空闲且存在待发帧时，将待发帧切换为活动帧并启动 DMA。 */
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

/* 提交一帧发送；DMA 忙时仅保留最新待发帧，避免状态帧排队过期。 */
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
/* 温度传感器有效时四舍五入为协议整数；无效时上报 0。 */
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

/* 向基站透传主控帧；心跳帧需补入桶体实时水位和温度。 */
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

/* B2 自动补水由基站执行水路动作，桶体停止加热并进入运行态。 */
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
/* A0~A9 为桶体本地业务命令，集中在此更新执行器和系统状态。 */
static void UART_Comm_ProcessBucketCommand(const uint8_t *frame)
{
  uint8_t cmd;

  cmd = frame[3];

  /* 自检期间忽略其他桶体命令，确保输出保持关闭且状态不被覆盖。 */
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
      /* 加热故障锁存时 Temp_Enable 会拒绝启动，此时不得单独开启循环泵。 */
      if (Temp_IsEnabled() != 0U)
      {
        PumpValve_SetMode(PUMP_VALVE_MODE_CIRCULATION);
      }
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
      SystemMonitor_SetMainStatus((Motor_GetLevel() == 0U) ? BUCKET_STATUS_STANDBY : BUCKET_STATUS_RUNNING, frame[9]);
      break;

    case UART_CMD_BUCKET_UV:
      UV_Set(frame[8]);
 
      /* UV 故障锁存可能拒绝开启，泵阀和主状态必须依据实际输出而非请求值。 */
      if (UV_IsOn() != 0U)
      {
        PumpValve_SetMode(PUMP_VALVE_MODE_CIRCULATION);
      }
      else if (Temp_IsEnabled() == 0U)
      {
        PumpValve_SetMode(PUMP_VALVE_MODE_OFF);
      }
      SystemMonitor_SetMainStatus((UV_IsOn() == 0U) ? BUCKET_STATUS_STANDBY : BUCKET_STATUS_RUNNING, frame[9]);
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

/* 处理 C0~CF 系统命令；复位前先关闭全部输出。 */
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
/* B4 调试命令会直接开启水泵并切到排水通路。 */
static void UART_Comm_ProcessDebugPumpValveCommand(void)
{
  PumpValve_SetMode(PUMP_VALVE_MODE_DRAIN);
  SystemMonitor_SetCommand(UART_CMD_DEBUG_PUMP_VALVE_ON);
  SystemMonitor_SetMainStatus(BUCKET_STATUS_RUNNING, 0U);
}
#endif

/* 解析一帧 Linux 主控数据；返回值表示该帧是否属于有效业务帧。 */
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
  /* 自检期间仍用合法帧维持链路，但不允许其他命令覆盖自检状态。 */
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
  /* 透传模式先发给基站；B0~BF 命令归基站处理，桶体不重复执行。 */
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
/* 自检采用定时状态窗口；到期后回到 POWER_ON 并立即上报。 */
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

/* 将基站清洗/排水状态映射为桶体泵阀模式。 */
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
/* 基站退出水路动作后，仅在恒温和 UV 都不需要循环时关闭水泵。 */
static void UART_Comm_SyncPumpValveFromBaseStatus(uint8_t status,
                                                  uint8_t circulation_requested)
{
  if (UART_Comm_IsBaseDrainStatus(status) != 0U)
  {
    PumpValve_SetMode(PUMP_VALVE_MODE_DRAIN);
  }
  else if ((circulation_requested != 0U) ||
           (UART_Comm_IsBaseCirculationStatus(status) != 0U))
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
/* 只有透传模式的合法基站帧才更新基站数据和在线时间。 */
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
  UART_Comm_SyncPumpValveFromBaseStatus(frame[23], frame[28]);
  now = HAL_GetTick();
  base_last_rx_tick = now;
  return 1U;
}
/* 组装状态快照；基站离线时不带入上次缓存的基站数据。 */
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

/* 基站在线需要同时满足 DCIN 接入且近期收到合法基站帧。 */
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

/* 外部同步解析入口：仅接受完整且校验正确的 30 字节帧。 */
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

/* 初始化通信状态，统一协议波特率，并启动两路循环 DMA 接收。 */
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
  /* UART 重新初始化完成后再启动 DMA，确保端口使用固定协议波特率。 */
#if UART_COMM_FORCE_PROTOCOL_BAUD
  UART_PORT_LINUX->Init.BaudRate = UART_PORT_PROTOCOL_BAUD;
  UART_PORT_BASE->Init.BaudRate = UART_PORT_PROTOCOL_BAUD;
  HAL_UART_Init(UART_PORT_LINUX);
  HAL_UART_Init(UART_PORT_BASE);
#endif

  UART_Comm_StartReceive();
}
/* 通信任务：处理最新收包、掉线保护、自检完成和周期状态上报。 */
void UART_Comm_TaskProcess(void)
{
  uint8_t frame[UART_COMM_FRAME_LEN];
  uint32_t now;
  /* 每周期最多取主控和基站各一帧；接收槽始终保留最新完整帧。 */
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
  /* 先执行安全/状态转换，再决定是否立即或按周期发送状态帧。 */
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

/* ReceiveToIdle 回调：仅消费 DMA 新增区间，不在中断中执行业务命令。 */
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

/* 发送完成后释放活动帧，并立即尝试发送最新待发帧。 */
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

/* UART/DMA 出错后重启对应接收通道，并恢复可继续发送的状态。 */
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
