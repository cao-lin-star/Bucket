#include "power_manager.h"
#include "main.h"
#include "sensor.h"
#include "uart_comm.h"

static uint8_t power_manager_charging_enabled;  // 充电使能标志 0=禁止 1=允许
static uint8_t power_manager_charge_pending;    // 充电待处理标志 0=无待处理 1=有待处理
static uint32_t power_manager_charge_pending_tick;  // 充电待处理时间戳

// 充电控制：根据基站连接和电池电压决定是否允许充电。
static void PowerManager_SetChargingEnabled(uint8_t enable)
{
  GPIO_PinState state;

  /* DCIN_ON 高电平允许充电，低电平关闭充电。 */
  state = (enable != 0U) ? GPIO_PIN_SET : GPIO_PIN_RESET;
  HAL_GPIO_WritePin(DCIN_ON_GPIO_Port, DCIN_ON_Pin, state);
  power_manager_charging_enabled = (enable != 0U) ? 1U : 0U;
}

// 充电延时标志清零，避免在基站掉线或电池电压升高后立即重新允许充电。
static void PowerManager_ResetChargeDelay(void)
{
  power_manager_charge_pending = 0U;
  power_manager_charge_pending_tick = 0UL;
}

void PowerManager_Init(void)
{
  /* 上电默认禁止充电；仅当两条通信链路在线且低电条件稳定 10 s 后才允许充电。 */
  PowerManager_SetChargingEnabled(0U);
  PowerManager_ResetChargeDelay();
}

void PowerManager_ForceOff(void)
{
  PowerManager_SetChargingEnabled(0U);
  PowerManager_ResetChargeDelay();
}

// 充电控制任务：每周期检查基站连接和电池电压，决定是否允许充电。
void PowerManager_TaskProcess(void)
{
  uint16_t battery_dv;
  uint8_t base_connected;
  uint8_t main_connected;
  uint32_t now;

  battery_dv = Sensor_GetBatteryDeciVolt();           // 电池电压（单位 0.1 V）
  /* BAT_ON/WHEEL_ON: below 20V off, above 20V on, equal holds state. */
  if (battery_dv < 200U)
  {
    HAL_GPIO_WritePin(BAT_ON_GPIO_Port, BAT_ON_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(WHEEL_ON_GPIO_Port, WHEEL_ON_Pin, GPIO_PIN_RESET);
  }
  else if (battery_dv > 200U)
  {
    HAL_GPIO_WritePin(BAT_ON_GPIO_Port, BAT_ON_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(WHEEL_ON_GPIO_Port, WHEEL_ON_Pin, GPIO_PIN_SET);
  }

  base_connected = UART_Comm_IsBaseConnected();       // 基站双向在线状态
  main_connected = UART_Comm_IsMainConnected();       // Linux 主控在线状态

  // 任一通信链路掉线或电压超过停止阈值，立即禁止充电。
  if ((base_connected == 0U) ||
      (main_connected == 0U) ||
      (battery_dv > POWER_MANAGER_BAT_CHARGE_STOP_DV))
  {
    PowerManager_SetChargingEnabled(0U);
    PowerManager_ResetChargeDelay();
    return;
  }

  // 基站在线且电池电压低于启动阈值，延时 10 s 后允许充电。
  if (battery_dv < POWER_MANAGER_BAT_CHARGE_START_DV)
  {
    now = HAL_GetTick();
    if (power_manager_charge_pending == 0U)
    {
      power_manager_charge_pending = 1U;
      power_manager_charge_pending_tick = now;
    }
    else if ((uint32_t)(now - power_manager_charge_pending_tick) >= POWER_MANAGER_CHARGE_START_DELAY_MS)
    {
      PowerManager_SetChargingEnabled(1U);
    }
  }
  else if (power_manager_charging_enabled == 0U)
  {
    PowerManager_ResetChargeDelay();
  }
}

uint8_t PowerManager_IsChargingEnabled(void)
{
  return power_manager_charging_enabled;
}
