#include "power_manager.h"
#include "main.h"
#include "sensor.h"
#include "uart_comm.h"

static uint8_t power_manager_charging_enabled;
static uint8_t power_manager_charge_pending;
static uint32_t power_manager_charge_pending_tick;

static void PowerManager_SetChargingEnabled(uint8_t enable)
{
  GPIO_PinState state;

  /* DCIN_ON low disables charging, high enables charging. */
  state = (enable != 0U) ? GPIO_PIN_SET : GPIO_PIN_RESET;
  HAL_GPIO_WritePin(DCIN_ON_GPIO_Port, DCIN_ON_Pin, state);
  power_manager_charging_enabled = (enable != 0U) ? 1U : 0U;
}

static void PowerManager_ResetChargeDelay(void)
{
  power_manager_charge_pending = 0U;
  power_manager_charge_pending_tick = 0UL;
}

void PowerManager_Init(void)
{
  /* Default to charging disabled; enable only after base/low-battery condition is stable for 10s. */
  PowerManager_SetChargingEnabled(0U);
  PowerManager_ResetChargeDelay();
}

void PowerManager_TaskProcess(void)
{
  uint16_t battery_dv;
  uint8_t base_connected;
  uint32_t now;

  battery_dv = Sensor_GetBatteryDeciVolt();
  base_connected = UART_Comm_IsBaseConnected();

  if ((base_connected == 0U) ||
      (battery_dv > POWER_MANAGER_BAT_CHARGE_STOP_DV))
  {
    PowerManager_SetChargingEnabled(0U);
    PowerManager_ResetChargeDelay();
    return;
  }

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
