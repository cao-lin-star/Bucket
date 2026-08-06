#include "temp_control.h"
#include "main.h"
#include "pump_valve.h"
#include "sensor.h"

/* 模块状态由温控任务及命令处理上下文访问；故障采用锁存方式。 */
static float temp_target_c = TEMP_TARGET_DEFAULT_C;
static uint8_t temp_enabled;
static uint8_t temp_heating;
static uint8_t temp_fault;
static uint8_t temp_external_heat_control;
static uint8_t temp_preheat_drain_active;
static uint32_t temp_heat_start_tick;
static uint32_t temp_preheat_drain_start_tick;

static float Temp_ClampTarget(float target)
{
  /* 所有来源的设定值都限制在产品允许的安全范围内。 */
  if (target < TEMP_TARGET_MIN_C)
  {
    return TEMP_TARGET_MIN_C;
  }
  if (target > TEMP_TARGET_MAX_C)
  {
    return TEMP_TARGET_MAX_C;
  }
  return target;
}

static void Temp_SetHeatOutput(uint8_t on)
{
  GPIO_PinState state;

  /* EN_HEAT 高有效，并同步软件状态以供监控和协议上报。 */
  state = (on != 0U) ? GPIO_PIN_SET : GPIO_PIN_RESET;
  HAL_GPIO_WritePin(EN_HEAT_GPIO_Port, EN_HEAT_Pin, state);
  temp_heating = (on != 0U) ? 1U : 0U;
}

void Temp_Init(void)
{
  temp_target_c = TEMP_TARGET_DEFAULT_C;
  temp_enabled = 0U;
  temp_heating = 0U;
  temp_fault = 0U;
  temp_external_heat_control = 0U;
  temp_preheat_drain_active = 0U;
  temp_heat_start_tick = 0U;
  temp_preheat_drain_start_tick = 0U;
  Temp_SetHeatOutput(0U);
}

void Temp_Set(float target)
{
  Temp_SetTargetC(target);
}

void Temp_SetTargetC(float target)
{
  temp_target_c = Temp_ClampTarget(target);
}

float Temp_GetTargetC(void)
{
  return temp_target_c;
}

float Temp_Read(void)
{
  return Sensor_GetTemperatureC();
}

void Temp_Enable(uint8_t enable)
{
  temp_enabled = (enable != 0U) ? 1U : 0U;
  /* 外部接管期间只记录使能意图，不擅自改变物理输出。 */
  if ((temp_enabled == 0U) && (temp_external_heat_control == 0U))
  {
    temp_preheat_drain_active = 0U;
    temp_preheat_drain_start_tick = 0U;
    Temp_SetHeatOutput(0U);
    temp_heat_start_tick = 0U;
  }
}

void Temp_SetExternalHeatControl(uint8_t enable)
{
  temp_external_heat_control = (enable != 0U) ? 1U : 0U;
  /* 退出接管时先关闭，下一周期再由本地控制器判断。 */
  if (temp_external_heat_control == 0U)
  {
    temp_preheat_drain_active = 0U;
    temp_preheat_drain_start_tick = 0U;
    Temp_SetHeatOutput(0U);
    temp_heat_start_tick = 0U;
  }
}

void Temp_SetExternalHeatOutput(uint8_t on)
{
  /* 外部状态机接管后，本地回差及超时逻辑均暂停。 */
  if (temp_external_heat_control != 0U)
  {
    Temp_SetHeatOutput(on);
  }
}

uint8_t Temp_IsEnabled(void)
{
  return temp_enabled;
}

uint8_t Temp_IsHeating(void)
{
  return temp_heating;
}

uint8_t Temp_HasFault(void)
{
  return temp_fault;
}

void Temp_ClearFault(void)
{
  temp_fault = 0U;
}

void Temp_Control_TaskProcess(void)
{
  float current_temp;
  uint32_t now;

  if (temp_external_heat_control != 0U)
  {
    return;
  }

  current_temp = Sensor_GetTemperatureC();
  now = HAL_GetTick();

  if (temp_enabled == 0U)
  {
    temp_preheat_drain_active = 0U;
    temp_preheat_drain_start_tick = 0U;
    Temp_SetHeatOutput(0U);
    return;
  }

  /* 回差区内保持原输出，避免在阈值附近反复启停。 */
  if (current_temp <= (temp_target_c - TEMP_HYSTERESIS_LOW_C))
  {
    if ((temp_heating == 0U) && (temp_preheat_drain_active == 0U))
    {
      /* 真正需要加热前先排空管道 5s，期间保持 EN_HEAT 关闭。 */
      temp_preheat_drain_active = 1U;
      temp_preheat_drain_start_tick = now;
      Temp_SetHeatOutput(0U);
      PumpValve_SetMode(PUMP_VALVE_MODE_DRAIN);
      return;
    }

    if (temp_preheat_drain_active != 0U)
    {
      Temp_SetHeatOutput(0U);
      PumpValve_SetMode(PUMP_VALVE_MODE_DRAIN);
      if ((now - temp_preheat_drain_start_tick) < TEMP_PREHEAT_DRAIN_MS)
      {
        return;
      }
      temp_preheat_drain_active = 0U;
      temp_preheat_drain_start_tick = 0U;
      PumpValve_SetMode(PUMP_VALVE_MODE_CIRCULATION);
    }

    if (temp_heating == 0U)
    {
      temp_heat_start_tick = now;
    }
    Temp_SetHeatOutput(1U);
  }
  else if (current_temp >= (temp_target_c + TEMP_HYSTERESIS_HIGH_C))
  {
    temp_preheat_drain_active = 0U;
    temp_preheat_drain_start_tick = 0U;
    Temp_SetHeatOutput(0U);
    temp_heat_start_tick = 0U;
  }

  /* 连续加热超时且仍未越过下回差线：关闭并锁存故障。 */
  if ((temp_heating != 0U) &&
      (temp_heat_start_tick != 0U) &&
      ((now - temp_heat_start_tick) > TEMP_HEAT_TIMEOUT_MS) &&
      (current_temp < (temp_target_c - TEMP_HYSTERESIS_LOW_C)))
  {
    temp_fault = 1U;
    temp_enabled = 0U;
    Temp_SetHeatOutput(0U);
  }
}
