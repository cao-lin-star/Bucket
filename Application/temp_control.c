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
static uint8_t temp_preheat_circulation_active;
static uint32_t temp_heat_start_tick;
static uint32_t temp_preheat_circulation_start_tick;

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

// 设置加热输出
static void Temp_SetHeatOutput(uint8_t on)
{
  GPIO_PinState state;

  /* 最终输出门禁同时覆盖本地恒温和外部接管接口。 */
  if ((on != 0U) && (Sensor_IsWaterSafe() == 0U))
  {
    on = 0U;
    temp_enabled = 0U;
    temp_preheat_circulation_active = 0U;
  }
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
  temp_preheat_circulation_active = 0U;
  temp_heat_start_tick = 0U;
  temp_preheat_circulation_start_tick = 0U;
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
  /* 锁存故障必须由 SystemMonitor_ClearErrors() 显式清除，周期命令不能重启加热。 */
  if ((enable != 0U) && ((temp_fault != 0U) || (Sensor_IsWaterSafe() == 0U)))
  {
    temp_enabled = 0U;
    temp_preheat_circulation_active = 0U;
    temp_preheat_circulation_start_tick = 0U;
    temp_heat_start_tick = 0U;
    Temp_SetHeatOutput(0U);
    return;
  }

  temp_enabled = (enable != 0U) ? 1U : 0U;
  /* 外部接管期间只记录使能意图，不擅自改变物理输出。 */
  if ((temp_enabled == 0U) && (temp_external_heat_control == 0U))
  {
    temp_preheat_circulation_active = 0U;
    temp_preheat_circulation_start_tick = 0U;
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
    temp_preheat_circulation_active = 0U;
    temp_preheat_circulation_start_tick = 0U;
    Temp_SetHeatOutput(0U);
    temp_heat_start_tick = 0U;
  }
}

void Temp_SetExternalHeatOutput(uint8_t on)
{
  /* 外部状态机可接管回差控制，但不得绕过已经锁存的加热故障。 */
  if (temp_external_heat_control != 0U)
  {
    if ((on != 0U) && (temp_fault != 0U))
    {
      Temp_SetHeatOutput(0U);
    }
    else
    {
      Temp_SetHeatOutput(on);
    }
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

  /* 水位保护优先于外部接管；失水关闭加热和内循环，保留排水模式。 */
  if (((temp_enabled != 0U) || (temp_heating != 0U)) &&
      (Sensor_IsWaterSafe() == 0U))
  {
    temp_fault = 1U;
    temp_enabled = 0U;
    temp_preheat_circulation_active = 0U;
    temp_preheat_circulation_start_tick = 0U;
    temp_heat_start_tick = 0U;
    Temp_SetHeatOutput(0U);
    if (PumpValve_GetMode() == PUMP_VALVE_MODE_CIRCULATION)
    {
      PumpValve_SetMode(PUMP_VALVE_MODE_OFF);
    }
    return;
  }

  // 外部接管期间不执行本地回差控制逻辑，避免与上层状态机冲突。
  if (temp_external_heat_control != 0U)
  {
    return;
  }

  current_temp = Sensor_GetTemperatureC();
  now = HAL_GetTick();

  if (temp_enabled == 0U)
  {
    temp_preheat_circulation_active = 0U;
    temp_preheat_circulation_start_tick = 0U;
    Temp_SetHeatOutput(0U);
    return;
  }

  /* 传感器异常、超温或缺水时立即停热并锁存，防止后续周期命令自动恢复。 */
  if ((Sensor_IsTempSensorOk() == 0U) ||
      (current_temp >= TEMP_HIGH_CUTOFF_C) ||
      (Sensor_IsWaterSafe() == 0U))
  {
    temp_fault = 1U;
    temp_enabled = 0U;
    temp_preheat_circulation_active = 0U;
    temp_heat_start_tick = 0U;
    Temp_SetHeatOutput(0U);
    return;
  }

  /* 回差区内保持原输出，避免在阈值附近反复启停。 */
  if (current_temp <= (temp_target_c - TEMP_HYSTERESIS_LOW_C))
  {
    /* 真正需要加热前先内循环 5 秒，让加热管充满水；期间保持 EN_HEAT 关闭。 */
    if ((temp_heating == 0U) && (temp_preheat_circulation_active == 0U))
    {
      temp_preheat_circulation_active = 1U;
      temp_preheat_circulation_start_tick = now;
      Temp_SetHeatOutput(0U);
      PumpValve_SetMode(PUMP_VALVE_MODE_CIRCULATION);
      return;
    }

    /* 预循环期间保持 EN_HEAT 关闭和排水阀关闭；满 5 秒后才允许加热。 */
    if (temp_preheat_circulation_active != 0U)
    {
      Temp_SetHeatOutput(0U);
      PumpValve_SetMode(PUMP_VALVE_MODE_CIRCULATION);
      if ((now - temp_preheat_circulation_start_tick) < TEMP_PREHEAT_CIRCULATION_MS)
      {
        return;
      }
      temp_preheat_circulation_active = 0U;
      temp_preheat_circulation_start_tick = 0U;
    }

    /* 低于下回差线且预循环完成后才允许加热，避免加热管无水空烧。 */
    if (temp_heating == 0U)
    {
      temp_heat_start_tick = now;
    }
    Temp_SetHeatOutput(1U);
  }
  // 高于上回差线时立即停热，避免温度过冲。
  else if (current_temp >= (temp_target_c + TEMP_HYSTERESIS_HIGH_C))
  {
    temp_preheat_circulation_active = 0U;
    temp_preheat_circulation_start_tick = 0U;
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
