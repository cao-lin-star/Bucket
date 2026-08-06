#include "pump_valve.h"
#include "main.h"
#include "sensor.h"

static PumpValveMode_t pump_mode;       //当前泵/阀模式 关/循环/排水
static uint8_t pump_fault;              //泵故障标志 0=正常 1=故障
static uint32_t pump_mode_start_tick;   //当前模式开始时间，用于排水模式超时检测

//根据模式设置泵和阀的状态
static void PumpValve_Apply(PumpValveMode_t mode)
{
  /* 先确定阀位再控制水泵，减少模式切换时的瞬态水路冲击。 */
  switch (mode)
  {
      //循环模式（加热/冲浪）：水泵开，阀门关（内部水循环）
    case PUMP_VALVE_MODE_CIRCULATION:
      HAL_GPIO_WritePin(EN_TWV_GPIO_Port, EN_TWV_Pin, GPIO_PIN_RESET);
      HAL_GPIO_WritePin(EN_PUMP_GPIO_Port, EN_PUMP_Pin, GPIO_PIN_SET);
      break;

      //排水模式：水泵开，阀门开（将水排出）
    case PUMP_VALVE_MODE_DRAIN:
      HAL_GPIO_WritePin(EN_TWV_GPIO_Port, EN_TWV_Pin, GPIO_PIN_SET);
      HAL_GPIO_WritePin(EN_PUMP_GPIO_Port, EN_PUMP_Pin, GPIO_PIN_SET);
      break;

      //关闭模式：水泵关，阀门关
    default:
      HAL_GPIO_WritePin(EN_PUMP_GPIO_Port, EN_PUMP_Pin, GPIO_PIN_RESET);
      HAL_GPIO_WritePin(EN_TWV_GPIO_Port, EN_TWV_Pin, GPIO_PIN_RESET);
      break;
  }
}

//初始化泵/阀模块
void PumpValve_Init(void)
{
  pump_mode = PUMP_VALVE_MODE_OFF;
  pump_fault = 0U;
  pump_mode_start_tick = HAL_GetTick();
  PumpValve_Apply(PUMP_VALVE_MODE_OFF);
}

//设置泵/阀模式
void PumpValve_SetMode(PumpValveMode_t mode)
{
  /* 故障锁存期间拒绝重新启动，只允许切换到安全的 OFF 模式。 */
  if (mode != pump_mode)
  {
    pump_mode_start_tick = HAL_GetTick();
  }
  pump_mode = mode;
  PumpValve_Apply(mode);
}

//获取当前泵/阀模式
PumpValveMode_t PumpValve_GetMode(void)
{
  return pump_mode;
}

//泵/阀状态处理任务，需定期调用
void PumpValve_TaskProcess(void)
{
  /* 运行态周期检查泵电流；排水模式还需检查最长持续时间。 */
  uint16_t current_ma;

  current_ma = Sensor_GetPumpCurrentMa();

  
  //如果泵正在运行但电流过大，认为泵发生故障，立即关闭泵
  if ((pump_mode != PUMP_VALVE_MODE_OFF) && (current_ma > PUMP_CURRENT_MAX_MA))
  {
    pump_fault = 1U;
    PumpValve_SetMode(PUMP_VALVE_MODE_OFF);
    return;
  }

  //排水超时
  //如果在排水模式下运行时间过长且水位仍然较高，认为泵发生故障，立即关闭泵
  if ((pump_mode == PUMP_VALVE_MODE_DRAIN) &&
      ((HAL_GetTick() - pump_mode_start_tick) > PUMP_VALVE_DRAIN_TIMEOUT_MS) &&
      (Sensor_GetWaterLevelProtocol() > 0U))
  {
    pump_fault = 1U;
    PumpValve_SetMode(PUMP_VALVE_MODE_OFF);
  }
}

//检查泵是否正在运行
uint8_t PumpValve_IsPumpOn(void)
{
  return (pump_mode != PUMP_VALVE_MODE_OFF) ? 1U : 0U;
}

//检查泵是否发生故障
uint8_t PumpValve_HasFault(void)
{
  return pump_fault;
}

//清除泵故障状态
void PumpValve_ClearFault(void)
{
  pump_fault = 0U;
}

//启动泵进行循环
void Pump_Start(void)
{
  PumpValve_SetMode(PUMP_VALVE_MODE_CIRCULATION);
}

//停止泵
void Pump_Stop(void)
{
  PumpValve_SetMode(PUMP_VALVE_MODE_OFF);
}

//设置阀状态 0=循环 1=排水            
void Valve_Set(uint8_t mode)
{
  if (mode != 0U)
  {
    PumpValve_SetMode(PUMP_VALVE_MODE_DRAIN);
  }
  else
  {
    PumpValve_SetMode(PUMP_VALVE_MODE_CIRCULATION);
  }
}
