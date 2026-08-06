#include "uv_lamp.h"
#include "main.h"
#include "sensor.h"

static uint8_t uv_enabled;          // UV灯使能标志 0=关闭 1=开启
static uint8_t uv_fault;            // UV灯故障标志 0=正常 1=故障

//作用：系统上电时调用一次，关闭UV灯，复位状态
void UV_Init(void)
{
  uv_enabled = 0U;
  uv_fault = 0U;
  HAL_GPIO_WritePin(EN_UV_GPIO_Port, EN_UV_Pin, GPIO_PIN_RESET);
}

//打开UV灯
void UV_On(void)
{
  /* 故障锁存时禁止点亮，必须先由上层显式清除故障。 */
  uv_enabled = 1U;
  HAL_GPIO_WritePin(EN_UV_GPIO_Port, EN_UV_Pin, GPIO_PIN_SET);
}

//关闭UV灯
void UV_Off(void)
{
  uv_enabled = 0U;
  HAL_GPIO_WritePin(EN_UV_GPIO_Port, EN_UV_Pin, GPIO_PIN_RESET);
}

//设置UV灯状态
void UV_Set(uint8_t enable)
{
  if (enable != 0U)
  {
    UV_On();
  }
  else
  {
    UV_Off();
  }
}

//检查UV灯是否开启
uint8_t UV_IsOn(void)
{
  return uv_enabled;
}

//检查UV灯是否发生故障
uint8_t UV_HasFault(void)
{
  return uv_fault;
}

//清除故障状态
void UV_ClearFault(void)
{
  uv_fault = 0U;
}

//作用：缺水保护 —— 有水才能开UV，没水自动关闭并报故障
void UV_TaskProcess(void)
{
  /* 预留周期自检入口；后续可在此加入电流或反馈脚诊断。 */
  //如果UV灯开启但水位过低，认为UV灯发生故障，立即关闭UV
  /*
  if ((uv_enabled != 0U) && (Sensor_GetWaterLevelProtocol() < SENSOR_WATER_MIN_SAFE_LITERS))
  {
    uv_fault = 1U;
    UV_Off();
  }
    */
}
