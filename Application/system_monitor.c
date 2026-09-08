#include "system_monitor.h"
#include "motor_control.h"
#include "pump_valve.h"
#include "sensor.h"
#include "temp_control.h"
#include "uv_lamp.h"

//低电压阈值：20.0V（低于则报警）
#ifndef SYSTEM_BAT_LOW_DV 
#define SYSTEM_BAT_LOW_DV            200U
#endif

//高电压阈值：26.0V（高于则报警）
#ifndef SYSTEM_BAT_HIGH_DV
#define SYSTEM_BAT_HIGH_DV           260U
#endif

static uint8_t system_main_status;    /* BucketMainStatus_t 对应的协议主状态。 */
static uint8_t system_sub_status;     /* 主状态下的细分状态，由业务命令定义。 */
static uint8_t system_err1;           /* 传感器、温度和水位类错误位图。 */
static uint8_t system_err2;           /* 执行器和电池类错误位图。 */
static uint8_t system_last_cmd;       /* 最近一次有效业务命令。 */
static uint8_t system_link_status;    /* 主控通信连接状态。 */
static uint8_t system_reset_requested;  /* 延时复位请求标志。 */
static uint32_t system_reset_tick;      /* 延时复位起始时刻。 */
static uint32_t system_timer_deadline_tick;   /* 定时关机绝对截止时刻。 */

//把指令里的时间编码（1~6）转换成对应的定时毫秒（10/15/20/25/30分钟）
static uint32_t SystemMonitor_TimerCodeToMs(uint8_t timer_code)
{
  switch (timer_code)
  {
    case 1U:
      return 10UL * 60UL * 1000UL;
    case 2U:
      return 15UL * 60UL * 1000UL;
    case 3U:
      return 20UL * 60UL * 1000UL;
    case 4U:
      return 25UL * 60UL * 1000UL;
    case 5U:
      return 30UL * 60UL * 1000UL;
    default:
      return 0UL;
  }
}

//系统监控初始化
void SystemMonitor_Init(void)
{
  system_main_status = BUCKET_STATUS_POWER_ON;  //初始状态：开机
  system_sub_status = 0U;                       //子状态清空
  system_err1 = 0U;                             //错误码1清空
  system_err2 = 0U;                             //错误码2清空
  system_last_cmd = 0xA1U;                      //默认指令
  system_link_status = 0U;                      //通信断开
  system_reset_requested = 0U;                  //无复位请求
  system_reset_tick = 0UL;                      //复位时间清空
  system_timer_deadline_tick = 0UL;             // 定时关机时间清空
}

/* 所有全局停机路径必须经由此函数，确保执行器按同一安全顺序关闭。 */
void SystemMonitor_StopAllOutputs(void)
{
  /* 全局停机优先撤销外部加热接管，确保 EN_HEAT 一定被拉低。 */
  Temp_SetExternalHeatControl(0U);
  Temp_Enable(0U);
  Motor_Stop();
  PumpValve_SetMode(PUMP_VALVE_MODE_OFF);
  UV_Off();
}

//设置系统主状态 + 子状态
void SystemMonitor_SetMainStatus(uint8_t main_status, uint8_t sub_status)
{
  system_main_status = main_status;
  system_sub_status = sub_status;
}

//获取状态接口
uint8_t SystemMonitor_GetMainStatus(void)
{
  return system_main_status;
}

//获取状态接口
uint8_t SystemMonitor_GetSubStatus(void)
{
  return system_sub_status;
}

//获取错误码1接口（最高位屏蔽）
uint8_t SystemMonitor_GetErrCode1(void)
{
  return system_err1 & 0x7FU;
}

//获取错误码2接口（最高位屏蔽）
uint8_t SystemMonitor_GetErrCode2(void)
{
  return system_err2 & 0x7FU;
}

//指令存储接口
void SystemMonitor_SetCommand(uint8_t cmd)
{
  system_last_cmd = cmd;
}

//指令存储接口
uint8_t SystemMonitor_GetCommand(void)
{
  return system_last_cmd;
}

//通信连接状态
void SystemMonitor_SetLinkStatus(uint8_t link_status)
{
  system_link_status = (link_status != 0U) ? 1U : 0U;
}

//通信连接状态
uint8_t SystemMonitor_GetLinkStatus(void)
{
  return system_link_status;
}

//设置定时关机
void SystemMonitor_SetBathTimer(uint8_t timer_code)
{
  uint32_t duration_ms;

  duration_ms = SystemMonitor_TimerCodeToMs(timer_code);
  if (duration_ms == 0UL)
  {
    system_timer_deadline_tick = 0UL;
  }
  else
  {
    system_timer_deadline_tick = HAL_GetTick() + duration_ms;
  }
}

//获取剩余时间（分钟）
uint8_t SystemMonitor_GetTimerRemainingMin(void)
{
  uint32_t now;
  uint32_t remaining_ms;

  if (system_timer_deadline_tick == 0UL)
  {
    return 0U;
  }

  now = HAL_GetTick();
  if ((int32_t)(system_timer_deadline_tick - now) <= 0)
  {
    return 0U;
  }

  remaining_ms = system_timer_deadline_tick - now;
  return (uint8_t)(remaining_ms / 60000UL);
}

/* 返回向上取整的剩余秒数，供定时到期判断使用。 */
uint32_t SystemMonitor_GetTimerRemainingSec(void)
{
  uint32_t now;
  uint32_t remaining_ms;

  if (system_timer_deadline_tick == 0UL)
  {
    return 0UL;
  }

  now = HAL_GetTick();
  if ((int32_t)(system_timer_deadline_tick - now) <= 0)
  {
    return 0UL;
  }

  remaining_ms = system_timer_deadline_tick - now;
  return (remaining_ms + 999UL) / 1000UL;
}

void SystemMonitor_RequestReset(void)
{
  system_reset_requested = 1U;
  system_reset_tick = HAL_GetTick();
}

//查询是否请求复位
uint8_t SystemMonitor_IsResetRequested(void)
{
  return system_reset_requested;
}

//清除所有故障
void SystemMonitor_ClearErrors(void)
{
  system_err1 = 0U;
  system_err2 = 0U;
  Temp_ClearFault();
  Motor_ClearFault();
  PumpValve_ClearFault();
  UV_ClearFault();
}

//系统监控核心任务（周期调用）
void SystemMonitor_TaskProcess(void)
{
  uint16_t battery_dv;      //电池电压（分厘伏）

  uint8_t active_needs_water;     //当前活动是否需要水（加热/循环/UV）

  //每次调用先清除之前的错误码，再根据当前传感器状态和执行部件状态重新计算错误码
  system_err1 = 0U;
  system_err2 = 0U;

  //读取当前水位和活动状态


  //判断当前是否有需要水的活动正在运行（加热/循环/UV），如果有但水位过低则报缺水故障
  active_needs_water = ((Temp_IsEnabled() != 0U) ||
                        (PumpValve_GetMode() == PUMP_VALVE_MODE_CIRCULATION) ||
                        (UV_IsOn() != 0U) ||
                        (Motor_IsRunning() != 0U)) ? 1U : 0U;
  // ==================== 错误检测1：传感器与安全保护 ====================
  // 1. 缺水保护：运行中但水位过低 → 报缺水故障
  if ((active_needs_water != 0U) && (Sensor_IsWaterSafe() == 0U))
  {
    system_err1 |= BUCKET_ERR1_LACK_WATER;
  }
  // 2. 水位传感器故障
  if (Sensor_IsWaterSensorOk() == 0U)
  {
    system_err1 |= BUCKET_ERR1_WATER_SENSOR;
  }
  // 3. 温度传感器故障
  if (Sensor_IsTempSensorOk() == 0U)
  {
    system_err1 |= BUCKET_ERR1_TEMP_SENSOR;
  }
  // 4. 温度异常保护：温度过高 → 报温度异常故障
  if (Sensor_GetTemperatureC() >= TEMP_HIGH_CUTOFF_C)
  {
    system_err1 |= BUCKET_ERR1_TEMP_ABNORMAL;
  }
  // 5. 加热超时保护：加热开启超过一定时间且温度未达到合理范围 → 报加热超时故障
  if (Temp_HasFault() != 0U)
  {
    system_err2 |= BUCKET_ERR2_HEAT_MODULE;
  }
  // 6. 水泵故障
  if (PumpValve_HasFault() != 0U)
  {
    system_err2 |= BUCKET_ERR2_PUMP;
  }
  // 7. 电机故障
  if (Motor_HasFault() != 0U)
  {
    system_err2 |= BUCKET_ERR2_MOTOR;
  }
  // 8. UV灯故障
  if (UV_HasFault() != 0U)
  {
    system_err2 |= BUCKET_ERR2_UV;
  }

  
  // 9. 电池电压异常保护：电池电压过低或过高 → 报电池异常故障
  battery_dv = Sensor_GetBatteryDeciVolt();
  // 电压异常（过低 或 过高）
  
  if ((battery_dv != 0U) && ((battery_dv < SYSTEM_BAT_LOW_DV) || (battery_dv > SYSTEM_BAT_HIGH_DV)))
  {
    system_err2 |= BUCKET_ERR2_BATTERY;
    if (battery_dv < SYSTEM_BAT_LOW_DV)
    {
      system_main_status = BUCKET_STATUS_LOW_POWER;
      SystemMonitor_StopAllOutputs();
    }
  }

  // ==================== 定时关机处理 ====================
  // 定时时间到 → 自动停止所有功能，进入待机
  if ((system_timer_deadline_tick != 0UL) && (SystemMonitor_GetTimerRemainingSec() == 0UL))
  {
    system_timer_deadline_tick = 0UL;
    SystemMonitor_StopAllOutputs();
    system_main_status = BUCKET_STATUS_STANDBY;
    system_sub_status = 0U;
  }

  // ==================== 系统复位处理 ====================
  // 收到复位请求 → 延时200ms后重启系统
  if ((system_reset_requested != 0U) && ((HAL_GetTick() - system_reset_tick) > 200UL))
  {
    NVIC_SystemReset();
  }
}
