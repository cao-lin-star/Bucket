#include "rtos_tasks.h"

#include "cmsis_os.h"
#include "log.h"
#include "motor_control.h"
#include "power_manager.h"
#include "pump_valve.h"
#include "sensor.h"
#include "system_monitor.h"
#include "temp_control.h"
#include "uart_comm.h"
#include "uv_lamp.h"

void RtosTasks_Sensor(void *argument)
{
  (void)argument;
  for (;;)
  {
    Sensor_TaskProcess();
    osDelay(20U);
  }
}

void RtosTasks_TempControl(void *argument)
{
  (void)argument;
  for (;;)
  {
    Temp_Control_TaskProcess();
    osDelay(100U);
  }
}

void RtosTasks_Motor(void *argument)
{
  (void)argument;
  for (;;)
  {
    Motor_TaskProcess();
    osDelay(20U);
  }
}

void RtosTasks_PumpValve(void *argument)
{
  (void)argument;
  for (;;)
  {
    PumpValve_TaskProcess();
    osDelay(50U);
  }
}

void RtosTasks_UvLamp(void *argument)
{
  (void)argument;
  for (;;)
  {
    UV_TaskProcess();
    osDelay(500U);
  }
}

void RtosTasks_Communication(void *argument)
{
  (void)argument;
  for (;;)
  {
    UART_Comm_TaskProcess();
    osDelay(50U);
  }
}

void RtosTasks_Logging(void *argument)
{
  (void)argument;
  for (;;)
  {
    Logging_TaskProcess();
    osDelay(1000U);
  }
}

void RtosTasks_SystemMonitor(void *argument)
{
  (void)argument;
  for (;;)
  {
    PowerManager_TaskProcess();
    SystemMonitor_TaskProcess();
    osDelay(50U);
  }
}

