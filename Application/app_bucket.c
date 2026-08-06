#include "app_bucket.h"

#include "log.h"
#include "motor_control.h"
#include "power_manager.h"
#include "pump_valve.h"
#include "sensor.h"
#include "system_monitor.h"
#include "temp_control.h"
#include "uart_comm.h"
#include "uv_lamp.h"

void BucketApp_Init(void)
{
  Sensor_Init();
  Temp_Init();
  Motor_Init();
  PumpValve_Init();
  UV_Init();
  SystemMonitor_Init();
  PowerManager_Init();
  Logging_Init();
  UART_Comm_Init();
}

