#include "app_bucket.h"
#include "config_console.h"
#include "device_config.h"

#include "log.h"
#include "motor_control.h"
#include "power_manager.h"
#include "pump_valve.h"
#include "sensor.h"
#include "system_monitor.h"
#include "temp_control.h"
#include "uart_comm.h"
#include "uv_lamp.h"

/*
 * 初始化顺序是业务约束：先让传感器和执行器进入安全初态，再初始化监控、
 * 电源和日志；UART DMA 最后启动，避免接收回调访问尚未初始化的模块状态。
 */
void BucketApp_Init(void)
{
  DeviceConfig_Init();
  Sensor_Init();
  Temp_Init();
  Motor_Init();
  PumpValve_Init();
  UV_Init();
  SystemMonitor_Init();
  PowerManager_Init();
  Logging_Init();
  UART_Comm_Init();
  ConfigConsole_Init();
}

