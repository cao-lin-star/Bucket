#ifndef SYSTEM_MONITOR_H
#define SYSTEM_MONITOR_H

#include "stm32f1xx_hal.h"
#include "cmsis_os.h"

#ifdef __cplusplus
extern "C" {
#endif

// ==================== 错误码 1 定义（传感器/温度/水位类故障） ====================
// 缺水故障（水位低于安全值）
#define BUCKET_ERR1_LACK_WATER       0x01U
// 排水超时故障（排水时间过长）
#define BUCKET_ERR1_DRAIN_TIMEOUT    0x02U
// 温度异常故障（温度超出合理范围）
#define BUCKET_ERR1_TEMP_ABNORMAL    0x04U
// 加热超时故障（加热很久不升温）
#define BUCKET_ERR1_HEAT_TIMEOUT     0x08U
// 温度传感器故障（短路/开路）
#define BUCKET_ERR1_TEMP_SENSOR      0x10U
// 水位传感器故障
#define BUCKET_ERR1_WATER_SENSOR     0x20U

// ==================== 错误码 2 定义（执行部件/硬件类故障） ====================
// 加热模块故障（加热片/继电器）
#define BUCKET_ERR2_HEAT_MODULE      0x01U
// 水泵故障
#define BUCKET_ERR2_PUMP             0x02U
// 阀门故障
#define BUCKET_ERR2_VALVE            0x04U
// 电机故障
#define BUCKET_ERR2_MOTOR            0x08U
// UV 杀菌灯故障
#define BUCKET_ERR2_UV               0x10U
// 电池异常
#define BUCKET_ERR2_BATTERY          0x20U

// ==================== 设备主状态枚举 ====================
// 定义足浴盆所有可能的运行状态
typedef enum
{
  BUCKET_STATUS_POWER_ON = 0,      // 0：开机上电状态
  BUCKET_STATUS_SELF_CHECK = 1,    // 1：自检状态（开机自动检测）
  BUCKET_STATUS_OFF = 2,           // 2：完全关闭状态
  BUCKET_STATUS_STANDBY = 3,       // 3：待机状态（通电但未工作）
  BUCKET_STATUS_RUNNING = 4,       // 4：运行状态（加热/按摩/UV）
  BUCKET_STATUS_LOW_POWER = 5      // 5：低电量保护状态
} BucketMainStatus_t;

void SystemMonitor_Init(void);
void SystemMonitor_TaskProcess(void);
void SystemMonitor_StopAllOutputs(void);
void SystemMonitor_SetMainStatus(uint8_t main_status, uint8_t sub_status);
uint8_t SystemMonitor_GetMainStatus(void);
uint8_t SystemMonitor_GetSubStatus(void);
uint8_t SystemMonitor_GetErrCode1(void);
uint8_t SystemMonitor_GetErrCode2(void);
void SystemMonitor_SetCommand(uint8_t cmd);
uint8_t SystemMonitor_GetCommand(void);
void SystemMonitor_SetLinkStatus(uint8_t link_status);
uint8_t SystemMonitor_GetLinkStatus(void);
void SystemMonitor_SetBathTimer(uint8_t timer_code);
uint8_t SystemMonitor_GetTimerRemainingMin(void);
uint32_t SystemMonitor_GetTimerRemainingSec(void);
void SystemMonitor_RequestReset(void);
uint8_t SystemMonitor_IsResetRequested(void);
void SystemMonitor_ClearErrors(void);

#ifdef __cplusplus
}
#endif

#endif
