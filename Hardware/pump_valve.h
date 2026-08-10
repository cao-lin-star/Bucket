#ifndef PUMP_VALVE_H
#define PUMP_VALVE_H

#include "stm32f1xx_hal.h"
#include "cmsis_os.h"

#ifdef __cplusplus
extern "C" {
#endif

//水泵最大工作电流
#ifndef PUMP_CURRENT_MAX_MA
#define PUMP_CURRENT_MAX_MA          3000U
#endif

//排水模式超时时间：8分钟
#ifndef PUMP_VALVE_DRAIN_TIMEOUT_MS
#define PUMP_VALVE_DRAIN_TIMEOUT_MS  (8UL * 60UL * 1000UL)
#endif

/*
 * 水泵/电磁阀组合模式：循环时关闭排水阀并启动水泵；排水时打开
 * 排水通路并启动水泵。任务函数负责过流与排水超时保护。
 */
typedef enum
{
  PUMP_VALVE_MODE_OFF = 0,                // 关闭
  PUMP_VALVE_MODE_CIRCULATION = 1,        // 循环
  PUMP_VALVE_MODE_DRAIN = 2             // 排水
} PumpValveMode_t;

void PumpValve_Init(void);
/** 原子切换水路模式；非法枚举值按关闭处理。 */
void PumpValve_SetMode(PumpValveMode_t mode);
PumpValveMode_t PumpValve_GetMode(void);
void PumpValve_TaskProcess(void);
uint8_t PumpValve_IsPumpOn(void);
uint8_t PumpValve_HasFault(void);
/** 清除锁存故障，但不会恢复故障前的运行模式。 */
void PumpValve_ClearFault(void);

/* 以下接口为兼容旧业务代码保留，新代码优先使用 SetMode。 */
void Pump_Start(void);
void Pump_Stop(void);
void Valve_Set(uint8_t mode);

#ifdef __cplusplus
}
#endif

#endif
