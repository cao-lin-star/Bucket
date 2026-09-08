#ifndef POWER_MANAGER_H
#define POWER_MANAGER_H

#include "stm32f1xx_hal.h"
#include "cmsis_os.h"
#ifdef __cplusplus
extern "C" {
#endif

#ifndef POWER_MANAGER_BAT_CHARGE_STOP_DV
/* Stop charging when battery is above 25.0V. Unit: 0.1V. */
#define POWER_MANAGER_BAT_CHARGE_STOP_DV      250U
#endif

#ifndef POWER_MANAGER_BAT_CHARGE_START_DV
/* Linux 与基站均在线且电池低于 24.0 V 时才进入充电延时。单位：0.1 V。 */
#define POWER_MANAGER_BAT_CHARGE_START_DV     240U
#endif

#ifndef POWER_MANAGER_CHARGE_START_DELAY_MS
/* 双链路在线且低电条件连续保持 10 s 后才开启充电。 */
#define POWER_MANAGER_CHARGE_START_DELAY_MS   10000UL
#endif

void PowerManager_Init(void);
/** 通信安全路径立即关闭充电并清除启动延时。 */
void PowerManager_ForceOff(void);
void PowerManager_TaskProcess(void);
uint8_t PowerManager_IsChargingEnabled(void);

#ifdef __cplusplus
}
#endif

#endif
