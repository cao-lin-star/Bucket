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
/* Start charging only when battery is below 24.0V and base is connected. Unit: 0.1V. */
#define POWER_MANAGER_BAT_CHARGE_START_DV     240U
#endif

#ifndef POWER_MANAGER_CHARGE_START_DELAY_MS
/* Base-connected and low-battery condition must remain valid for 10s before charging starts. */
#define POWER_MANAGER_CHARGE_START_DELAY_MS   10000UL
#endif

void PowerManager_Init(void);
void PowerManager_TaskProcess(void);
uint8_t PowerManager_IsChargingEnabled(void);

#ifdef __cplusplus
}
#endif

#endif
