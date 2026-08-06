#ifndef TEMP_CONTROL_H
#define TEMP_CONTROL_H

#include "stm32f1xx_hal.h"
#include "cmsis_os.h"

#ifdef __cplusplus
extern "C" {
#endif

/*
 * 恒温控制采用不对称回差：低于目标 0.5℃开启，高于目标 0.3℃关闭，
 * 避免继电器在设定温度附近频繁吸合。
 */
#ifndef TEMP_TARGET_DEFAULT_C
#define TEMP_TARGET_DEFAULT_C       40.0f
#endif

#ifndef TEMP_TARGET_MIN_C
#define TEMP_TARGET_MIN_C           35.0f
#endif

#ifndef TEMP_TARGET_MAX_C
#define TEMP_TARGET_MAX_C           48.0f
#endif

#ifndef TEMP_HYSTERESIS_LOW_C
#define TEMP_HYSTERESIS_LOW_C       0.5f
#endif

#ifndef TEMP_HYSTERESIS_HIGH_C
#define TEMP_HYSTERESIS_HIGH_C      0.3f
#endif

#ifndef TEMP_HIGH_CUTOFF_C
#define TEMP_HIGH_CUTOFF_C          55.0f
#endif

#ifndef TEMP_HEAT_TIMEOUT_MS
/* 连续加热最长时间；超时仍未达到下回差线则锁存故障。 */
#define TEMP_HEAT_TIMEOUT_MS        (15UL * 60UL * 1000UL)
#endif

#ifndef TEMP_PREHEAT_DRAIN_MS
/* 每次真正需要打开加热片前，先排空管道，避免管路无水导致加热空烧。 */
#define TEMP_PREHEAT_DRAIN_MS       5000UL
#endif

void Temp_Init(void);
void Temp_Set(float target);
void Temp_SetTargetC(float target);
float Temp_GetTargetC(void);
float Temp_Read(void);
void Temp_Enable(uint8_t enable);
/** 切换为外部直接控制，供通信业务状态机接管加热输出。 */
void Temp_SetExternalHeatControl(uint8_t enable);
/** 仅在外部控制启用时生效，防止绕过本地恒温状态机。 */
void Temp_SetExternalHeatOutput(uint8_t on);
uint8_t Temp_IsEnabled(void);
uint8_t Temp_IsHeating(void);
uint8_t Temp_HasFault(void);
/** 清除锁存故障；不会自动重新启用加热。 */
void Temp_ClearFault(void);
void Temp_Control_TaskProcess(void);

#ifdef __cplusplus
}
#endif

#endif
