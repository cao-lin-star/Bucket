#ifndef UV_LAMP_H
#define UV_LAMP_H

#include "stm32f1xx_hal.h"
#include "cmsis_os.h"

#ifdef __cplusplus
extern "C" {
#endif

/* UV 模块封装杀菌灯 GPIO 输出及运行状态/故障状态查询。 */
void UV_Init(void);
void UV_On(void);
void UV_Off(void);
void UV_Set(uint8_t enable);
uint8_t UV_IsOn(void);
uint8_t UV_HasFault(void);
/** 清除锁存故障，不会自动重新点亮 UV 灯。 */
void UV_ClearFault(void);
void UV_TaskProcess(void);

#ifdef __cplusplus
}
#endif

#endif
