#ifndef LOGGING_H
#define LOGGING_H

#include "stm32f1xx_hal.h"
#include "cmsis_os.h"

#ifdef __cplusplus
extern "C" {
#endif

/*
 * 异步日志模块：业务调用只写入环形缓冲区，由日志任务通过 UART DMA
 * 分段发送，避免 printf 阻塞实时控制任务。
 */
void Logging_Init(void);
void Logging_Print(const char *msg);
/** 受限格式化输出；消息过长时按内部临时缓冲区容量截断。 */
void Logging_Printf(const char *fmt, ...);
void Logging_TaskProcess(void);
void Logging_TxCpltCallback(UART_HandleTypeDef *huart);
void Logging_ErrorCallback(UART_HandleTypeDef *huart);
/** 返回最近一次 HAL 发送状态，供诊断使用。 */
HAL_StatusTypeDef Logging_GetLastStatus(void);

#ifdef __cplusplus
}
#endif

#endif
