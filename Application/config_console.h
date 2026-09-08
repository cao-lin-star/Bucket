#ifndef CONFIG_CONSOLE_H
#define CONFIG_CONSOLE_H

#include "stm32f1xx_hal.h"

#ifdef __cplusplus
extern "C" {
#endif

/** 初始化日志串口上的文本配置控制台，并启动 ReceiveToIdle 中断接收。 */
void ConfigConsole_Init(void);

/** 任务上下文处理入口：组装命令行、解析命令并执行配置操作。 */
void ConfigConsole_TaskProcess(void);

/** 返回周期状态日志开关；1 表示允许每秒打印状态日志。 */
uint8_t ConfigConsole_IsPeriodicLogEnabled(void);

/** HAL ReceiveToIdle 回调入口；中断中仅搬运字节并立即重启接收。 */
void ConfigConsole_RxEventCallback(UART_HandleTypeDef *huart, uint16_t size);

/** HAL UART 错误回调入口；仅在接收状态就绪时恢复控制台接收。 */
void ConfigConsole_ErrorCallback(UART_HandleTypeDef *huart);

#ifdef __cplusplus
}
#endif

#endif
