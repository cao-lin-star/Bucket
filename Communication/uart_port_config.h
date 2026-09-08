#ifndef UART_PORT_CONFIG_H
#define UART_PORT_CONFIG_H

#include "usart.h"

// 0: Linux=USART1, Log=USART3
// 1: Linux=USART3, Log=USART1
#ifndef UART_PORT_DEBUG_SWAP_LOG_LINUX
#define UART_PORT_DEBUG_SWAP_LOG_LINUX  0U
#endif

#ifndef UART_PORT_PROTOCOL_BAUD
#define UART_PORT_PROTOCOL_BAUD         115200U
#endif

/* 桶体与基站的独立通信波特率；两端必须保持一致。 */
#ifndef UART_PORT_BASE_BAUD
#define UART_PORT_BASE_BAUD             9600U
#endif

#ifndef UART_PORT_LOGGING_BAUD
#define UART_PORT_LOGGING_BAUD          115200U
#endif

#if (UART_PORT_DEBUG_SWAP_LOG_LINUX != 0U)
#define UART_PORT_LINUX                 (&huart3)
#define UART_PORT_LOGGING               (&huart1)
#else
#define UART_PORT_LINUX                 (&huart1)
#define UART_PORT_LOGGING               (&huart3)
#endif

#define UART_PORT_BASE                  (&huart2)

#endif
