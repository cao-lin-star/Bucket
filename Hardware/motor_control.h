#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H

#include "stm32f1xx_hal.h"
#include "cmsis_os.h"

#ifdef __cplusplus
extern "C" {
#endif

// 是否使用 TIM4 产生 PWM：0=不使用
#ifndef MOTOR_USE_TIM4_PWM
#define MOTOR_USE_TIM4_PWM           0U
#endif

// 是否使用 TIM1 产生 PWM：1=使用（硬件实际用的是 TIM1）
#ifndef MOTOR_USE_TIM1_PWM
#define MOTOR_USE_TIM1_PWM           1U
#endif

// ==================== 电机保护/运行参数 ====================
// 电机自动反转间隔：30000ms = 30 分钟
// 防止电机长期单方向运转导致卡死/疲劳
#ifndef MOTOR_AUTO_REVERSE_MS
#define MOTOR_AUTO_REVERSE_MS        30000UL
#endif

// Motor overcurrent threshold: 2000mA, slightly below the 2.2A hardware limit.
// Stop the motor when the measured current exceeds this value.
#ifndef MOTOR_CURRENT_MAX_MA
#define MOTOR_CURRENT_MAX_MA         2000U
#endif

// ==================== 电机方向枚举 ====================
/* 按摩电机支持 PWM 调速、方向切换、自动换向及过流锁存保护。 */
typedef enum
{
  MOTOR_DIR_FORWARD = 0,  // 电机正转（正向按摩）
  MOTOR_DIR_REVERSE = 1   // 电机反转（反向按摩）
} MotorDirection_t;

void Motor_Init(void);
/** 按百分比速度和方向运行；输入会在模块内限制到有效范围。 */
void Motor_Run(uint8_t speed, uint8_t direction);
void Motor_Stop(void);
/** 设置协议档位，模块负责将档位映射为 PWM 占空比。 */
void Motor_SetLevel(uint8_t level);
void Motor_SetDirection(MotorDirection_t direction);
/** 开启周期换向；interval_ms 为 0 时使用默认换向周期。 */
void Motor_SetAutoReverse(uint8_t enable, uint32_t interval_ms);
uint8_t Motor_GetLevel(void);
uint8_t Motor_GetDutyPercent(void);
MotorDirection_t Motor_GetDirection(void);
uint8_t Motor_IsRunning(void);
uint8_t Motor_HasFault(void);
void Motor_ClearFault(void);
void Motor_TaskProcess(void);

#ifdef __cplusplus
}
#endif

#endif
