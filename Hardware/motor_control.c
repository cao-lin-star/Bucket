#include "motor_control.h"
#include "sensor.h"
#include "tim.h"

static uint8_t motor_level;           //电机档位：0=停止，1/2/3档
static uint8_t motor_manual_duty;     //手动占空比（0~100%）
static uint8_t motor_running;         //电机运行状态 0=停止 1=运行
static uint8_t motor_fault;           //电机故障标志 0=正常 1=故障
static uint8_t motor_auto_reverse;      //自动反转使能 0=禁用 1=启用
static uint32_t motor_reverse_interval_ms = MOTOR_AUTO_REVERSE_MS;    //自动反转间隔时间（毫秒）
static uint32_t motor_last_reverse_tick;                              //上次反转时间，用于自动反转计时
static MotorDirection_t motor_direction = MOTOR_DIR_FORWARD;          //电机当前转向

//根据档位获取对应的占空比
static uint8_t Motor_LevelToDuty(uint8_t level)
{
  /* 协议档位映射为经实机调校的占空比，而不是简单等比例输出。 */
  switch (level)
  {
    case 1U:
      return 33U;
    case 2U:
      return 66U;
    case 3U:
      return 100U;
    default:
      return 0U;
  }
}

//将占空比转换为定时器比较值
static uint32_t Motor_DutyToPulse(TIM_HandleTypeDef *htim, uint8_t duty_percent)
{
  uint32_t period;
  uint32_t pulse;

  period = htim->Init.Period;
  if (duty_percent > 100U)
  {
    duty_percent = 100U;
  }
  pulse = ((period + 1U) * duty_percent) / 100U;
  if (pulse > period)
  {
    pulse = period;
  }
  return pulse;
}

//设置定时器通道的占空比
static void Motor_SetTimerChannel(TIM_HandleTypeDef *htim, uint32_t channel, uint8_t duty_percent)
{
  __HAL_TIM_SET_COMPARE(htim, channel, Motor_DutyToPulse(htim, duty_percent));
}

static void Motor_ApplyPwm(uint8_t duty_percent, MotorDirection_t direction)
{
  /* H 桥两侧只允许一侧输出 PWM，另一侧保持关闭，避免直通。 */
#if MOTOR_USE_TIM4_PWM
  if (direction == MOTOR_DIR_FORWARD)
  {
    Motor_SetTimerChannel(&htim4, TIM_CHANNEL_1, duty_percent);
    Motor_SetTimerChannel(&htim4, TIM_CHANNEL_2, 0U);
    Motor_SetTimerChannel(&htim4, TIM_CHANNEL_3, duty_percent);
    Motor_SetTimerChannel(&htim4, TIM_CHANNEL_4, 0U);
  }
  else
  {
    Motor_SetTimerChannel(&htim4, TIM_CHANNEL_1, 0U);
    Motor_SetTimerChannel(&htim4, TIM_CHANNEL_2, duty_percent);
    Motor_SetTimerChannel(&htim4, TIM_CHANNEL_3, 0U);
    Motor_SetTimerChannel(&htim4, TIM_CHANNEL_4, duty_percent);
  }
#endif

#if MOTOR_USE_TIM1_PWM
  if (direction == MOTOR_DIR_FORWARD)
  {
    Motor_SetTimerChannel(&htim1, TIM_CHANNEL_1, duty_percent);
    Motor_SetTimerChannel(&htim1, TIM_CHANNEL_4, 0U);
  }
  else
  {
    Motor_SetTimerChannel(&htim1, TIM_CHANNEL_1, 0U);
    Motor_SetTimerChannel(&htim1, TIM_CHANNEL_4, duty_percent);
  }
#endif
}

//电机控制模块初始化
void Motor_Init(void)
{
#if MOTOR_USE_TIM4_PWM
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_4);
#endif

#if MOTOR_USE_TIM1_PWM
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);
#endif

  motor_level = 0U;
  motor_manual_duty = 0U;
  motor_running = 0U;
  motor_fault = 0U;
  motor_auto_reverse = 0U;
  motor_direction = MOTOR_DIR_FORWARD;
  motor_last_reverse_tick = HAL_GetTick();
  Motor_ApplyPwm(0U, motor_direction);
}

//设置电机运行速度和方向
void Motor_Run(uint8_t speed, uint8_t direction)
{
  /* 故障未清除时保持停机；有效启动同时刷新自动换向计时基准。 */
  motor_direction = (direction != 0U) ? MOTOR_DIR_REVERSE : MOTOR_DIR_FORWARD;
  if (speed <= 3U)
  {
    Motor_SetLevel(speed);
  }
  else
  {
    if (speed > 100U)
    {
      speed = 100U;
    }
    motor_level = 1U;
    motor_manual_duty = speed;
    motor_running = (speed != 0U) ? 1U : 0U;
  }
}

//  停止电机
void Motor_Stop(void)
{
  motor_level = 0U;
  motor_manual_duty = 0U;
  motor_running = 0U;
  Motor_ApplyPwm(0U, motor_direction);
}

//设置电机档位（0=停止 1/2/3档）
void Motor_SetLevel(uint8_t level)
{
  if (level > 3U)
  {
    level = 3U;
  }
  motor_level = level;
  motor_manual_duty = 0U;
  motor_running = (level != 0U) ? 1U : 0U;
  if (motor_running == 0U)
  {
    Motor_ApplyPwm(0U, motor_direction);
  }
}

//设置电机转向
void Motor_SetDirection(MotorDirection_t direction)
{
  motor_direction = direction;
  motor_last_reverse_tick = HAL_GetTick();
}

//设置自动反转功能
void Motor_SetAutoReverse(uint8_t enable, uint32_t interval_ms)
{
  motor_auto_reverse = (enable != 0U) ? 1U : 0U;
  if (interval_ms >= 5000UL)
  {
    motor_reverse_interval_ms = interval_ms;
  }
  motor_last_reverse_tick = HAL_GetTick();
}

//获取当前电机档位
uint8_t Motor_GetLevel(void)
{
  return motor_level;
}

//获取当前占空比（0~100%）
uint8_t Motor_GetDutyPercent(void)
{
  if (motor_manual_duty != 0U)
  {
    return motor_manual_duty;
  }
  return Motor_LevelToDuty(motor_level);
}

//获取当前转向
MotorDirection_t Motor_GetDirection(void)
{
  return motor_direction;
}

//检查电机是否正在运行
uint8_t Motor_IsRunning(void)
{
  return motor_running;
}

//检查电机是否发生故障
uint8_t Motor_HasFault(void)
{
  return motor_fault;
}

//清除电机故障状态
void Motor_ClearFault(void)
{
  motor_fault = 0U;
}

//电机控制任务处理函数，负责自动反转和过流保护
void Motor_TaskProcess(void)
{
  /* 先处理过流保护，再处理自动换向，安全条件拥有更高优先级。 */
  uint32_t now;
  uint16_t current_ma;
  uint8_t duty;

  now = HAL_GetTick();
  // ===================== 1. 自动换向逻辑 =====================
  // 如果电机运行中 && 自动反转开启 && 到达换向时间
  if ((motor_running != 0U) &&
      (motor_auto_reverse != 0U) &&
      ((now - motor_last_reverse_tick) >= motor_reverse_interval_ms))
  {
    motor_direction = (motor_direction == MOTOR_DIR_FORWARD) ? MOTOR_DIR_REVERSE : MOTOR_DIR_FORWARD;
    motor_last_reverse_tick = now;
  }

  current_ma = Sensor_GetMotorCurrentMa();
  // ===================== 2. 过流保护 =====================
  // 读取电机电流
  if ((motor_running != 0U) && (current_ma > MOTOR_CURRENT_MAX_MA))
  {
    motor_fault = 1U;
    Motor_Stop();
    return;
  }

  // ===================== 3. 输出PWM控制电机 =====================
  duty = Motor_GetDutyPercent();
  // 根据运行状态输出速度或停止
  if (motor_running != 0U)
  {
    Motor_ApplyPwm(duty, motor_direction);
  }
  else
  {
    Motor_ApplyPwm(0U, motor_direction);
  }
}
