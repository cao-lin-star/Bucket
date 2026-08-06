/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "app_bucket.h"
#include "rtos_tasks.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

/* USER CODE END Variables */
/* Definitions for Task1 */
osThreadId_t Task1Handle;
const osThreadAttr_t Task1_attributes = {
  .name = "Task1",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityHigh,
};
/* Definitions for Task2 */
osThreadId_t Task2Handle;
const osThreadAttr_t Task2_attributes = {
  .name = "Task2",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityHigh,
};
/* Definitions for Task3 */
osThreadId_t Task3Handle;
const osThreadAttr_t Task3_attributes = {
  .name = "Task3",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for Task4 */
osThreadId_t Task4Handle;
const osThreadAttr_t Task4_attributes = {
  .name = "Task4",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for Task5 */
osThreadId_t Task5Handle;
const osThreadAttr_t Task5_attributes = {
  .name = "Task5",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for Task6 */
osThreadId_t Task6Handle;
const osThreadAttr_t Task6_attributes = {
  .name = "Task6",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for Task7 */
osThreadId_t Task7Handle;
const osThreadAttr_t Task7_attributes = {
  .name = "Task7",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for Task8 */
osThreadId_t Task8Handle;
const osThreadAttr_t Task8_attributes = {
  .name = "Task8",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityHigh,
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void Sensor_Task(void *argument);
void Temp_Control_Task(void *argument);
void Motor_Task(void *argument);
void Pump_Valve_Task(void *argument);
void UV_Lamp_Task(void *argument);
void Communication_Task(void *argument);
void Logging_Task(void *argument);
void System_Monitor_Task(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */
  BucketApp_Init();

  //水泵运放电路调试
  //HAL_GPIO_WritePin(EN_PUMP_GPIO_Port, EN_PUMP_Pin, GPIO_PIN_SET);
  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of Task1 */
  Task1Handle = osThreadNew(Sensor_Task, NULL, &Task1_attributes);

  /* creation of Task2 */
  Task2Handle = osThreadNew(Temp_Control_Task, NULL, &Task2_attributes);

  /* creation of Task3 */
  Task3Handle = osThreadNew(Motor_Task, NULL, &Task3_attributes);

  /* creation of Task4 */
  Task4Handle = osThreadNew(Pump_Valve_Task, NULL, &Task4_attributes);

  /* creation of Task5 */
  Task5Handle = osThreadNew(UV_Lamp_Task, NULL, &Task5_attributes);

  /* creation of Task6 */
  Task6Handle = osThreadNew(Communication_Task, NULL, &Task6_attributes);

  /* creation of Task7 */
  Task7Handle = osThreadNew(Logging_Task, NULL, &Task7_attributes);

  /* creation of Task8 */
  Task8Handle = osThreadNew(System_Monitor_Task, NULL, &Task8_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_Sensor_Task */
/**
  * @brief  Function implementing the Task1 thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_Sensor_Task */
void Sensor_Task(void *argument)
{
  /* USER CODE BEGIN Sensor_Task */
  RtosTasks_Sensor(argument);
  /* USER CODE END Sensor_Task */
}

/* USER CODE BEGIN Header_Temp_Control_Task */
/**
* @brief Function implementing the Task2 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Temp_Control_Task */
void Temp_Control_Task(void *argument)
{
  /* USER CODE BEGIN Temp_Control_Task */
  RtosTasks_TempControl(argument);
  /* USER CODE END Temp_Control_Task */
}

/* USER CODE BEGIN Header_Motor_Task */
/**
* @brief Function implementing the Task3 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Motor_Task */
void Motor_Task(void *argument)
{
  /* USER CODE BEGIN Motor_Task */
  RtosTasks_Motor(argument);
  /* USER CODE END Motor_Task */
}

/* USER CODE BEGIN Header_Pump_Valve_Task */
/**
* @brief Function implementing the Task4 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Pump_Valve_Task */
void Pump_Valve_Task(void *argument)
{
  /* USER CODE BEGIN Pump_Valve_Task */
  RtosTasks_PumpValve(argument);
  /* USER CODE END Pump_Valve_Task */
}

/* USER CODE BEGIN Header_UV_Lamp_Task */
/**
* @brief Function implementing the Task5 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_UV_Lamp_Task */
void UV_Lamp_Task(void *argument)
{
  /* USER CODE BEGIN UV_Lamp_Task */
  RtosTasks_UvLamp(argument);
  /* USER CODE END UV_Lamp_Task */
}

/* USER CODE BEGIN Header_Communication_Task */
/**
* @brief Function implementing the Task6 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Communication_Task */
void Communication_Task(void *argument)
{
  /* USER CODE BEGIN Communication_Task */
  RtosTasks_Communication(argument);
  /* USER CODE END Communication_Task */
}

/* USER CODE BEGIN Header_Logging_Task */
/**
* @brief Function implementing the Task7 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Logging_Task */
void Logging_Task(void *argument)
{
  /* USER CODE BEGIN Logging_Task */
  RtosTasks_Logging(argument);
  /* USER CODE END Logging_Task */
}

/* USER CODE BEGIN Header_System_Monitor_Task */
/**
* @brief Function implementing the Task8 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_System_Monitor_Task */
void System_Monitor_Task(void *argument)
{
  /* USER CODE BEGIN System_Monitor_Task */
  RtosTasks_SystemMonitor(argument);
  /* USER CODE END System_Monitor_Task */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

