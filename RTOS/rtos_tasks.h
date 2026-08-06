#ifndef RTOS_TASKS_H
#define RTOS_TASKS_H

void RtosTasks_Sensor(void *argument);
void RtosTasks_TempControl(void *argument);
void RtosTasks_Motor(void *argument);
void RtosTasks_PumpValve(void *argument);
void RtosTasks_UvLamp(void *argument);
void RtosTasks_Communication(void *argument);
void RtosTasks_Logging(void *argument);
void RtosTasks_SystemMonitor(void *argument);

#endif /* RTOS_TASKS_H */
