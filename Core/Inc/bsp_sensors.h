/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    bsp_sensors.h
  * @brief   Bumper and cliff sensor handling.
  ******************************************************************************
  */
/* USER CODE END Header */
#ifndef BSP_SENSORS_H
#define BSP_SENSORS_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>

typedef struct
{
  bool bump_left;
  bool bump_right;
  bool cliff_left;
  bool cliff_right;
  bool cliff_front;
} SensorState;

void Sensors_Init(void); /* 传感器初始化（GPIO/EXTI 已由 CubeMX 配置）。 */
void Sensors_Poll(void); /* 轮询将中断事件快照更新到状态。 */
SensorState Sensors_GetState(void); /* 读取当前传感器状态。 */
void Sensors_ClearEvents(void); /* 清除中断触发事件标志。 */

#ifdef __cplusplus
}
#endif

#endif /* BSP_SENSORS_H */
