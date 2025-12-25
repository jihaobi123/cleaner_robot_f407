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

void Sensors_Init(void);
void Sensors_Poll(void);
SensorState Sensors_GetState(void);
void Sensors_ClearEvents(void);

#ifdef __cplusplus
}
#endif

#endif /* BSP_SENSORS_H */
