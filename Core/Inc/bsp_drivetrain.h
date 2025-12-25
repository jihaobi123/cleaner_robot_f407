/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    bsp_drivetrain.h
  * @brief   Differential drivetrain control.
  ******************************************************************************
  */
/* USER CODE END Header */
#ifndef BSP_DRIVETRAIN_H
#define BSP_DRIVETRAIN_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>

void Drivetrain_Init(void);
void Drivetrain_SetRaw(float left_norm, float right_norm);
void Drivetrain_Stop(void);

#ifdef __cplusplus
}
#endif

#endif /* BSP_DRIVETRAIN_H */
