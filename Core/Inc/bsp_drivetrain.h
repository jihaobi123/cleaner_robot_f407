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

void Drivetrain_Init(void); /* Start left/right PWM outputs. */
void Drivetrain_SetRaw(float left_norm, float right_norm); /* Set normalized speed [-1,1]. */
void Drivetrain_Stop(void); /* Stop left/right PWM outputs. */
void Drivetrain_SetDirection(bool left_forward, bool right_forward); /* Set direction GPIO. */

float Drivetrain_UnitToNorm(float units); /* Map speed units to norm. */
void Drivetrain_SetUnits(float left_units, float right_units); /* Drive using speed units. */
void Drivetrain_SetTwist(float v_linear, float v_angular); /* Twist to wheel units mapping. */

#ifdef __cplusplus
}
#endif

#endif /* BSP_DRIVETRAIN_H */


