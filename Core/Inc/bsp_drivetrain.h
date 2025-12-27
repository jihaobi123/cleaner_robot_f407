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

void Drivetrain_Init(void); /* 启动左右轮 PWM 输出通道。 */
void Drivetrain_SetRaw(float left_norm, float right_norm); /* 设置左右轮归一化速度 [-1,1]，负值为反转预留。 */
void Drivetrain_Stop(void); /* 立即停止左右轮 PWM 输出。 */
void Drivetrain_SetDirection(bool left_forward, bool right_forward); /* 设置左右轮正反转方向 GPIO。 */

#ifdef __cplusplus
}
#endif

#endif /* BSP_DRIVETRAIN_H */
