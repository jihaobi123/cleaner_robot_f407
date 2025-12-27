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

void Drivetrain_Init(void); /* 鍚姩宸﹀彸杞?PWM 杈撳嚭閫氶亾銆?*/
void Drivetrain_SetRaw(float left_norm, float right_norm); /* 璁剧疆宸﹀彸杞綊涓€鍖栭€熷害 [-1,1]锛岃礋鍊间负鍙嶈浆棰勭暀銆?*/
void Drivetrain_Stop(void); /* 绔嬪嵆鍋滄宸﹀彸杞?PWM 杈撳嚭銆?*/
void Drivetrain_SetDirection(bool left_forward, bool right_forward); /* 璁剧疆宸﹀彸杞鍙嶈浆鏂瑰悜 GPIO銆?*/

float Drivetrain_UnitToNorm(float units); /* Map speed units to norm. */
void Drivetrain_SetUnits(float left_units, float right_units); /* Drive using speed units. */

#ifdef __cplusplus
}
#endif

#endif /* BSP_DRIVETRAIN_H */

