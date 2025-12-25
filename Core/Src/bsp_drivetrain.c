/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    bsp_drivetrain.c
  * @brief   Differential drivetrain control.
  ******************************************************************************
  */
/* USER CODE END Header */

#include "bsp_drivetrain.h"
#include "tim.h"

#define DRIVETRAIN_MAX_NORM  1.0f
#define DRIVETRAIN_MIN_NORM -1.0f

static float clamp_norm(float v)
{
  if (v > DRIVETRAIN_MAX_NORM)
  {
    return DRIVETRAIN_MAX_NORM;
  }
  if (v < DRIVETRAIN_MIN_NORM)
  {
    return DRIVETRAIN_MIN_NORM;
  }
  return v;
}

static uint32_t norm_to_ccr(TIM_HandleTypeDef *htim, float norm)
{
  float mag = (norm < 0.0f) ? -norm : norm;
  uint32_t arr = __HAL_TIM_GET_AUTORELOAD(htim);
  if (mag > 1.0f)
  {
    mag = 1.0f;
  }
  return (uint32_t)((float)arr * mag + 0.5f);
}

void Drivetrain_Init(void)
{
  (void)HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_1);
  (void)HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
}

void Drivetrain_SetRaw(float left_norm, float right_norm)
{
  float left = clamp_norm(left_norm);
  float right = clamp_norm(right_norm);

  /* TODO: set direction GPIOs when hardware is available. */
  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, norm_to_ccr(&htim3, left));
  __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_1, norm_to_ccr(&htim8, right));
}

void Drivetrain_Stop(void)
{
  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 0);
  __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_1, 0);
}
