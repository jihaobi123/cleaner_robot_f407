#include "bsp_drivetrain.h"

#include <math.h>

#include "tim.h"

#define PWM_MAX_MAGNITUDE 1.0f

static float clamp_normalized(float value)
{
  if (value > PWM_MAX_MAGNITUDE)
  {
    return PWM_MAX_MAGNITUDE;
  }
  if (value < -PWM_MAX_MAGNITUDE)
  {
    return -PWM_MAX_MAGNITUDE;
  }
  return value;
}

static uint32_t normalized_to_compare(TIM_HandleTypeDef *htim, float normalized)
{
  float magnitude = fabsf(clamp_normalized(normalized));
  return (uint32_t)(magnitude * (float)htim->Init.Period);
}

void Drivetrain_Init(void)
{
  /* Start PWM channels for left (TIM3 CH2) and right (TIM8 CH1) wheels. */
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_1);

  Drivetrain_Stop();
}

void Drivetrain_SetRaw(float left_norm, float right_norm)
{
  /* TODO: Add GPIO direction control for left wheel. */
  uint32_t left_compare = normalized_to_compare(&htim3, left_norm);
  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, left_compare);

  /* TODO: Add GPIO direction control for right wheel. */
  uint32_t right_compare = normalized_to_compare(&htim8, right_norm);
  __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_1, right_compare);
}

void Drivetrain_Stop(void)
{
  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 0);
  __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_1, 0);
}
