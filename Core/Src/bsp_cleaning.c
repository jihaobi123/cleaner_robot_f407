/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    bsp_cleaning.c
  * @brief   Cleaning subsystem control.
  ******************************************************************************
  */
/* USER CODE END Header */

#include "bsp_cleaning.h"
#include "gpio.h"
#include "tim.h"

#define CLEANING_PWM_ON_PERCENT  0.80f

static uint32_t percent_to_ccr(TIM_HandleTypeDef *htim, float percent)
{
  uint32_t arr = __HAL_TIM_GET_AUTORELOAD(htim);
  if (percent < 0.0f)
  {
    percent = 0.0f;
  }
  if (percent > 1.0f)
  {
    percent = 1.0f;
  }
  return (uint32_t)((float)arr * percent + 0.5f);
}

void Cleaning_Init(void)
{
  (void)HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
  (void)HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
  Cleaning_SetPump(false);
  Cleaning_SetBrush(false);
  Cleaning_SetFan(false);
}

void Cleaning_SetBrush(bool on)
{
  uint32_t ccr = on ? percent_to_ccr(&htim4, CLEANING_PWM_ON_PERCENT) : 0U;
  __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, ccr);
}

void Cleaning_SetPump(bool on)
{
  uint32_t ccr = on ? percent_to_ccr(&htim3, CLEANING_PWM_ON_PERCENT) : 0U;
  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, ccr);
}

void Cleaning_SetFan(bool on)
{
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, on ? GPIO_PIN_SET : GPIO_PIN_RESET);
}
