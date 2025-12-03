#include "bsp_cleaning.h"

#include "gpio.h"
#include "tim.h"

#define CLEANING_ON_DUTY_PERCENT 0.8f

static uint32_t duty_to_compare(TIM_HandleTypeDef *htim, float duty_percent)
{
  if (duty_percent < 0.0f)
  {
    duty_percent = 0.0f;
  }
  if (duty_percent > 1.0f)
  {
    duty_percent = 1.0f;
  }
  return (uint32_t)(duty_percent * (float)htim->Init.Period);
}

void Cleaning_Init(void)
{
  /* Start PWM for pump (TIM3 CH1) and brush motor (TIM4 CH1). */
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);

  /* Default everything off. */
  Cleaning_SetPump(false);
  Cleaning_SetBrush(false);
  Cleaning_SetFan(false);
}

void Cleaning_SetBrush(bool on)
{
  uint32_t compare = on ? duty_to_compare(&htim4, CLEANING_ON_DUTY_PERCENT) : 0U;
  __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, compare);
}

void Cleaning_SetPump(bool on)
{
  uint32_t compare = on ? duty_to_compare(&htim3, CLEANING_ON_DUTY_PERCENT) : 0U;
  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, compare);
}

void Cleaning_SetFan(bool on)
{
  /* PB3 is a simple GPIO output for the fan. */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, on ? GPIO_PIN_SET : GPIO_PIN_RESET);
}
