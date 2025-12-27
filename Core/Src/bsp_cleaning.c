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

void Cleaning_Init(void)
{
  /* 初始化清扫 GPIO，并默认关闭输出。 */
  (void)HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_1); /* PB4 水泵 */
  (void)HAL_TIM_PWM_Stop(&htim4, TIM_CHANNEL_1); /* PD12 左刷子 */
  (void)HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_3); /* PA2 右刷子 */

  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  {
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;

    GPIO_InitStruct.Pin = GPIO_PIN_4;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_12;
    HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_2;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
  }

  Cleaning_SetPump(false);
  Cleaning_SetBrush(false);
  Cleaning_SetFan(false);
}

void Cleaning_SetBrush(bool on)
{
  /* 同时控制左右刷子 MOS 开关。 */
  Cleaning_SetBrushLeft(on);
  Cleaning_SetBrushRight(on);
}

void Cleaning_SetBrushLeft(bool on)
{
  /* 控制左刷子 MOS 开关（PD12）。 */
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, on ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

void Cleaning_SetBrushRight(bool on)
{
  /* 控制右刷子 MOS 开关（PA2）。 */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, on ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

void Cleaning_SetPump(bool on)
{
  /* 控制水泵 MOS 开关（PB4）。 */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, on ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

void Cleaning_SetFan(bool on)
{
  /* 控制风机 GPIO 开关。 */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, on ? GPIO_PIN_SET : GPIO_PIN_RESET);
}
