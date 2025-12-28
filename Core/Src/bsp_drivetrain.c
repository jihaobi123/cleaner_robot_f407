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
#include "gpio.h"

#define DRIVETRAIN_MAX_NORM  1.0f
#define DRIVETRAIN_MIN_NORM -1.0f

/* 方向控制引脚 */
#define DIR_LEFT_GPIO_Port   GPIOB
#define DIR_LEFT_Pin         GPIO_PIN_15
#define DIR_RIGHT_GPIO_Port  GPIOC
#define DIR_RIGHT_Pin        GPIO_PIN_3

static float clamp_norm(float v)
{
  if (v > DRIVETRAIN_MAX_NORM)  return DRIVETRAIN_MAX_NORM;
  if (v < DRIVETRAIN_MIN_NORM)  return DRIVETRAIN_MIN_NORM;
  return v;
}

static uint32_t duty_to_ccr(TIM_HandleTypeDef *htim, float duty_01)
{
  if (duty_01 < 0.0f) duty_01 = 0.0f;
  if (duty_01 > 1.0f) duty_01 = 1.0f;

  uint32_t arr = __HAL_TIM_GET_AUTORELOAD(htim);
  return (uint32_t)((float)arr * duty_01 + 0.5f);
}

/**
 * Mapping rule (as requested):
 *  - norm in [-1,1]
 *  - mag = |norm|
 *  - forward: IN2=0, PWM=mag
 *  - reverse: IN2=1, PWM=mag
 *  - mag==0: PWM=0, IN2=0
 *  - IN1 (PWM) and IN2 (DIR GPIO) updated in the same call
 */
static void set_wheel_user_map(TIM_HandleTypeDef *htim, uint32_t ch,
                               GPIO_TypeDef *dir_port, uint16_t dir_pin,
                               float norm)
{
  float mag = (norm >= 0.0f) ? norm : -norm;
  if (mag > 1.0f) mag = 1.0f;

  /* 停止（滑行停）：PWM=0, IN2=0 */
  if (mag <= 0.0f)
  {
    /* 同步更新 IN2 + PWM */
    HAL_GPIO_WritePin(dir_port, dir_pin, GPIO_PIN_RESET);   /* IN2=0 */
    __HAL_TIM_SET_COMPARE(htim, ch, 0U);                    /* IN1=0% */
    return;
  }

  /* 方向：norm>=0 -> IN2=0；norm<0 -> IN2=1 */
  GPIO_PinState in2 = (norm >= 0.0f) ? GPIO_PIN_RESET : GPIO_PIN_SET;

  /* Duty always follows |norm|; direction by sign only. */
  float duty = mag;

  /* 璁＄畻 CCR */
  uint32_t ccr = duty_to_ccr(htim, duty);

  /* IN1/IN2 同步更新（先计算，再写 GPIO 与 CCR） */
  HAL_GPIO_WritePin(dir_port, dir_pin, in2);
  __HAL_TIM_SET_COMPARE(htim, ch, ccr);
}

void Drivetrain_Init(void)
{
  (void)HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_2);
  (void)HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);

  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull  = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;

  GPIO_InitStruct.Pin = DIR_LEFT_Pin;
  HAL_GPIO_Init(DIR_LEFT_GPIO_Port, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = DIR_RIGHT_Pin;
  HAL_GPIO_Init(DIR_RIGHT_GPIO_Port, &GPIO_InitStruct);

  /* 上电默认停止 */
  HAL_GPIO_WritePin(DIR_LEFT_GPIO_Port, DIR_LEFT_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(DIR_RIGHT_GPIO_Port, DIR_RIGHT_Pin, GPIO_PIN_RESET);
  __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_2, 0U);
  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 0U);
}

void Drivetrain_SetRaw(float left_norm, float right_norm)
{
  float left  = clamp_norm(left_norm);
  float right = clamp_norm(right_norm);

  /* 左轮：TIM8_CH2 \+ DIR_LEFT\(PB15\)
     右轮：TIM3_CH2 \+ DIR_RIGHT\(PC3\)
     （保持当前映射） */
  set_wheel_user_map(&htim8, TIM_CHANNEL_2, DIR_LEFT_GPIO_Port, DIR_LEFT_Pin, left);
  set_wheel_user_map(&htim3, TIM_CHANNEL_2, DIR_RIGHT_GPIO_Port, DIR_RIGHT_Pin, right);
}

void Drivetrain_Stop(void)
{
  /* 停止：IN2=0, PWM=0 */
  HAL_GPIO_WritePin(DIR_LEFT_GPIO_Port, DIR_LEFT_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(DIR_RIGHT_GPIO_Port, DIR_RIGHT_Pin, GPIO_PIN_RESET);
  __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_2, 0U);
  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 0U);
}

/* 方向辅助：低=正转，高=反转 */
void Drivetrain_SetDirection(bool left_forward, bool right_forward)
{
  /* 正转：IN2=0，反转：IN2=1 */
  HAL_GPIO_WritePin(DIR_LEFT_GPIO_Port, DIR_LEFT_Pin,
                    left_forward ? GPIO_PIN_RESET : GPIO_PIN_SET);
  HAL_GPIO_WritePin(DIR_RIGHT_GPIO_Port, DIR_RIGHT_Pin,
                    right_forward ? GPIO_PIN_RESET : GPIO_PIN_SET);
}

float Drivetrain_UnitToNorm(float units)
{
  const float d0 = 0.50f;
  const float kf = 0.17f;
  const float kr = 0.10f;
  float norm;

  if (units == 0)
  {
    return 0.0f;
  }

  if (units > 0)
  {
    norm = d0 + kf * (float)units;
  }
  else
  {
    float u = (float)(-units);
    norm = -(d0 - kr * u);
  }

  if (norm > 1.0f)
  {
    norm = 1.0f;
  }
  if (norm < -1.0f)
  {
    norm = -1.0f;
  }
  return norm;
}

void Drivetrain_SetUnits(float left_units, float right_units)
{
  Drivetrain_SetRaw(Drivetrain_UnitToNorm(left_units),
                    Drivetrain_UnitToNorm(right_units));
}

void Drivetrain_SetTwist(float v_linear, float v_angular)
{
  /* k: turning gain; adjust to match your wheelbase if needed. */
  const float k = 1.0f;
  float left_units  = v_linear - k * v_angular;
  float right_units = v_linear + k * v_angular;
  Drivetrain_SetUnits(left_units, right_units);
}

