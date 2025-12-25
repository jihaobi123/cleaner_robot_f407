/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    bsp_sensors.c
  * @brief   Bumper and cliff sensor handling.
  ******************************************************************************
  */
/* USER CODE END Header */

#include "bsp_sensors.h"
#include "stm32f4xx_hal.h"

/* Adjust these pin mappings to your wiring if needed. */
#define PIN_BUMP_LEFT    GPIO_PIN_4   /* PC4 */
#define PIN_BUMP_RIGHT   GPIO_PIN_5   /* PC5 */
#define PIN_CLIFF_LEFT   GPIO_PIN_7   /* PC7 */
#define PIN_CLIFF_RIGHT  GPIO_PIN_0   /* PB0 */
#define PIN_CLIFF_FRONT  GPIO_PIN_12  /* PB12 */

static volatile SensorState s_events;
static SensorState s_state;

void Sensors_Init(void)
{
  /* GPIO and EXTI are configured by CubeMX. */
  Sensors_ClearEvents();
}

void Sensors_Poll(void)
{
  SensorState snapshot;

  __disable_irq();
  snapshot = s_events;
  __enable_irq();

  s_state = snapshot;
}

SensorState Sensors_GetState(void)
{
  return s_state;
}

void Sensors_ClearEvents(void)
{
  __disable_irq();
  s_events.bump_left = false;
  s_events.bump_right = false;
  s_events.cliff_left = false;
  s_events.cliff_right = false;
  s_events.cliff_front = false;
  __enable_irq();
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  if (GPIO_Pin == PIN_BUMP_LEFT)
  {
    s_events.bump_left = true;
  }
  else if (GPIO_Pin == PIN_BUMP_RIGHT)
  {
    s_events.bump_right = true;
  }
  else if (GPIO_Pin == PIN_CLIFF_LEFT)
  {
    s_events.cliff_left = true;
  }
  else if (GPIO_Pin == PIN_CLIFF_RIGHT)
  {
    s_events.cliff_right = true;
  }
  else if (GPIO_Pin == PIN_CLIFF_FRONT)
  {
    s_events.cliff_front = true;
  }
  else
  {
    /* Unused EXTI line. */
  }
}
