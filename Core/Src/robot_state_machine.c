/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    robot_state_machine.c
  * @brief   High-level robot state machine.
  ******************************************************************************
  */
/* USER CODE END Header */

#include "robot_state_machine.h"
#include "bsp_drivetrain.h"
#include "bsp_sensors.h"
#include "stm32f4xx_hal.h"

#define SPEED_FORWARD      0.5f
#define SPEED_TURN         0.4f
#define SPEED_BACKUP       0.4f

#define TURN_MS_BUMP       500U
#define TURN_MS_CLIFF      700U
#define BACKUP_MS_FRONT    500U
#define TURN_MS_FRONT      700U

static RobotState s_state = ROBOT_STATE_IDLE;
static uint32_t s_state_enter_ms = 0;
static uint8_t s_front_phase = 0;

static void set_state(RobotState new_state)
{
  s_state = new_state;
  s_state_enter_ms = HAL_GetTick();
  if (new_state == ROBOT_STATE_AVOID_CLIFF_FRONT)
  {
    s_front_phase = 0;
  }
}

void RobotSM_Init(void)
{
  set_state(ROBOT_STATE_FORWARD);
}

void RobotSM_Update(void)
{
  uint32_t now_ms = HAL_GetTick();
  uint32_t elapsed = now_ms - s_state_enter_ms;
  SensorState sensors;

  Sensors_Poll();
  sensors = Sensors_GetState();

  switch (s_state)
  {
  case ROBOT_STATE_IDLE:
    Drivetrain_Stop();
    break;

  case ROBOT_STATE_FORWARD:
    Drivetrain_SetRaw(SPEED_FORWARD, SPEED_FORWARD);

    if (sensors.cliff_front)
    {
      set_state(ROBOT_STATE_AVOID_CLIFF_FRONT);
      Sensors_ClearEvents();
    }
    else if (sensors.cliff_left)
    {
      set_state(ROBOT_STATE_AVOID_CLIFF_LEFT);
      Sensors_ClearEvents();
    }
    else if (sensors.cliff_right)
    {
      set_state(ROBOT_STATE_AVOID_CLIFF_RIGHT);
      Sensors_ClearEvents();
    }
    else if (sensors.bump_left)
    {
      set_state(ROBOT_STATE_AVOID_BUMP_LEFT);
      Sensors_ClearEvents();
    }
    else if (sensors.bump_right)
    {
      set_state(ROBOT_STATE_AVOID_BUMP_RIGHT);
      Sensors_ClearEvents();
    }
    break;

  case ROBOT_STATE_AVOID_BUMP_LEFT:
    Drivetrain_SetRaw(SPEED_TURN, -SPEED_TURN);
    if (elapsed >= TURN_MS_BUMP)
    {
      set_state(ROBOT_STATE_FORWARD);
      Sensors_ClearEvents();
    }
    break;

  case ROBOT_STATE_AVOID_BUMP_RIGHT:
    Drivetrain_SetRaw(-SPEED_TURN, SPEED_TURN);
    if (elapsed >= TURN_MS_BUMP)
    {
      set_state(ROBOT_STATE_FORWARD);
      Sensors_ClearEvents();
    }
    break;

  case ROBOT_STATE_AVOID_CLIFF_LEFT:
    Drivetrain_SetRaw(SPEED_TURN, -SPEED_TURN);
    if (elapsed >= TURN_MS_CLIFF)
    {
      set_state(ROBOT_STATE_FORWARD);
      Sensors_ClearEvents();
    }
    break;

  case ROBOT_STATE_AVOID_CLIFF_RIGHT:
    Drivetrain_SetRaw(-SPEED_TURN, SPEED_TURN);
    if (elapsed >= TURN_MS_CLIFF)
    {
      set_state(ROBOT_STATE_FORWARD);
      Sensors_ClearEvents();
    }
    break;

  case ROBOT_STATE_AVOID_CLIFF_FRONT:
    if (s_front_phase == 0U)
    {
      Drivetrain_SetRaw(-SPEED_BACKUP, -SPEED_BACKUP);
      if (elapsed >= BACKUP_MS_FRONT)
      {
        s_front_phase = 1U;
        s_state_enter_ms = now_ms;
      }
    }
    else
    {
      Drivetrain_SetRaw(SPEED_TURN, -SPEED_TURN);
      if (elapsed >= TURN_MS_FRONT)
      {
        set_state(ROBOT_STATE_FORWARD);
        Sensors_ClearEvents();
      }
    }
    break;

  default:
    set_state(ROBOT_STATE_IDLE);
    break;
  }
}
