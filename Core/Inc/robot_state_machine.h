/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    robot_state_machine.h
  * @brief   High-level robot state machine.
  ******************************************************************************
  */
/* USER CODE END Header */
#ifndef ROBOT_STATE_MACHINE_H
#define ROBOT_STATE_MACHINE_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

typedef enum
{
  ROBOT_STATE_IDLE = 0,
  ROBOT_STATE_FORWARD,
  ROBOT_STATE_AVOID_BUMP_LEFT,
  ROBOT_STATE_AVOID_BUMP_RIGHT,
  ROBOT_STATE_AVOID_CLIFF_LEFT,
  ROBOT_STATE_AVOID_CLIFF_RIGHT,
  ROBOT_STATE_AVOID_CLIFF_FRONT,
} RobotState;

void RobotSM_Init(void);
void RobotSM_Update(void);

#ifdef __cplusplus
}
#endif

#endif /* ROBOT_STATE_MACHINE_H */
