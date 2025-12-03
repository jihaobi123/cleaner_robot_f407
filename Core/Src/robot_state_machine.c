#include "robot_state_machine.h"

#include "bsp_sensors.h"
#include "main.h"

/* External drivetrain control function. */
void Drivetrain_SetRaw(float left, float right);

#define FORWARD_SPEED               0.5f
#define TURN_SPEED                  0.5f
#define BACKUP_SPEED               -0.3f

#define AVOID_BUMP_DURATION_MS      400U
#define AVOID_CLIFF_DURATION_MS     450U
#define AVOID_FRONT_BACKUP_MS       500U
#define AVOID_FRONT_TURN_MS         500U

static RobotState current_state = ROBOT_STATE_IDLE;
static uint32_t state_timestamp_ms = 0U;
static uint8_t cliff_front_phase = 0U;

static void RobotSM_EnterState(RobotState new_state)
{
    current_state = new_state;
    state_timestamp_ms = HAL_GetTick();
}

static void RobotSM_HandleForward(const SensorState *sensors)
{
    if (sensors->bump_left) {
        RobotSM_EnterState(ROBOT_STATE_AVOID_BUMP_LEFT);
        Drivetrain_SetRaw(TURN_SPEED, -TURN_SPEED);
        Sensors_ClearEvents();
        return;
    }

    if (sensors->bump_right) {
        RobotSM_EnterState(ROBOT_STATE_AVOID_BUMP_RIGHT);
        Drivetrain_SetRaw(-TURN_SPEED, TURN_SPEED);
        Sensors_ClearEvents();
        return;
    }

    if (sensors->cliff_left) {
        RobotSM_EnterState(ROBOT_STATE_AVOID_CLIFF_LEFT);
        Drivetrain_SetRaw(TURN_SPEED, -TURN_SPEED);
        Sensors_ClearEvents();
        return;
    }

    if (sensors->cliff_right) {
        RobotSM_EnterState(ROBOT_STATE_AVOID_CLIFF_RIGHT);
        Drivetrain_SetRaw(-TURN_SPEED, TURN_SPEED);
        Sensors_ClearEvents();
        return;
    }

    if (sensors->cliff_front) {
        RobotSM_EnterState(ROBOT_STATE_AVOID_CLIFF_FRONT);
        cliff_front_phase = 0U;
        Drivetrain_SetRaw(BACKUP_SPEED, BACKUP_SPEED);
        Sensors_ClearEvents();
        return;
    }

    /* Default forward motion. */
    Drivetrain_SetRaw(FORWARD_SPEED, FORWARD_SPEED);
}

static void RobotSM_HandleTimedState(RobotState next_state, uint32_t duration_ms, float left_speed, float right_speed)
{
    uint32_t elapsed = HAL_GetTick() - state_timestamp_ms;
    if (elapsed >= duration_ms) {
        RobotSM_EnterState(next_state);
        Drivetrain_SetRaw(FORWARD_SPEED, FORWARD_SPEED);
    } else {
        Drivetrain_SetRaw(left_speed, right_speed);
    }
}

static void RobotSM_HandleCliffFront(void)
{
    uint32_t elapsed = HAL_GetTick() - state_timestamp_ms;

    if (cliff_front_phase == 0U) {
        /* Back up for a fixed duration. */
        if (elapsed >= AVOID_FRONT_BACKUP_MS) {
            cliff_front_phase = 1U;
            RobotSM_EnterState(ROBOT_STATE_AVOID_CLIFF_FRONT);
            Drivetrain_SetRaw(TURN_SPEED, -TURN_SPEED);
        } else {
            Drivetrain_SetRaw(BACKUP_SPEED, BACKUP_SPEED);
        }
    } else {
        /* Turn to clear the obstacle, then resume forward motion. */
        if (elapsed >= AVOID_FRONT_TURN_MS) {
            RobotSM_EnterState(ROBOT_STATE_FORWARD);
            Drivetrain_SetRaw(FORWARD_SPEED, FORWARD_SPEED);
        } else {
            Drivetrain_SetRaw(TURN_SPEED, -TURN_SPEED);
        }
    }
}

void RobotSM_Init(void)
{
    Sensors_ClearEvents();
    Sensors_Poll();
    RobotSM_EnterState(ROBOT_STATE_FORWARD);
    Drivetrain_SetRaw(FORWARD_SPEED, FORWARD_SPEED);
}

void RobotSM_Update(void)
{
    Sensors_Poll();
    SensorState sensors = Sensors_GetState();

    if (current_state == ROBOT_STATE_IDLE) {
        RobotSM_EnterState(ROBOT_STATE_FORWARD);
    }

    switch (current_state) {
    case ROBOT_STATE_FORWARD:
        RobotSM_HandleForward(&sensors);
        break;
    case ROBOT_STATE_AVOID_BUMP_LEFT:
        RobotSM_HandleTimedState(ROBOT_STATE_FORWARD, AVOID_BUMP_DURATION_MS, TURN_SPEED, -TURN_SPEED);
        break;
    case ROBOT_STATE_AVOID_BUMP_RIGHT:
        RobotSM_HandleTimedState(ROBOT_STATE_FORWARD, AVOID_BUMP_DURATION_MS, -TURN_SPEED, TURN_SPEED);
        break;
    case ROBOT_STATE_AVOID_CLIFF_LEFT:
        RobotSM_HandleTimedState(ROBOT_STATE_FORWARD, AVOID_CLIFF_DURATION_MS, TURN_SPEED, -TURN_SPEED);
        break;
    case ROBOT_STATE_AVOID_CLIFF_RIGHT:
        RobotSM_HandleTimedState(ROBOT_STATE_FORWARD, AVOID_CLIFF_DURATION_MS, -TURN_SPEED, TURN_SPEED);
        break;
    case ROBOT_STATE_AVOID_CLIFF_FRONT:
        RobotSM_HandleCliffFront();
        break;
    case ROBOT_STATE_IDLE:
    default:
        break;
    }
}
