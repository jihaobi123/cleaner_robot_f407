#ifndef ROBOT_COMM_H
#define ROBOT_COMM_H

#include <stdbool.h>
#include <stdint.h>
#include "usart.h"

#define ROBOT_COMM_HEADER_FIRST_BYTE 0xAA
#define ROBOT_COMM_HEADER_SECOND_BYTE 0x55

#ifndef ROBOT_COMM_MAX_PAYLOAD
#define ROBOT_COMM_MAX_PAYLOAD 64U
#endif

#ifndef ROBOT_COMM_CRC_POLY
#define ROBOT_COMM_CRC_POLY 0xA001U
#endif

#ifndef ROBOT_COMM_CRC_INIT
#define ROBOT_COMM_CRC_INIT 0xFFFFU
#endif

typedef enum
{
  CMD_SEND_TASK        = 0x01,
  CMD_REQUEST_STATUS   = 0x02,
  CMD_CANCEL_TASK      = 0x03,
  CMD_EMERGENCY_STOP   = 0x04,

  RESP_TASK_ACK        = 0x81,
  RESP_STATUS          = 0x82,
  RESP_ERROR           = 0xFF,
} RobotComm_CommandId;

typedef enum
{
  ROBOT_COMM_STATE_IDLE = 0,
  ROBOT_COMM_STATE_RUNNING,
  ROBOT_COMM_STATE_CANCELLED,
  ROBOT_COMM_STATE_EMERGENCY_STOP,
} RobotComm_State;

typedef struct
{
  uint32_t task_id;
  float target_x;
  float target_y;
  uint8_t cleaning_type;
  uint8_t priority;
} RobotComm_Task;

typedef struct
{
  RobotComm_State state;
  RobotComm_Task active_task;
} RobotComm_Status;

void RobotComm_Init(UART_HandleTypeDef *huart);
void RobotComm_Process(void);
bool RobotComm_HasActiveTask(void);
bool RobotComm_IsEmergencyStop(void);
RobotComm_Status RobotComm_GetStatus(void);

__weak void RobotComm_OnTaskReceived(const RobotComm_Task *task);
__weak void RobotComm_OnTaskCancelled(uint32_t task_id);
__weak void RobotComm_OnEmergencyStop(void);

#endif /* ROBOT_COMM_H */
