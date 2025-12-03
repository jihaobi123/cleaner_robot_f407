#include "robot_comm.h"
#include <string.h>

#define TASK_PAYLOAD_SIZE 14U

typedef enum
{
  PARSER_WAIT_HEADER_AA = 0,
  PARSER_WAIT_HEADER_55,
  PARSER_WAIT_CMD,
  PARSER_WAIT_LEN_L,
  PARSER_WAIT_LEN_H,
  PARSER_WAIT_PAYLOAD,
  PARSER_WAIT_CRC_L,
  PARSER_WAIT_CRC_H,
} RobotComm_ParserState;

typedef struct
{
  RobotComm_ParserState state;
  uint8_t cmd_id;
  uint16_t payload_len;
  uint16_t payload_index;
  uint8_t payload[ROBOT_COMM_MAX_PAYLOAD];
  uint16_t crc;
} RobotComm_FrameBuilder;

typedef struct
{
  UART_HandleTypeDef *huart;
  uint8_t rx_byte;
  RobotComm_FrameBuilder frame;
  RobotComm_Status status;
} RobotComm_Context;

static RobotComm_Context s_ctx;

static void RobotComm_ResetFrame(void);
static void RobotComm_OnByte(uint8_t byte);
static void RobotComm_HandleFrame(uint8_t cmd_id, const uint8_t *payload, uint16_t length);
static void RobotComm_HandleTaskCommand(const uint8_t *payload, uint16_t length);
static void RobotComm_HandleStatusRequest(void);
static void RobotComm_HandleCancelTask(void);
static void RobotComm_HandleEmergencyStop(void);
static void RobotComm_SendFrame(uint8_t resp_id, const uint8_t *payload, uint16_t payload_len);
static void RobotComm_SendError(uint8_t error_code, uint8_t offending_cmd);
static uint16_t RobotComm_UpdateCRC(uint16_t crc, uint8_t data);
static uint16_t RobotComm_FinalizeCRC(uint16_t crc, const uint8_t *data, uint16_t length);

void RobotComm_Init(UART_HandleTypeDef *huart)
{
  memset(&s_ctx, 0, sizeof(s_ctx));
  s_ctx.huart = huart;
  s_ctx.status.state = ROBOT_COMM_STATE_IDLE;
  RobotComm_ResetFrame();
  HAL_UART_Receive_IT(s_ctx.huart, &s_ctx.rx_byte, 1);
}

void RobotComm_Process(void)
{
  /* Placeholder for future background work (timeouts, watchdogs, etc.). */
}

bool RobotComm_HasActiveTask(void)
{
  return s_ctx.status.state == ROBOT_COMM_STATE_RUNNING;
}

bool RobotComm_IsEmergencyStop(void)
{
  return s_ctx.status.state == ROBOT_COMM_STATE_EMERGENCY_STOP;
}

RobotComm_Status RobotComm_GetStatus(void)
{
  return s_ctx.status;
}

static void RobotComm_ResetFrame(void)
{
  s_ctx.frame.state = PARSER_WAIT_HEADER_AA;
  s_ctx.frame.payload_len = 0;
  s_ctx.frame.payload_index = 0;
  s_ctx.frame.crc = ROBOT_COMM_CRC_INIT;
}

static void RobotComm_OnByte(uint8_t byte)
{
  switch (s_ctx.frame.state)
  {
    case PARSER_WAIT_HEADER_AA:
      if (byte == ROBOT_COMM_HEADER_FIRST_BYTE)
      {
        s_ctx.frame.crc = RobotComm_UpdateCRC(ROBOT_COMM_CRC_INIT, byte);
        s_ctx.frame.state = PARSER_WAIT_HEADER_55;
      }
      break;

    case PARSER_WAIT_HEADER_55:
      if (byte == ROBOT_COMM_HEADER_SECOND_BYTE)
      {
        s_ctx.frame.crc = RobotComm_UpdateCRC(s_ctx.frame.crc, byte);
        s_ctx.frame.state = PARSER_WAIT_CMD;
      }
      else if (byte == ROBOT_COMM_HEADER_FIRST_BYTE)
      {
        /* Stay in header search if another 0xAA is received. */
        s_ctx.frame.crc = RobotComm_UpdateCRC(ROBOT_COMM_CRC_INIT, byte);
      }
      else
      {
        RobotComm_ResetFrame();
      }
      break;

    case PARSER_WAIT_CMD:
      s_ctx.frame.cmd_id = byte;
      s_ctx.frame.crc = RobotComm_UpdateCRC(s_ctx.frame.crc, byte);
      s_ctx.frame.state = PARSER_WAIT_LEN_L;
      break;

    case PARSER_WAIT_LEN_L:
      s_ctx.frame.payload_len = byte;
      s_ctx.frame.crc = RobotComm_UpdateCRC(s_ctx.frame.crc, byte);
      s_ctx.frame.state = PARSER_WAIT_LEN_H;
      break;

    case PARSER_WAIT_LEN_H:
      s_ctx.frame.payload_len |= ((uint16_t)byte << 8);
      s_ctx.frame.crc = RobotComm_UpdateCRC(s_ctx.frame.crc, byte);
      if (s_ctx.frame.payload_len > ROBOT_COMM_MAX_PAYLOAD)
      {
        RobotComm_SendError(0x01, s_ctx.frame.cmd_id);
        RobotComm_ResetFrame();
      }
      else if (s_ctx.frame.payload_len == 0U)
      {
        s_ctx.frame.state = PARSER_WAIT_CRC_L;
      }
      else
      {
        s_ctx.frame.state = PARSER_WAIT_PAYLOAD;
        s_ctx.frame.payload_index = 0;
      }
      break;

    case PARSER_WAIT_PAYLOAD:
      s_ctx.frame.payload[s_ctx.frame.payload_index++] = byte;
      s_ctx.frame.crc = RobotComm_UpdateCRC(s_ctx.frame.crc, byte);
      if (s_ctx.frame.payload_index >= s_ctx.frame.payload_len)
      {
        s_ctx.frame.state = PARSER_WAIT_CRC_L;
      }
      break;

    case PARSER_WAIT_CRC_L:
      s_ctx.frame.payload_index = byte; /* reuse field as low byte holder */
      s_ctx.frame.state = PARSER_WAIT_CRC_H;
      break;

    case PARSER_WAIT_CRC_H:
    {
      uint16_t received_crc = (uint16_t)s_ctx.frame.payload_index | ((uint16_t)byte << 8);
      if (received_crc == s_ctx.frame.crc)
      {
        RobotComm_HandleFrame(s_ctx.frame.cmd_id, s_ctx.frame.payload, s_ctx.frame.payload_len);
      }
      else
      {
        RobotComm_SendError(0x02, s_ctx.frame.cmd_id);
      }
      RobotComm_ResetFrame();
      break;
    }

    default:
      RobotComm_ResetFrame();
      break;
  }
}

static void RobotComm_HandleFrame(uint8_t cmd_id, const uint8_t *payload, uint16_t length)
{
  switch (cmd_id)
  {
    case CMD_SEND_TASK:
      RobotComm_HandleTaskCommand(payload, length);
      break;

    case CMD_REQUEST_STATUS:
      RobotComm_HandleStatusRequest();
      break;

    case CMD_CANCEL_TASK:
      RobotComm_HandleCancelTask();
      break;

    case CMD_EMERGENCY_STOP:
      RobotComm_HandleEmergencyStop();
      break;

    default:
      RobotComm_SendError(0x03, cmd_id);
      break;
  }
}

static void RobotComm_HandleTaskCommand(const uint8_t *payload, uint16_t length)
{
  if (length != TASK_PAYLOAD_SIZE)
  {
    RobotComm_SendError(0x04, CMD_SEND_TASK);
    return;
  }

  RobotComm_Task task;
  memcpy(&task.task_id, &payload[0], sizeof(task.task_id));
  memcpy(&task.target_x, &payload[4], sizeof(task.target_x));
  memcpy(&task.target_y, &payload[8], sizeof(task.target_y));
  task.cleaning_type = payload[12];
  task.priority = payload[13];

  s_ctx.status.active_task = task;
  s_ctx.status.state = ROBOT_COMM_STATE_RUNNING;

  RobotComm_OnTaskReceived(&task);

  uint8_t ack_payload[5];
  memcpy(&ack_payload[0], &task.task_id, sizeof(task.task_id));
  ack_payload[4] = 0x00; /* success */
  RobotComm_SendFrame(RESP_TASK_ACK, ack_payload, sizeof(ack_payload));
}

static void RobotComm_HandleStatusRequest(void)
{
  uint8_t payload[1 + sizeof(uint32_t) + sizeof(float) * 2 + 2];
  payload[0] = (uint8_t)s_ctx.status.state;
  memcpy(&payload[1], &s_ctx.status.active_task.task_id, sizeof(uint32_t));
  memcpy(&payload[5], &s_ctx.status.active_task.target_x, sizeof(float));
  memcpy(&payload[9], &s_ctx.status.active_task.target_y, sizeof(float));
  payload[13] = s_ctx.status.active_task.cleaning_type;
  payload[14] = s_ctx.status.active_task.priority;
  RobotComm_SendFrame(RESP_STATUS, payload, sizeof(payload));
}

static void RobotComm_HandleCancelTask(void)
{
  uint32_t cancelled_id = s_ctx.status.active_task.task_id;
  s_ctx.status.state = ROBOT_COMM_STATE_CANCELLED;
  memset(&s_ctx.status.active_task, 0, sizeof(s_ctx.status.active_task));
  RobotComm_OnTaskCancelled(cancelled_id);

  uint8_t payload[5];
  memcpy(&payload[0], &cancelled_id, sizeof(cancelled_id));
  payload[4] = 0x00;
  RobotComm_SendFrame(RESP_TASK_ACK, payload, sizeof(payload));
}

static void RobotComm_HandleEmergencyStop(void)
{
  s_ctx.status.state = ROBOT_COMM_STATE_EMERGENCY_STOP;
  memset(&s_ctx.status.active_task, 0, sizeof(s_ctx.status.active_task));
  RobotComm_OnEmergencyStop();
  RobotComm_SendFrame(RESP_ERROR, (uint8_t[]){0xEE, CMD_EMERGENCY_STOP}, 2);
}

static uint16_t RobotComm_UpdateCRC(uint16_t crc, uint8_t data)
{
  crc ^= data;
  for (uint8_t i = 0; i < 8; i++)
  {
    if (crc & 0x0001U)
    {
      crc = (crc >> 1U) ^ ROBOT_COMM_CRC_POLY;
    }
    else
    {
      crc >>= 1U;
    }
  }
  return crc;
}

static uint16_t RobotComm_FinalizeCRC(uint16_t crc, const uint8_t *data, uint16_t length)
{
  for (uint16_t i = 0; i < length; i++)
  {
    crc = RobotComm_UpdateCRC(crc, data[i]);
  }
  return crc;
}

static void RobotComm_SendFrame(uint8_t resp_id, const uint8_t *payload, uint16_t payload_len)
{
  uint8_t frame[2 + 1 + 2 + ROBOT_COMM_MAX_PAYLOAD + 2];
  uint16_t index = 0;
  frame[index++] = ROBOT_COMM_HEADER_FIRST_BYTE;
  frame[index++] = ROBOT_COMM_HEADER_SECOND_BYTE;
  frame[index++] = resp_id;
  frame[index++] = (uint8_t)(payload_len & 0xFFU);
  frame[index++] = (uint8_t)((payload_len >> 8) & 0xFFU);

  if (payload_len > 0U && payload != NULL)
  {
    memcpy(&frame[index], payload, payload_len);
    index += payload_len;
  }

  uint16_t crc = RobotComm_FinalizeCRC(ROBOT_COMM_CRC_INIT, frame, index);
  frame[index++] = (uint8_t)(crc & 0xFFU);
  frame[index++] = (uint8_t)((crc >> 8) & 0xFFU);

  HAL_UART_Transmit(s_ctx.huart, frame, index, HAL_MAX_DELAY);
}

static void RobotComm_SendError(uint8_t error_code, uint8_t offending_cmd)
{
  uint8_t payload[2] = {error_code, offending_cmd};
  RobotComm_SendFrame(RESP_ERROR, payload, sizeof(payload));
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart == s_ctx.huart)
  {
    RobotComm_OnByte(s_ctx.rx_byte);
    HAL_UART_Receive_IT(s_ctx.huart, &s_ctx.rx_byte, 1);
  }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
  if (huart == s_ctx.huart)
  {
    HAL_UART_Receive_IT(s_ctx.huart, &s_ctx.rx_byte, 1);
  }
}

__weak void RobotComm_OnTaskReceived(const RobotComm_Task *task)
{
  (void)task;
}

__weak void RobotComm_OnTaskCancelled(uint32_t task_id)
{
  (void)task_id;
}

__weak void RobotComm_OnEmergencyStop(void)
{
}
