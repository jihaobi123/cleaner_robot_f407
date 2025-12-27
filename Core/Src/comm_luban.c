/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    comm_luban.c
  * @brief   LubanCat UART protocol handler.
  ******************************************************************************
  */
/* USER CODE END Header */

#include "comm_luban.h"
#include <string.h>

#define COMM_LUBAN_HEADER_0            0xAA
#define COMM_LUBAN_HEADER_1            0x55
#define COMM_LUBAN_RX_BUF_SIZE         512
#define COMM_LUBAN_MAX_PAYLOAD         256
#define COMM_LUBAN_TX_TIMEOUT_MS       20

typedef enum
{
  PARSER_WAIT_H0 = 0,
  PARSER_WAIT_H1,
  PARSER_READ_CMD,
  PARSER_READ_LEN0,
  PARSER_READ_LEN1,
  PARSER_READ_PAYLOAD,
  PARSER_READ_CRC0,
  PARSER_READ_CRC1
} ParserState;

typedef struct
{
  ParserState state;
  uint8_t cmd_id;
  uint16_t payload_len;
  uint16_t payload_idx;
  uint8_t payload[COMM_LUBAN_MAX_PAYLOAD];
  uint16_t crc_recv;
  uint16_t crc_calc;
} ParserCtx;

static UART_HandleTypeDef *s_huart = NULL;
static uint8_t s_rx_byte = 0;

static volatile uint16_t s_rb_head = 0;
static volatile uint16_t s_rb_tail = 0;
static uint8_t s_rb_data[COMM_LUBAN_RX_BUF_SIZE];

static ParserCtx s_parser;

static LubanTask s_last_task;
static volatile bool s_task_pending = false;

static uint16_t crc16_update(uint16_t crc, uint8_t data)
{
  /* Replace with your CRC16 update function if different. */
  crc ^= data;
  for (uint8_t i = 0; i < 8; i++)
  {
    if (crc & 0x0001)
    {
      crc = (crc >> 1) ^ 0xA001;
    }
    else
    {
      crc >>= 1;
    }
  }
  return crc;
}

static uint16_t crc16_compute(const uint8_t *data, uint16_t len)
{
  uint16_t crc = 0xFFFF;
  for (uint16_t i = 0; i < len; i++)
  {
    crc = crc16_update(crc, data[i]);
  }
  return crc;
}

static void rb_push(uint8_t byte)
{
  uint16_t next = (uint16_t)((s_rb_head + 1U) % COMM_LUBAN_RX_BUF_SIZE);
  if (next == s_rb_tail)
  {
    return;
  }
  s_rb_data[s_rb_head] = byte;
  s_rb_head = next;
}

static bool rb_pop(uint8_t *out)
{
  if (s_rb_head == s_rb_tail)
  {
    return false;
  }
  *out = s_rb_data[s_rb_tail];
  s_rb_tail = (uint16_t)((s_rb_tail + 1U) % COMM_LUBAN_RX_BUF_SIZE);
  return true;
}

static void parser_reset(void)
{
  s_parser.state = PARSER_WAIT_H0;
  s_parser.cmd_id = 0;
  s_parser.payload_len = 0;
  s_parser.payload_idx = 0;
  s_parser.crc_recv = 0;
  s_parser.crc_calc = 0xFFFF;
}

static void send_frame(uint8_t resp_id, const uint8_t *payload, uint16_t payload_len)
{
  uint8_t frame[2 + 1 + 2 + COMM_LUBAN_MAX_PAYLOAD + 2];
  uint16_t idx = 0;
  uint16_t crc;

  if (payload_len > COMM_LUBAN_MAX_PAYLOAD)
  {
    payload_len = COMM_LUBAN_MAX_PAYLOAD;
  }

  frame[idx++] = COMM_LUBAN_HEADER_0;
  frame[idx++] = COMM_LUBAN_HEADER_1;
  frame[idx++] = resp_id;
  frame[idx++] = (uint8_t)(payload_len & 0xFF);
  frame[idx++] = (uint8_t)((payload_len >> 8) & 0xFF);

  if (payload_len > 0 && payload != NULL)
  {
    memcpy(&frame[idx], payload, payload_len);
    idx = (uint16_t)(idx + payload_len);
  }

  crc = crc16_compute(&frame[2], (uint16_t)(1 + 2 + payload_len));
  frame[idx++] = (uint8_t)(crc & 0xFF);
  frame[idx++] = (uint8_t)((crc >> 8) & 0xFF);

  if (s_huart != NULL)
  {
    (void)HAL_UART_Transmit(s_huart, frame, idx, COMM_LUBAN_TX_TIMEOUT_MS);
  }
}

static void send_error(uint8_t code)
{
  send_frame(RESP_ERROR, &code, 1);
}

static void handle_cmd_send_task(const uint8_t *payload, uint16_t len)
{
  if (len != 14)
  {
    send_error(1);
    return;
  }

  memcpy(&s_last_task.task_id, &payload[0], sizeof(uint32_t));
  memcpy(&s_last_task.x, &payload[4], sizeof(float));
  memcpy(&s_last_task.y, &payload[8], sizeof(float));
  s_last_task.cleaning_type = payload[12];
  s_last_task.priority = payload[13];
  s_task_pending = true;

  {
    uint8_t ack[5];
    memcpy(&ack[0], &s_last_task.task_id, sizeof(uint32_t));
    ack[4] = 0;
    send_frame(RESP_TASK_ACK, ack, (uint16_t)sizeof(ack));
  }
}

static void handle_cmd_request_status(void)
{
  Comm_Luban_SendStatus("OK");
}

static void handle_cmd_cancel_task(void)
{
  Comm_Luban_SendStatus("CANCELLED");
}

static void handle_cmd_emergency_stop(void)
{
  Comm_Luban_SendStatus("EMERGENCY_STOP");
}

static void handle_frame(uint8_t cmd_id, const uint8_t *payload, uint16_t payload_len)
{
  switch (cmd_id)
  {
  case CMD_SEND_TASK:
    handle_cmd_send_task(payload, payload_len);
    break;
  case CMD_REQUEST_STATUS:
    handle_cmd_request_status();
    break;
  case CMD_CANCEL_TASK:
    handle_cmd_cancel_task();
    break;
  case CMD_EMERGENCY_STOP:
    handle_cmd_emergency_stop();
    break;
  default:
    send_error(2);
    break;
  }
}

void Comm_Luban_Init(UART_HandleTypeDef *huart)
{
  /* 初始化通信模块：绑定 UART 并启动中断接收。 */
  s_huart = huart;
  parser_reset();
  s_rb_head = 0;
  s_rb_tail = 0;
  s_task_pending = false;

  if (s_huart != NULL)
  {
    (void)HAL_UART_Receive_IT(s_huart, &s_rx_byte, 1);
  }
}

void Comm_Luban_Poll(void)
{
  /* 主循环调用：解析接收缓冲并处理完整帧。 */
  uint8_t byte;

  while (rb_pop(&byte))
  {
    switch (s_parser.state)
    {
    case PARSER_WAIT_H0:
      if (byte == COMM_LUBAN_HEADER_0)
      {
        s_parser.state = PARSER_WAIT_H1;
      }
      break;
    case PARSER_WAIT_H1:
      if (byte == COMM_LUBAN_HEADER_1)
      {
        s_parser.state = PARSER_READ_CMD;
        s_parser.crc_calc = 0xFFFF;
      }
      else if (byte == COMM_LUBAN_HEADER_0)
      {
        s_parser.state = PARSER_WAIT_H1;
      }
      else
      {
        s_parser.state = PARSER_WAIT_H0;
      }
      break;
    case PARSER_READ_CMD:
      s_parser.cmd_id = byte;
      s_parser.crc_calc = crc16_update(s_parser.crc_calc, byte);
      s_parser.state = PARSER_READ_LEN0;
      break;
    case PARSER_READ_LEN0:
      s_parser.payload_len = byte;
      s_parser.crc_calc = crc16_update(s_parser.crc_calc, byte);
      s_parser.state = PARSER_READ_LEN1;
      break;
    case PARSER_READ_LEN1:
      s_parser.payload_len |= (uint16_t)(byte << 8);
      s_parser.crc_calc = crc16_update(s_parser.crc_calc, byte);
      s_parser.payload_idx = 0;
      if (s_parser.payload_len > COMM_LUBAN_MAX_PAYLOAD)
      {
        send_error(3);
        parser_reset();
      }
      else if (s_parser.payload_len == 0)
      {
        s_parser.state = PARSER_READ_CRC0;
      }
      else
      {
        s_parser.state = PARSER_READ_PAYLOAD;
      }
      break;
    case PARSER_READ_PAYLOAD:
      s_parser.payload[s_parser.payload_idx++] = byte;
      s_parser.crc_calc = crc16_update(s_parser.crc_calc, byte);
      if (s_parser.payload_idx >= s_parser.payload_len)
      {
        s_parser.state = PARSER_READ_CRC0;
      }
      break;
    case PARSER_READ_CRC0:
      s_parser.crc_recv = byte;
      s_parser.state = PARSER_READ_CRC1;
      break;
    case PARSER_READ_CRC1:
      s_parser.crc_recv |= (uint16_t)(byte << 8);
      if (s_parser.crc_recv == s_parser.crc_calc)
      {
        handle_frame(s_parser.cmd_id, s_parser.payload, s_parser.payload_len);
      }
      else
      {
        send_error(4);
      }
      parser_reset();
      break;
    default:
      parser_reset();
      break;
    }
  }
}

bool Comm_Luban_GetNextTask(LubanTask *out)
{
  /* 获取最新任务，成功时返回 true 并清除待处理标志。 */
  if (out == NULL)
  {
    return false;
  }

  if (!s_task_pending)
  {
    return false;
  }

  *out = s_last_task;
  s_task_pending = false;
  return true;
}

void Comm_Luban_SendStatus(const char *status_str)
{
  /* 发送状态字符串响应（RESP_STATUS）。 */
  uint16_t len = 0;

  if (status_str != NULL)
  {
    size_t slen = strlen(status_str);
    if (slen > COMM_LUBAN_MAX_PAYLOAD)
    {
      slen = COMM_LUBAN_MAX_PAYLOAD;
    }
    len = (uint16_t)slen;
  }

  send_frame(RESP_STATUS, (const uint8_t *)status_str, len);
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  /* UART 接收中断回调：写入环形缓冲并续接收。 */
  if (huart == s_huart)
  {
    rb_push(s_rx_byte);
    (void)HAL_UART_Receive_IT(s_huart, &s_rx_byte, 1);
  }
}
