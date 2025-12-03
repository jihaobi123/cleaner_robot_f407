#include "comm_luban.h"
#include <string.h>

/* Simple UART-based communication layer for the Luban protocol.
 * Uses interrupt-driven single-byte reception with a circular buffer. */

#define LUBAN_RX_BUFFER_SIZE 256U
#define LUBAN_TASK_QUEUE_SIZE 4U
#define LUBAN_FRAME_HEADER_L 0x55U
#define LUBAN_FRAME_HEADER_H 0xAAU
#define COMM_UART_TIMEOUT 50U

static UART_HandleTypeDef *s_uart = NULL;
static uint8_t s_rx_byte = 0U;
static volatile uint16_t s_rb_head = 0U;
static volatile uint16_t s_rb_tail = 0U;
static uint8_t s_rx_buffer[LUBAN_RX_BUFFER_SIZE];

static LubanTask s_task_queue[LUBAN_TASK_QUEUE_SIZE];
static uint8_t s_task_head = 0U;
static uint8_t s_task_tail = 0U;

static uint16_t Comm_Luban_CRC16(const uint8_t *data, uint16_t length);
static void Comm_Luban_RingPush(uint8_t byte);
static bool Comm_Luban_RingPop(uint8_t *out);
static uint16_t Comm_Luban_RingAvailable(void);
static bool Comm_Luban_Peek(uint16_t index, uint8_t *out);
static void Comm_Luban_Consume(uint16_t length);
static void Comm_Luban_HandleFrame(const uint8_t *frame, uint16_t frame_len);
static void Comm_Luban_SendResponse(uint8_t resp_id, const uint8_t *payload, uint16_t payload_len);

void Comm_Luban_Init(UART_HandleTypeDef *huart)
{
  s_uart = huart;
  s_rb_head = 0U;
  s_rb_tail = 0U;
  s_task_head = 0U;
  s_task_tail = 0U;

  if (s_uart != NULL)
  {
    HAL_UART_Receive_IT(s_uart, &s_rx_byte, 1U);
  }
}

bool Comm_Luban_GetNextTask(LubanTask *out)
{
  if (out == NULL)
  {
    return false;
  }

  if (s_task_head == s_task_tail)
  {
    return false;
  }

  *out = s_task_queue[s_task_tail];
  s_task_tail = (uint8_t)((s_task_tail + 1U) % LUBAN_TASK_QUEUE_SIZE);
  return true;
}

void Comm_Luban_Poll(void)
{
  while (Comm_Luban_RingAvailable() >= 7U)
  {
    uint8_t first = 0U;
    uint8_t second = 0U;

    if (!Comm_Luban_Peek(0U, &first) || !Comm_Luban_Peek(1U, &second))
    {
      return;
    }

    if (first != LUBAN_FRAME_HEADER_H || second != LUBAN_FRAME_HEADER_L)
    {
      uint8_t dummy;
      Comm_Luban_RingPop(&dummy);
      continue;
    }

    uint8_t cmd = 0U;
    uint8_t len_low = 0U;
    uint8_t len_high = 0U;
    Comm_Luban_Peek(2U, &cmd);
    Comm_Luban_Peek(3U, &len_low);
    Comm_Luban_Peek(4U, &len_high);

    uint16_t payload_len = (uint16_t)((((uint16_t)len_high) << 8U) | len_low);
    uint16_t total_len = (uint16_t)(2U + 1U + 2U + payload_len + 2U);

    if (total_len > LUBAN_RX_BUFFER_SIZE)
    {
      /* Frame too large for our buffer, drop the header byte and resync. */
      uint8_t dummy;
      Comm_Luban_RingPop(&dummy);
      continue;
    }

    if (Comm_Luban_RingAvailable() < total_len)
    {
      /* Wait for more data. */
      return;
    }

    uint8_t frame[LUBAN_RX_BUFFER_SIZE];
    for (uint16_t i = 0U; i < total_len; ++i)
    {
      Comm_Luban_Peek(i, &frame[i]);
    }

    uint16_t calculated_crc = Comm_Luban_CRC16(frame, (uint16_t)(total_len - 2U));
    uint16_t received_crc = (uint16_t)(((uint16_t)frame[total_len - 1U] << 8U) | frame[total_len - 2U]);

    if (calculated_crc != received_crc)
    {
      uint8_t dummy;
      Comm_Luban_RingPop(&dummy);
      continue;
    }

    Comm_Luban_HandleFrame(frame, total_len);
    Comm_Luban_Consume(total_len);
  }
}

void Comm_Luban_SendStatus(const char *status_str)
{
  const uint8_t *payload = (const uint8_t *)status_str;
  uint16_t len = 0U;

  if (status_str != NULL)
  {
    len = (uint16_t)strlen(status_str);
  }

  Comm_Luban_SendResponse((uint8_t)RESP_STATUS, payload, len);
}

/* HAL callback invoked when one byte is received over UART (non-blocking). */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart == s_uart)
  {
    Comm_Luban_RingPush(s_rx_byte);
    HAL_UART_Receive_IT(s_uart, &s_rx_byte, 1U);
  }
}

static void Comm_Luban_HandleFrame(const uint8_t *frame, uint16_t frame_len)
{
  if (frame_len < 7U)
  {
    return;
  }

  uint8_t cmd = frame[2];
  uint16_t payload_len = (uint16_t)(((uint16_t)frame[4] << 8U) | frame[3]);
  const uint8_t *payload = &frame[5];

  switch (cmd)
  {
  case CMD_SEND_TASK:
    if (payload_len >= 14U)
    {
      LubanTask task;
      memcpy(&task.task_id, &payload[0], sizeof(task.task_id));
      memcpy(&task.x, &payload[4], sizeof(task.x));
      memcpy(&task.y, &payload[8], sizeof(task.y));
      task.cleaning_type = payload[12];
      task.priority = payload[13];

      uint8_t next_head = (uint8_t)((s_task_head + 1U) % LUBAN_TASK_QUEUE_SIZE);
      if (next_head != s_task_tail)
      {
        s_task_queue[s_task_head] = task;
        s_task_head = next_head;
        Comm_Luban_SendResponse((uint8_t)RESP_TASK_ACK, (const uint8_t *)&task.task_id, sizeof(task.task_id));
      }
      else
      {
        /* Queue full */
        Comm_Luban_SendResponse((uint8_t)RESP_ERROR, (const uint8_t *)&cmd, 1U);
      }
    }
    else
    {
      Comm_Luban_SendResponse((uint8_t)RESP_ERROR, (const uint8_t *)&cmd, 1U);
    }
    break;

  case CMD_REQUEST_STATUS:
    Comm_Luban_SendStatus("OK");
    break;

  case CMD_CANCEL_TASK:
  case CMD_EMERGENCY_STOP:
    Comm_Luban_SendResponse((uint8_t)RESP_TASK_ACK, (const uint8_t *)&cmd, 1U);
    break;

  default:
    Comm_Luban_SendResponse((uint8_t)RESP_ERROR, (const uint8_t *)&cmd, 1U);
    break;
  }
}

static void Comm_Luban_SendResponse(uint8_t resp_id, const uint8_t *payload, uint16_t payload_len)
{
  if (s_uart == NULL)
  {
    return;
  }

  uint16_t total_len = (uint16_t)(2U + 1U + 2U + payload_len + 2U);
  if (total_len > LUBAN_RX_BUFFER_SIZE)
  {
    return;
  }

  uint8_t frame[LUBAN_RX_BUFFER_SIZE];
  frame[0] = LUBAN_FRAME_HEADER_H;
  frame[1] = LUBAN_FRAME_HEADER_L;
  frame[2] = resp_id;
  frame[3] = (uint8_t)(payload_len & 0xFFU);
  frame[4] = (uint8_t)((payload_len >> 8U) & 0xFFU);

  if (payload_len > 0U && payload != NULL)
  {
    memcpy(&frame[5], payload, payload_len);
  }

  uint16_t crc = Comm_Luban_CRC16(frame, (uint16_t)(total_len - 2U));
  frame[total_len - 2U] = (uint8_t)(crc & 0xFFU);
  frame[total_len - 1U] = (uint8_t)((crc >> 8U) & 0xFFU);

  HAL_UART_Transmit(s_uart, frame, total_len, COMM_UART_TIMEOUT);
}

static void Comm_Luban_RingPush(uint8_t byte)
{
  uint16_t next_head = (uint16_t)((s_rb_head + 1U) % LUBAN_RX_BUFFER_SIZE);
  if (next_head != s_rb_tail)
  {
    s_rx_buffer[s_rb_head] = byte;
    s_rb_head = next_head;
  }
  else
  {
    /* Buffer overflow, drop the oldest byte to make room */
    s_rb_tail = (uint16_t)((s_rb_tail + 1U) % LUBAN_RX_BUFFER_SIZE);
    s_rx_buffer[s_rb_head] = byte;
    s_rb_head = next_head;
  }
}

static bool Comm_Luban_RingPop(uint8_t *out)
{
  if (s_rb_head == s_rb_tail)
  {
    return false;
  }

  if (out != NULL)
  {
    *out = s_rx_buffer[s_rb_tail];
  }
  s_rb_tail = (uint16_t)((s_rb_tail + 1U) % LUBAN_RX_BUFFER_SIZE);
  return true;
}

static uint16_t Comm_Luban_RingAvailable(void)
{
  return (uint16_t)((s_rb_head + LUBAN_RX_BUFFER_SIZE - s_rb_tail) % LUBAN_RX_BUFFER_SIZE);
}

static bool Comm_Luban_Peek(uint16_t index, uint8_t *out)
{
  if (index >= Comm_Luban_RingAvailable())
  {
    return false;
  }

  uint16_t pos = (uint16_t)((s_rb_tail + index) % LUBAN_RX_BUFFER_SIZE);
  if (out != NULL)
  {
    *out = s_rx_buffer[pos];
  }
  return true;
}

static void Comm_Luban_Consume(uint16_t length)
{
  s_rb_tail = (uint16_t)((s_rb_tail + length) % LUBAN_RX_BUFFER_SIZE);
}

/* CRC16-IBM (polynomial 0xA001, initial value 0xFFFF) */
static uint16_t Comm_Luban_CRC16(const uint8_t *data, uint16_t length)
{
  uint16_t crc = 0xFFFFU;
  for (uint16_t i = 0U; i < length; ++i)
  {
    crc ^= data[i];
    for (uint8_t j = 0U; j < 8U; ++j)
    {
      if (crc & 0x0001U)
      {
        crc = (uint16_t)((crc >> 1U) ^ 0xA001U);
      }
      else
      {
        crc >>= 1U;
      }
    }
  }
  return crc;
}
