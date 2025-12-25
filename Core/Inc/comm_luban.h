/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    comm_luban.h
  * @brief   LubanCat UART protocol handler.
  ******************************************************************************
  */
/* USER CODE END Header */
#ifndef COMM_LUBAN_H
#define COMM_LUBAN_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>
#include "stm32f4xx_hal.h"

typedef enum
{
  CMD_SEND_TASK = 0x01,
  CMD_REQUEST_STATUS = 0x02,
  CMD_CANCEL_TASK = 0x03,
  CMD_EMERGENCY_STOP = 0x04
} LubanCmdId;

typedef enum
{
  RESP_TASK_ACK = 0x81,
  RESP_STATUS = 0x82,
  RESP_ERROR = 0xFF
} LubanRespId;

typedef struct
{
  uint32_t task_id;
  float x;
  float y;
  uint8_t cleaning_type;
  uint8_t priority;
} LubanTask;

void Comm_Luban_Init(UART_HandleTypeDef *huart);
void Comm_Luban_Poll(void);
bool Comm_Luban_GetNextTask(LubanTask *out);
void Comm_Luban_SendStatus(const char *status_str);

#ifdef __cplusplus
}
#endif

#endif /* COMM_LUBAN_H */
