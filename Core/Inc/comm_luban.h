#ifndef COMM_LUBAN_H
#define COMM_LUBAN_H

#include <stdbool.h>
#include <stdint.h>
#include "main.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Forward declaration to allow use even when HAL_UART module is not enabled
 * in stm32f4xx_hal_conf.h. The concrete definition provided by the HAL will
 * satisfy this declaration when the UART driver is included. */
typedef struct __UART_HandleTypeDef UART_HandleTypeDef;

/* Basic HAL UART API forward declarations to avoid implicit prototypes when
 * stm32f4xx_hal_uart.h is not pulled in by stm32f4xx_hal_conf.h. */
HAL_StatusTypeDef HAL_UART_Receive_IT(UART_HandleTypeDef *huart, uint8_t *pData, uint16_t Size);
HAL_StatusTypeDef HAL_UART_Transmit(UART_HandleTypeDef *huart, uint8_t *pData, uint16_t Size, uint32_t Timeout);
void HAL_UART_IRQHandler(UART_HandleTypeDef *huart);

/* Command identifiers from the Luban protocol. */
typedef enum {
  CMD_SEND_TASK = 0x01,
  CMD_REQUEST_STATUS = 0x02,
  CMD_CANCEL_TASK = 0x03,
  CMD_EMERGENCY_STOP = 0x04
} LubanCmdId;

/* Response identifiers from the Luban protocol. */
typedef enum {
  RESP_TASK_ACK = 0x81,
  RESP_STATUS = 0x82,
  RESP_ERROR = 0xFF
} LubanRespId;

/* Task payload representation. */
typedef struct {
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
