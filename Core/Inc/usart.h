/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    usart.h
  * @brief   USART configuration.
  ******************************************************************************
  */
/* USER CODE END Header */
#ifndef __USART_H__
#define __USART_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

typedef struct
{
  uint32_t BaudRate;
  uint32_t WordLength;
  uint32_t StopBits;
  uint32_t Parity;
  uint32_t Mode;
  uint32_t HwFlowCtl;
  uint32_t OverSampling;
} UART_InitTypeDef;

typedef struct
{
  USART_TypeDef *Instance;
  UART_InitTypeDef Init;
  uint8_t *pRxBuffPtr;
  uint16_t RxXferSize;
  uint16_t RxXferCount;
} UART_HandleTypeDef;

/* Minimal UART configuration macros (覆盖本工程使用的配置). */
#define UART_WORDLENGTH_8B        0x00000000U
#define UART_STOPBITS_1           0x00000000U
#define UART_PARITY_NONE          0x00000000U
#define UART_MODE_RX              USART_CR1_RE
#define UART_MODE_TX              USART_CR1_TE
#define UART_MODE_TX_RX           (USART_CR1_TE | USART_CR1_RE)
#define UART_HWCONTROL_NONE       0x00000000U
#define UART_OVERSAMPLING_16      0x00000000U

extern UART_HandleTypeDef huart3;

HAL_StatusTypeDef HAL_UART_Init(UART_HandleTypeDef *huart); /* 初始化 UART 外设寄存器并使能。 */
HAL_StatusTypeDef HAL_UART_Transmit(UART_HandleTypeDef *huart, uint8_t *pData, uint16_t Size, uint32_t Timeout); /* 阻塞发送。 */
HAL_StatusTypeDef HAL_UART_Receive_IT(UART_HandleTypeDef *huart, uint8_t *pData, uint16_t Size); /* 使能 RX 中断接收。 */
void HAL_UART_IRQHandler(UART_HandleTypeDef *huart); /* UART 中断处理（仅处理 RXNE）。 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart); /* 接收完成回调（可由用户实现）。 */

void MX_USART3_UART_Init(void); /* 初始化 USART3，供 LubanCat 通信使用。 */

#ifdef __cplusplus
}
#endif

#endif /* __USART_H__ */
