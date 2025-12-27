/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    usart.c
  * @brief   USART configuration.
  ******************************************************************************
  */
/* USER CODE END Header */

#include "usart.h"

void HAL_UART_MspInit(UART_HandleTypeDef *huart);

UART_HandleTypeDef huart3;

static uint32_t uart_get_pclk(USART_TypeDef *instance)
{
  if (instance == USART1 || instance == USART6)
  {
    return HAL_RCC_GetPCLK2Freq();
  }
  return HAL_RCC_GetPCLK1Freq();
}

HAL_StatusTypeDef HAL_UART_Init(UART_HandleTypeDef *huart)
{
  uint32_t pclk;
  uint32_t brr;

  if (huart == NULL || huart->Instance == NULL)
  {
    return HAL_ERROR;
  }

  HAL_UART_MspInit(huart);

  /* Disable USART before configuration. */
  huart->Instance->CR1 &= ~USART_CR1_UE;

  /* Configure word length, parity, and mode. */
  huart->Instance->CR1 &= ~(USART_CR1_M | USART_CR1_PCE | USART_CR1_PS | USART_CR1_TE | USART_CR1_RE);
  huart->Instance->CR1 |= (huart->Init.WordLength | huart->Init.Parity | huart->Init.Mode);

  /* Configure stop bits. */
  huart->Instance->CR2 &= ~USART_CR2_STOP;
  huart->Instance->CR2 |= huart->Init.StopBits;

  /* No hardware flow control in this minimal driver. */
  huart->Instance->CR3 &= ~(USART_CR3_CTSE | USART_CR3_RTSE);

  /* Configure baud rate (oversampling by 16). */
  pclk = uart_get_pclk(huart->Instance);
  if (huart->Init.BaudRate == 0U)
  {
    return HAL_ERROR;
  }
  brr = (pclk + (huart->Init.BaudRate / 2U)) / huart->Init.BaudRate;
  huart->Instance->BRR = brr;

  /* Enable USART. */
  huart->Instance->CR1 |= USART_CR1_UE;

  return HAL_OK;
}

HAL_StatusTypeDef HAL_UART_Transmit(UART_HandleTypeDef *huart, uint8_t *pData, uint16_t Size, uint32_t Timeout)
{
  uint32_t tick_start;

  if (huart == NULL || pData == NULL || Size == 0U)
  {
    return HAL_ERROR;
  }

  tick_start = HAL_GetTick();
  for (uint16_t i = 0; i < Size; i++)
  {
    while ((huart->Instance->SR & USART_SR_TXE) == 0U)
    {
      if ((HAL_GetTick() - tick_start) > Timeout)
      {
        return HAL_TIMEOUT;
      }
    }
    huart->Instance->DR = pData[i];
  }

  while ((huart->Instance->SR & USART_SR_TC) == 0U)
  {
    if ((HAL_GetTick() - tick_start) > Timeout)
    {
      return HAL_TIMEOUT;
    }
  }

  return HAL_OK;
}

HAL_StatusTypeDef HAL_UART_Receive_IT(UART_HandleTypeDef *huart, uint8_t *pData, uint16_t Size)
{
  if (huart == NULL || pData == NULL || Size == 0U)
  {
    return HAL_ERROR;
  }

  huart->pRxBuffPtr = pData;
  huart->RxXferSize = Size;
  huart->RxXferCount = Size;

  huart->Instance->CR1 |= USART_CR1_RXNEIE;
  return HAL_OK;
}

void HAL_UART_IRQHandler(UART_HandleTypeDef *huart)
{
  uint32_t sr;

  if (huart == NULL)
  {
    return;
  }

  sr = huart->Instance->SR;
  if ((sr & USART_SR_RXNE) != 0U && (huart->Instance->CR1 & USART_CR1_RXNEIE) != 0U)
  {
    uint8_t data = (uint8_t)(huart->Instance->DR & 0xFFU);

    if (huart->RxXferCount > 0U && huart->pRxBuffPtr != NULL)
    {
      *huart->pRxBuffPtr++ = data;
      huart->RxXferCount--;
      if (huart->RxXferCount == 0U)
      {
        huart->Instance->CR1 &= ~USART_CR1_RXNEIE;
        HAL_UART_RxCpltCallback(huart);
      }
    }
  }
}

__weak void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  (void)huart;
}

void MX_USART3_UART_Init(void)
{
  /* 初始化 USART3（115200, 8N1），用于与 LubanCat 通信。 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
}
