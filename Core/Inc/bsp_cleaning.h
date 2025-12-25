/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    bsp_cleaning.h
  * @brief   Cleaning subsystem control.
  ******************************************************************************
  */
/* USER CODE END Header */
#ifndef BSP_CLEANING_H
#define BSP_CLEANING_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>

void Cleaning_Init(void);
void Cleaning_SetBrush(bool on);
void Cleaning_SetPump(bool on);
void Cleaning_SetFan(bool on);

#ifdef __cplusplus
}
#endif

#endif /* BSP_CLEANING_H */
