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

void Cleaning_Init(void); /* 初始化清扫 GPIO，并默认关闭输出。 */
void Cleaning_SetBrush(bool on); /* 同时控制左右刷子 MOS 开关。 */
void Cleaning_SetBrushLeft(bool on); /* 控制左刷子 MOS 开关（PD12）。 */
void Cleaning_SetBrushRight(bool on); /* 控制右刷子 MOS 开关（PA2）。 */
void Cleaning_SetPump(bool on); /* 控制水泵 MOS 开关（PB4）。 */
void Cleaning_SetFan(bool on); /* 控制风机 GPIO 开关。 */

#ifdef __cplusplus
}
#endif

#endif /* BSP_CLEANING_H */
