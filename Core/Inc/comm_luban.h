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
#include "usart.h"

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

/* 新增速度控制命令 */
#define CMD_SET_TWIST  0x10

typedef struct
{
  uint32_t task_id;
  float x;
  float y;
  uint8_t cleaning_type;
  uint8_t priority;
} LubanTask;

void Comm_Luban_Init(UART_HandleTypeDef *huart); /* 初始化通信模块，绑定 UART 并启动中断接收。 */
void Comm_Luban_Poll(void); /* 主循环轮询解析 UART 帧，触发命令处理。 */
bool Comm_Luban_GetNextTask(LubanTask *out); /* 获取最新任务，若有新任务返回 true 并填充 out。 */
void Comm_Luban_SendStatus(const char *status_str); /* 发送状态字符串响应（RESP_STATUS）。 */
void Comm_Luban_Watchdog(void); /* 超过 300ms 未收到速度指令则刹车。 */
void Comm_UART1_TestInit(void); /* 启动 UART1 回显测试（验证链路）。 */

#ifdef __cplusplus
}
#endif

#endif /* COMM_LUBAN_H */
