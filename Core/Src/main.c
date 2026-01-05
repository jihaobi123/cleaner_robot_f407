/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "comm_luban.h"
#include "bsp_drivetrain.h"
#include "bsp_cleaning.h"
#include "bsp_sensors.h"
#include "robot_state_machine.h"
#include "usart.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define ENABLE_LOCAL_DRIVE_DEMO   0   /* set 1 to auto-run forward/back/left/right */
#define DEMO_LINEAR_SPEED         0.6f
#define DEMO_TURN_SPEED           0.5f

/* Direct PWM+DIR wheel tests (bypass abstraction) */
#define ENABLE_RAW_WHEEL_TEST     0   /* set 1 to run per-wheel forward/reverse on boot */
#define RAW_TEST_EFF              0.70f
#define ENABLE_FORWARD_SMOKE      0   /* set 1 to drive both wheels forward briefly after init */
#define ENABLE_BACKWARD_SMOKE     0   /* set 1 to drive both wheels backward briefly after init */
#define FORWARD_TEST_NORM         0.70f
#define BACKWARD_TEST_NORM        -0.70f
#define SMOKE_TEST_TIME_MS        4000U
/* 固定 cmd_vel 冒烟测试：1 次性发送 vx/wz 以验证解算 */
#define ENABLE_CMDVEL_SMOKE       0
#define CMDVEL_SMOKE_VX           0.4f
#define CMDVEL_SMOKE_WZ           0.0f
#define CMDVEL_SMOKE_TIME_MS      4000U

/* 直接双轮前进冒烟（绕开解算，直接发左右相同 norm） */
#define ENABLE_DIRECT_FWD_SMOKE   0
#define DIRECT_FWD_NORM           0.70f
#define DIRECT_FWD_TIME_MS        4000U

/* Wheel DIR pins (duplicated here for raw test convenience) */
#define DIR_LEFT_GPIO_Port   GPIOB
#define DIR_LEFT_Pin         GPIO_PIN_15
#define DIR_RIGHT_GPIO_Port  GPIOC
#define DIR_RIGHT_Pin        GPIO_PIN_3

/* 方向与起转参数 */
#define WHEEL_MIN_EFF        0.45f
#define WHEEL_KICK_MS        180U
#define LEFT_EFF_SCALE_FWD   0.55f
#define LEFT_EFF_SCALE_BWD   0.80f
#define LEFT_KICK_FWD_EFF    0.65f
#define LEFT_KICK_BWD_EFF    0.85f
#define RIGHT_EFF_SCALE_FWD  0.90f
#define RIGHT_EFF_SCALE_BWD  0.70f
#define RIGHT_KICK_FWD_EFF   0.85f
#define RIGHT_KICK_BWD_EFF   0.65f

typedef enum
{
  MOTOR_CW = 0,   /* DIR=0, PWM=eff */
  MOTOR_CCW = 1   /* DIR=1, PWM=1-eff */
} MotorDir_t;

/* 车前进时各轮电机旋向（按实际安装设置） */
#define LEFT_MOTOR_DIR_WHEN_VEH_FWD   MOTOR_CCW
#define RIGHT_MOTOR_DIR_WHEN_VEH_FWD  MOTOR_CW

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
static void Drive_Forward(float speed_norm);
static void Drive_Backward(float speed_norm);
static void Drive_TurnLeft(float turn_norm);
static void Drive_TurnRight(float turn_norm);
static void Demo_LocalMotion(void);
static void Raw_Wheel_Test(void);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
static void Drive_Forward(float speed_norm)
{
  Drivetrain_SetTwist(speed_norm, 0.0f);
}

static void Drive_Backward(float speed_norm)
{
  Drivetrain_SetTwist(-speed_norm, 0.0f);
}

static void Drive_TurnLeft(float turn_norm)
{
  Drivetrain_SetTwist(0.0f, turn_norm); /* 正角速度：左?*/
}

static void Drive_TurnRight(float turn_norm)
{
  Drivetrain_SetTwist(0.0f, -turn_norm); /* 负角速度：右?*/
}

/* 简单本地演示：?>?>左转->右转，然后停 */
static void Demo_LocalMotion(void)
{
  Drive_Forward(DEMO_LINEAR_SPEED);
  HAL_Delay(1500);

  Drive_Backward(DEMO_LINEAR_SPEED);
  HAL_Delay(1500);

  Drive_TurnLeft(DEMO_TURN_SPEED);
  HAL_Delay(1200);

  Drive_TurnRight(DEMO_TURN_SPEED);
  HAL_Delay(1200);

  Drivetrain_Stop();
  HAL_Delay(500);
}

static inline float clamp01(float x)
{
  if (x < 0.0f) return 0.0f;
  if (x > 1.0f) return 1.0f;
  return x;
}

static uint32_t duty_to_ccr_local(TIM_HandleTypeDef *htim, float duty_01)
{
  duty_01 = clamp01(duty_01);
  uint32_t arr = __HAL_TIM_GET_AUTORELOAD(htim);
  return (uint32_t)((float)arr * duty_01 + 0.5f);
}

/**
 * 双输入（应为双 PWM）但 DIR 只能 0/100% 的电机控制：
 *  forward : DIR=0%，PWM=duty
 *  reverse : DIR=100%，PWM=1-duty  => DIR 路占空比更高，方向翻转且力度对称
 */
static void Wheel_Set_ByMotorDir(TIM_HandleTypeDef *htim, uint32_t ch,
                                 GPIO_TypeDef *dir_port, uint16_t dir_pin,
                                 float eff_01, MotorDir_t mdir)
{
  eff_01 = clamp01(eff_01);

  /* 起转死区：确保有效力度不小于阈值 */
  if (eff_01 > 0.0f && eff_01 < WHEEL_MIN_EFF)
  {
    eff_01 = WHEEL_MIN_EFF;
  }

  /* 限幅，避免 100% vs 0% 极端组合 */
  if (eff_01 > 0.95f) eff_01 = 0.95f;

  if (eff_01 <= 0.0f)
  {
    HAL_GPIO_WritePin(dir_port, dir_pin, GPIO_PIN_RESET); /* DIR=0% */
    __HAL_TIM_SET_COMPARE(htim, ch, 0U);                  /* PWM=0% */
    return;
  }

  if (mdir == MOTOR_CW)
  {
    HAL_GPIO_WritePin(dir_port, dir_pin, GPIO_PIN_RESET); /* DIR=0% */
    __HAL_TIM_SET_COMPARE(htim, ch, duty_to_ccr_local(htim, eff_01));
  }
  else
  {
    HAL_GPIO_WritePin(dir_port, dir_pin, GPIO_PIN_SET);   /* DIR=100% */
    __HAL_TIM_SET_COMPARE(htim, ch, duty_to_ccr_local(htim, 1.0f - eff_01));
  }
}

/* 左/右轮封装：车视角前进/后退 */
static void Left_Set_Vehicle(bool veh_forward, float eff_01)
{
  MotorDir_t dir = veh_forward ? LEFT_MOTOR_DIR_WHEN_VEH_FWD
                               : (LEFT_MOTOR_DIR_WHEN_VEH_FWD == MOTOR_CW ? MOTOR_CCW : MOTOR_CW);
  float scaled = eff_01 * (veh_forward ? LEFT_EFF_SCALE_FWD : LEFT_EFF_SCALE_BWD);
  if (scaled > 1.0f) scaled = 1.0f;
  Wheel_Set_ByMotorDir(&htim8, TIM_CHANNEL_2, DIR_LEFT_GPIO_Port, DIR_LEFT_Pin, scaled, dir);
}

static void Right_Set_Vehicle(bool veh_forward, float eff_01)
{
  MotorDir_t dir = veh_forward ? RIGHT_MOTOR_DIR_WHEN_VEH_FWD
                               : (RIGHT_MOTOR_DIR_WHEN_VEH_FWD == MOTOR_CW ? MOTOR_CCW : MOTOR_CW);
  float scaled = eff_01 * (veh_forward ? RIGHT_EFF_SCALE_FWD : RIGHT_EFF_SCALE_BWD);
  if (scaled > 1.0f) scaled = 1.0f;
  Wheel_Set_ByMotorDir(&htim3, TIM_CHANNEL_2, DIR_RIGHT_GPIO_Port, DIR_RIGHT_Pin, scaled, dir);
}

static void Wheels_Stop_All(void)
{
  HAL_GPIO_WritePin(DIR_LEFT_GPIO_Port,  DIR_LEFT_Pin,  GPIO_PIN_RESET);
  HAL_GPIO_WritePin(DIR_RIGHT_GPIO_Port, DIR_RIGHT_Pin, GPIO_PIN_RESET);
  __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_2, 0U);
  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 0U);
}

static void Wheel_Run_WithKick_Vehicle(bool left, bool veh_forward, float eff_01)
{
  eff_01 = clamp01(eff_01);

  if (left)
  {
    const float kick = veh_forward ? LEFT_KICK_FWD_EFF : LEFT_KICK_BWD_EFF;
    Left_Set_Vehicle(veh_forward, kick);
    HAL_Delay(WHEEL_KICK_MS);
    Left_Set_Vehicle(veh_forward, eff_01);
  }
  else
  {
    const float kick = veh_forward ? RIGHT_KICK_FWD_EFF : RIGHT_KICK_BWD_EFF;
    Right_Set_Vehicle(veh_forward, kick);
    HAL_Delay(WHEEL_KICK_MS);
    Right_Set_Vehicle(veh_forward, eff_01);
  }
}

/* 只测右轮：直接测 MOTOR_CW / MOTOR_CCW 各跑一段，观察哪个是“车前进” */
static void RightWheel_DirProbe(void)
{
  const float eff = RAW_TEST_EFF;

  Wheels_Stop_All();
  HAL_Delay(800);

  /* A) 右轮 MOTOR_CW */
  Wheel_Set_ByMotorDir(&htim3, TIM_CHANNEL_2,
                       DIR_RIGHT_GPIO_Port, DIR_RIGHT_Pin,
                       eff, MOTOR_CW);
  HAL_Delay(2000);

  Wheels_Stop_All();
  HAL_Delay(1200);

  /* B) 右轮 MOTOR_CCW */
  Wheel_Set_ByMotorDir(&htim3, TIM_CHANNEL_2,
                       DIR_RIGHT_GPIO_Port, DIR_RIGHT_Pin,
                       eff, MOTOR_CCW);
  HAL_Delay(2000);

  Wheels_Stop_All();
  HAL_Delay(1500);
}

/* 开机动作测试：左正/左反/右正/右反/双前/双后/原地左/原地右/停 */
static void Raw_Wheel_Test(void)
{
  RightWheel_DirProbe();
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_TIM8_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
  MX_USART3_UART_Init();
  Drivetrain_Init();
  Cleaning_Init();
  Sensors_Init();
  Comm_Luban_Init(&huart3);
  Comm_UART1_TestInit();
  RobotSM_Init();
#if ENABLE_FORWARD_SMOKE
  /* 简单验证：双轮前进一段时间 */
  Drivetrain_SetRaw(FORWARD_TEST_NORM, FORWARD_TEST_NORM);
  HAL_Delay(SMOKE_TEST_TIME_MS);
  Drivetrain_Stop();
#endif
#if ENABLE_BACKWARD_SMOKE
  /* 简单验证：双轮后退一段时间 */
  Drivetrain_SetRaw(BACKWARD_TEST_NORM, BACKWARD_TEST_NORM);
  HAL_Delay(SMOKE_TEST_TIME_MS);
  Drivetrain_Stop();
#endif
#if ENABLE_DIRECT_FWD_SMOKE
  /* 绕开解算，直接控制左右相同 norm 前进 */
  Drivetrain_SetRaw(DIRECT_FWD_NORM, DIRECT_FWD_NORM);
  HAL_Delay(DIRECT_FWD_TIME_MS);
  Drivetrain_Stop();
#endif
#if ENABLE_CMDVEL_SMOKE
  /* 固定 cmd_vel 冒烟：vx/wz 测解算与左右轮输出 */
  Drivetrain_SetTwist(CMDVEL_SMOKE_VX, CMDVEL_SMOKE_WZ);
  HAL_Delay(CMDVEL_SMOKE_TIME_MS);
  Drivetrain_Stop();
#endif
#if ENABLE_RAW_WHEEL_TEST
  Raw_Wheel_Test();
#endif
#if ENABLE_LOCAL_DRIVE_DEMO
  Demo_LocalMotion();
#endif

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    /* 周期通过 UART3 发送心跳，验证上行链路 */
   /* static uint32_t uart3_tx_tick = 0;
    if ((HAL_GetTick() - uart3_tx_tick) > 1000U)
    {
      uart3_tx_tick = HAL_GetTick();
      const char hb3[] = "UART3_PING\r\n";
      (void)HAL_UART_Transmit(&huart3, (uint8_t *)hb3, (uint16_t)(sizeof(hb3) - 1U), 20);
    }*/

    Comm_Luban_Poll();
    Comm_Luban_Watchdog();
    /* USER CODE END 3 */
  }
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */



