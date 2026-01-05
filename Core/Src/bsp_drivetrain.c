/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    bsp_drivetrain.c
  * @brief   Differential drivetrain control.
  ******************************************************************************
  */
/* USER CODE END Header */

#include "bsp_drivetrain.h"
#include "tim.h"
#include "gpio.h"

/* =========================
 *  TUNE HERE (可微调参数区)
 * ========================= */

/* 归一化输入限幅 */
#define DRIVETRAIN_MAX_NORM   1.0f
#define DRIVETRAIN_MIN_NORM  -1.0f

/* 方向控制引脚 */
#define DIR_LEFT_GPIO_Port   GPIOB
#define DIR_LEFT_Pin         GPIO_PIN_15
#define DIR_RIGHT_GPIO_Port  GPIOC
#define DIR_RIGHT_Pin        GPIO_PIN_3

/* 起转死区：eff>0 但太小会不动，就抬到这个值（0.2~0.45 常见） */
#define WHEEL_MIN_EFF        0.35f

/* 避免极限组合：DIR=1 且 PWM=0 或者 DIR=0 且 PWM=100% 造成奇怪抖动 */
#define WHEEL_MAX_EFF        0.95f

/* 左轮：车视角前进/后退力度缩放（对齐 main 当前标定） */
#define LEFT_EFF_SCALE_FWD   0.75f
#define LEFT_EFF_SCALE_BWD   0.80f

/* 右轮：车视角前进/后退力度缩放（对齐 main 当前标定） */
#define RIGHT_EFF_SCALE_FWD  0.83f
#define RIGHT_EFF_SCALE_BWD  0.55f

/*
 * 电机“旋向定义”与你在 main 里验证的定义一致：
 * MOTOR_CW  : DIR=0, PWM=eff
 * MOTOR_CCW : DIR=1, PWM=1-eff
 */
typedef enum
{
  MOTOR_CW  = 0,
  MOTOR_CCW = 1
} MotorDir_t;

/*
 * 车视角“前进”时，每个轮子应该用电机的哪个旋向
 * （按你目前验证结果：左=CCW，右=CW）
 */
#define LEFT_MOTOR_DIR_WHEN_VEH_FWD   MOTOR_CCW
#define RIGHT_MOTOR_DIR_WHEN_VEH_FWD  MOTOR_CW

/* =========================
 *  内部工具函数
 * ========================= */

static inline float clamp01(float x)
{
  if (x < 0.0f) return 0.0f;
  if (x > 1.0f) return 1.0f;
  return x;
}

static inline float clamp_norm(float v)
{
  if (v > DRIVETRAIN_MAX_NORM)  return DRIVETRAIN_MAX_NORM;
  if (v < DRIVETRAIN_MIN_NORM)  return DRIVETRAIN_MIN_NORM;
  return v;
}

static uint32_t duty_to_ccr(TIM_HandleTypeDef *htim, float duty_01)
{
  duty_01 = clamp01(duty_01);
  uint32_t arr = __HAL_TIM_GET_AUTORELOAD(htim);
  return (uint32_t)((float)arr * duty_01 + 0.5f);
}

/*
 * 关键：用你已验证成功的“两输入等效”方式驱动：
 * MOTOR_CW  : DIR=0, PWM=eff
 * MOTOR_CCW : DIR=1, PWM=1-eff
 */
static void Wheel_Set_ByMotorDir(TIM_HandleTypeDef *htim, uint32_t ch,
                                 GPIO_TypeDef *dir_port, uint16_t dir_pin,
                                 float eff_01, MotorDir_t mdir)
{
  eff_01 = clamp01(eff_01);

  if (eff_01 > 0.0f && eff_01 < WHEEL_MIN_EFF)
    eff_01 = WHEEL_MIN_EFF;

  if (eff_01 > WHEEL_MAX_EFF)
    eff_01 = WHEEL_MAX_EFF;

  if (eff_01 <= 0.0f)
  {
    HAL_GPIO_WritePin(dir_port, dir_pin, GPIO_PIN_RESET);
    __HAL_TIM_SET_COMPARE(htim, ch, 0U);
    return;
  }

  if (mdir == MOTOR_CW)
  {
    HAL_GPIO_WritePin(dir_port, dir_pin, GPIO_PIN_RESET);
    __HAL_TIM_SET_COMPARE(htim, ch, duty_to_ccr(htim, eff_01));
  }
  else
  {
    HAL_GPIO_WritePin(dir_port, dir_pin, GPIO_PIN_SET);
    __HAL_TIM_SET_COMPARE(htim, ch, duty_to_ccr(htim, 1.0f - eff_01));
  }
}

/* 车视角：左轮 norm -> 旋向 + 缩放 */
static void Left_Set_VehicleNorm(float norm)
{
  norm = clamp_norm(norm);

  const bool veh_forward = (norm >= 0.0f);
  float eff = (norm >= 0.0f) ? norm : -norm;   /* eff = |norm| */

  MotorDir_t dir = veh_forward ? LEFT_MOTOR_DIR_WHEN_VEH_FWD
                               : (LEFT_MOTOR_DIR_WHEN_VEH_FWD == MOTOR_CW ? MOTOR_CCW : MOTOR_CW);

  float scaled = eff * (veh_forward ? LEFT_EFF_SCALE_FWD : LEFT_EFF_SCALE_BWD);
  if (scaled > 1.0f) scaled = 1.0f;

  Wheel_Set_ByMotorDir(&htim8, TIM_CHANNEL_2,
                       DIR_LEFT_GPIO_Port, DIR_LEFT_Pin,
                       scaled, dir);
}

/* 车视角：右轮 norm -> 旋向 + 缩放 */
static void Right_Set_VehicleNorm(float norm)
{
  norm = clamp_norm(norm);

  const bool veh_forward = (norm >= 0.0f);
  float eff = (norm >= 0.0f) ? norm : -norm;

  MotorDir_t dir = veh_forward ? RIGHT_MOTOR_DIR_WHEN_VEH_FWD
                               : (RIGHT_MOTOR_DIR_WHEN_VEH_FWD == MOTOR_CW ? MOTOR_CCW : MOTOR_CW);

  float scaled = eff * (veh_forward ? RIGHT_EFF_SCALE_FWD : RIGHT_EFF_SCALE_BWD);
  if (scaled > 1.0f) scaled = 1.0f;

  Wheel_Set_ByMotorDir(&htim3, TIM_CHANNEL_2,
                       DIR_RIGHT_GPIO_Port, DIR_RIGHT_Pin,
                       scaled, dir);
}

/* =========================
 *  对外接口（保留 cmd_vel 解算）
 * ========================= */

void Drivetrain_Init(void)
{
  (void)HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_2);
  (void)HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);

  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull  = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;

  GPIO_InitStruct.Pin = DIR_LEFT_Pin;
  HAL_GPIO_Init(DIR_LEFT_GPIO_Port, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = DIR_RIGHT_Pin;
  HAL_GPIO_Init(DIR_RIGHT_GPIO_Port, &GPIO_InitStruct);

  Drivetrain_Stop();
}

void Drivetrain_Stop(void)
{
  HAL_GPIO_WritePin(DIR_LEFT_GPIO_Port, DIR_LEFT_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(DIR_RIGHT_GPIO_Port, DIR_RIGHT_Pin, GPIO_PIN_RESET);
  __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_2, 0U);
  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 0U);
}

/*
 * 这里是“左/右轮归一化 raw”入口：
 * 车视角 forward/back -> 电机 CW/CCW + 力度缩放
 */
void Drivetrain_SetRaw(float left_norm, float right_norm)
{
  Left_Set_VehicleNorm(left_norm);
  Right_Set_VehicleNorm(right_norm);
}

/* 这个接口如果你没用到，可以先不管；保留但语义不再建议使用 */
void Drivetrain_SetDirection(bool left_forward, bool right_forward)
{
  HAL_GPIO_WritePin(DIR_LEFT_GPIO_Port, DIR_LEFT_Pin,
                    left_forward ? GPIO_PIN_RESET : GPIO_PIN_SET);
  HAL_GPIO_WritePin(DIR_RIGHT_GPIO_Port, DIR_RIGHT_Pin,
                    right_forward ? GPIO_PIN_RESET : GPIO_PIN_SET);
}

/* ====== 下面保持你原来的 cmd_vel 解算部分不动 ====== */

float Drivetrain_UnitToNorm(float units)
{
  /* 对齐你当前前后标定：直接按归一化[-1,1]线性映射，避免正/负不对称 */
  float norm = units;
  if (norm > 1.0f)  norm = 1.0f;
  if (norm < -1.0f) norm = -1.0f;
  return norm;
}

void Drivetrain_SetUnits(float left_units, float right_units)
{
  Drivetrain_SetRaw(Drivetrain_UnitToNorm(left_units),
                    Drivetrain_UnitToNorm(right_units));
}

/* ======== cmd_vel -> norm 映射参数（根据上位机设定调整） ======== */
#define CMDVEL_VX_MAX   0.20f  /* 上位机线速度上限（m/s） */
#define CMDVEL_WZ_MAX   0.80f  /* 上位机角速度上限（rad/s） */
#define TWIST_K         1.0f   /* 转向混合增益，转弯太猛可调小 */

static inline float clampf(float x, float lo, float hi)
{
  if (x < lo) return lo;
  if (x > hi) return hi;
  return x;
}

void Drivetrain_SetTwist(float vx, float wz)
{
  /* 1) 物理量映射到归一化 [-1,1] */
  float v = (CMDVEL_VX_MAX > 1e-6f) ? (vx / CMDVEL_VX_MAX) : 0.0f;
  float w = (CMDVEL_WZ_MAX > 1e-6f) ? (wz / CMDVEL_WZ_MAX) : 0.0f;

  v = clampf(v, -1.0f, 1.0f);
  w = clampf(w, -1.0f, 1.0f);

  /* 2) 差速混合：w>0 -> 左慢右快（若方向反了，把 w 取反） */
  float left  = v - TWIST_K * w;
  float right = v + TWIST_K * w;

  /* 3) 统一归一化，避免一侧饱和另一侧被“吃掉” */
  float m = fabsf(left);
  if (fabsf(right) > m) m = fabsf(right);
  if (m > 1.0f)
  {
    left  /= m;
    right /= m;
  }

  Drivetrain_SetRaw(left, right);
}
