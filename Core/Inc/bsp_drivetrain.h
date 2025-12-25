#ifndef __BSP_DRIVETRAIN_H
#define __BSP_DRIVETRAIN_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>

/* Initialize drivetrain PWM outputs. */
void Drivetrain_Init(void);

/* Set normalized wheel speeds in range [-1.0, 1.0]. */
void Drivetrain_SetRaw(float left_norm, float right_norm);

/* Immediately stop both wheels. */
void Drivetrain_Stop(void);

#ifdef __cplusplus
}
#endif

#endif /* __BSP_DRIVETRAIN_H */
