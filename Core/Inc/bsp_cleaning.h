#ifndef __BSP_CLEANING_H
#define __BSP_CLEANING_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>

/* Initialize cleaning subsystem PWM outputs and fan control. */
void Cleaning_Init(void);

/* Control the brush motor PWM output. */
void Cleaning_SetBrush(bool on);

/* Control the water pump PWM output. */
void Cleaning_SetPump(bool on);

/* Control the suction fan on/off GPIO. */
void Cleaning_SetFan(bool on);

#ifdef __cplusplus
}
#endif

#endif /* __BSP_CLEANING_H */
