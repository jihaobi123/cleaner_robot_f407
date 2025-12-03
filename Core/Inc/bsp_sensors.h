#ifndef BSP_SENSORS_H
#define BSP_SENSORS_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    bool bump_left;
    bool bump_right;
    bool cliff_left;
    bool cliff_right;
    bool cliff_front;
} SensorState;

void Sensors_Init(void);
void Sensors_Poll(void);
SensorState Sensors_GetState(void);
void Sensors_ClearEvents(void);

#ifdef __cplusplus
}
#endif

#endif /* BSP_SENSORS_H */
