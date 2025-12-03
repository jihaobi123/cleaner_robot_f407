#include "bsp_sensors.h"

#include "gpio.h"
#include "main.h"

static volatile SensorState sensor_events = {0};
static SensorState latched_state = {0};

void Sensors_Init(void)
{
    /* GPIOs are already configured in MX_GPIO_Init. */
}

void Sensors_Poll(void)
{
    /* Merge new events into the latched state so they can be consumed later. */
    if (sensor_events.bump_left) {
        latched_state.bump_left = true;
    }
    if (sensor_events.bump_right) {
        latched_state.bump_right = true;
    }
    if (sensor_events.cliff_left) {
        latched_state.cliff_left = true;
    }
    if (sensor_events.cliff_right) {
        latched_state.cliff_right = true;
    }
    if (sensor_events.cliff_front) {
        latched_state.cliff_front = true;
    }
}

SensorState Sensors_GetState(void)
{
    return latched_state;
}

void Sensors_ClearEvents(void)
{
    sensor_events = (SensorState){0};
    latched_state = (SensorState){0};
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    switch (GPIO_Pin) {
    case GPIO_PIN_4:
        sensor_events.bump_left = true;
        break;
    case GPIO_PIN_5:
        sensor_events.bump_right = true;
        break;
    case GPIO_PIN_7:
        sensor_events.cliff_front = true;
        break;
    case GPIO_PIN_0:
        sensor_events.cliff_left = true;
        break;
    case GPIO_PIN_12:
        sensor_events.cliff_right = true;
        break;
    default:
        break;
    }
}
