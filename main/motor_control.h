#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H

#include <stdint.h>
#include <stdbool.h>
#include "driver/gpio.h"

#define MOTOR_COUNT 4

typedef struct {
    gpio_num_t pwm_gpio;    // PWM → SV
    gpio_num_t dir_gpio;    // F/R
    gpio_num_t en_gpio;     // EN
    gpio_num_t pulse_gpio;  // PG
} MotorPins_t;

// initialize all 4 motors: PWM timers, GPIOs, ISRs
void motor_control_init_all(void);

// control API
void    motor_control_set_enable(int idx, bool en);
void    motor_control_set_direction(int idx, bool forward);
void    motor_control_set_pwm(int idx, uint8_t pct);  // pct = 0–100%
uint32_t motor_control_get_pulses(int idx);

#endif // MOTOR_CONTROL_H
