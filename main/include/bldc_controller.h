// main/include/bldc_controller.h
#pragma once
#include <stdbool.h>
#include <stdint.h>

// number of motors in your array above
#define MOTOR_COUNT  4

// call once at startup
void motor_control_init(void);

// control functions
void motor_control_set_enable(int id, bool on);
void motor_control_set_direction(int id, bool forward);
void motor_control_set_pwm(int id, uint8_t percent);

// optional: read back pulses (or hook your ISR)
uint32_t motor_control_get_pulses(int id);




// // main/include/bldc_controller.h
// #ifndef BLDC_CONTROLLER_H
// #define BLDC_CONTROLLER_H

// #include <stdint.h>
// #include <stdbool.h>

// #ifdef __cplusplus
// extern "C" {
// #endif

// #define MOTOR_COUNT 4

// // Initializes the BLDC motor driver, PWM, direction, enable pins, and pulse generator interrupts
// void motor_control_init(void);

// // Enable or disable the motor driver output for motor 'id'
// void motor_control_set_enable(int id, bool on);

// // Set direction (true=forward, false=reverse) for motor 'id'
// void motor_control_set_direction(int id, bool forward);

// // Set PWM duty cycle (0-100) percent for motor 'id'
// void motor_control_set_pwm(int id, uint8_t pct);

// // Get the net number of pulses (signed) for motor 'id'.
// // Positive = forward pulses, negative = reverse pulses.
// int32_t motor_control_get_pulses(int id);

// #ifdef __cplusplus
// }
// #endif

// #endif // BLDC_CONTROLLER_H
