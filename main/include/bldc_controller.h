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
