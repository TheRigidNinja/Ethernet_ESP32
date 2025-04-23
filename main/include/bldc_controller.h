#pragma once
#include <stdbool.h>
#include <stdint.h>

// call once at startup to configure LEDC + GPIO + pulse‐ISR
void bldc_init(void);

// control interface:
void bldc_set_enable(bool on);
void bldc_set_direction(bool forward);
void bldc_set_pwm_percent(uint8_t pct);
uint32_t bldc_get_pulse_count(void);
