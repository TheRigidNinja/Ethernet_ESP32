// bldc_pid.h
#ifndef BLDC_PID_H
#define BLDC_PID_H

#include <stdint.h>
#include <stdbool.h>

typedef struct {
    float kp;
    float ki;
    float kd;
    float integral_max;
    float output_max;
} PID_config_t;

void motor_control_set_pid(int id, PID_config_t config);
void motor_control_set_target(int id, int32_t target_pulses);
void motor_control_update_all();

#endif