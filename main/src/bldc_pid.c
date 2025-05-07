// bldc_pid.c
#include "bldc_pid.h"
#include "bldc_controller.h"
#include <math.h>

typedef struct {
    PID_config_t pid;
    int32_t target_pulses;
    float integral;
    float prev_error;
    bool enabled;
} MotorControlState_t;

static MotorControlState_t motor_states[MOTOR_COUNT] = {0};

void motor_control_set_pid(int id, PID_config_t config) {
    if (id < 0 || id >= MOTOR_COUNT) return;
    motor_states[id].pid = config;
}

void motor_control_set_target(int id, int32_t target_pulses) {
    if (id < 0 || id >= MOTOR_COUNT) return;
    motor_states[id].target_pulses = target_pulses;
    motor_states[id].enabled = true;
    motor_states[id].integral = 0;  // Reset integral term when setting new target
}

void motor_control_update_all() {
    for (int id = 0; id < MOTOR_COUNT; id++) {
        if (!motor_states[id].enabled) continue;

        int32_t current = motor_control_get_pulses(id);
        int32_t target = motor_states[id].target_pulses;
        float error = (float)(target - current);

        // PID calculation
        MotorControlState_t *state = &motor_states[id];
        PID_config_t *pid = &state->pid;

        // Proportional term
        float p = pid->kp * error;

        // Integral term with anti-windup
        state->integral += pid->ki * error;
        state->integral = fmaxf(fminf(state->integral, pid->integral_max), -pid->integral_max);

        // Derivative term
        float d = pid->kd * (error - state->prev_error);
        state->prev_error = error;

        // Calculate output
        float output = p + state->integral + d;
        output = fmaxf(fminf(output, pid->output_max), -pid->output_max);

        // Apply to motor
        if (fabsf(error) > 5) {  // Deadband of ±5 pulses
            bool direction = (output > 0);
            uint8_t pwm = (uint8_t)fminf(fabsf(output), 100);
            
            motor_control_set_direction(id, direction);
            motor_control_set_pwm(id, pwm);
        } else {
            motor_control_set_pwm(id, 0);
            motor_control_set_enable(id, false);
            state->enabled = false;  // Disable control when reached target
        }
    }
}