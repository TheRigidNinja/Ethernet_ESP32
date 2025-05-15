#ifndef BLDC_PID_H
#define BLDC_PID_H

#include <stdint.h>
#include <stdbool.h>
#include "bldc_controller.h"  // for MOTOR_COUNT

// ── 1) Define your basic PID type first ────────────────────────────────
typedef struct {
    float kp, ki, kd;
    float integral_max, output_max;
} PID_config_t;

// ── 2) Configuration ────────────────────────────────────────────────────
#define CONTROL_PERIOD_MS 50

// ── 3) Per‐motor state for the two loops ────────────────────────────────
typedef struct {
    PID_config_t pid;        // position‐loop gains
    int32_t      target_pulses;
    float        integral;
    float        prev_error;
    bool         enabled;
} PosState_t;

typedef struct {
    PID_config_t pid;        
    float        target_rpm;
    float        integral;
    float        prev_error;
    int32_t      last_count;
    bool         enabled;
} SpeedState_t;

// ── 4) Extern declarations of your state arrays ────────────────────────
extern PosState_t   pos_states[MOTOR_COUNT];
extern SpeedState_t speed_states[MOTOR_COUNT];

// ── 5) Prototypes ──────────────────────────────────────────────────────
void motor_control_set_pid          (int id, PID_config_t pid);
void motor_control_set_target       (int id, int32_t target_pulses);
void motor_control_set_pwm_limit    (int id, uint8_t pct);
void motor_control_update_cascade_all(void);

#endif // BLDC_PID_H