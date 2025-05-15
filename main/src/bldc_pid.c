#include "bldc_controller.h"
#include <math.h>
#include "bldc_pid.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"

static const char *TAG = "bldc_pid";

// Tunables
#define POLE_PAIRS 2                    // 4-pole motor => 2 pole-pairs
#define PULSES_PER_REV (POLE_PAIRS * 6) // PG pulses per electrical rev × pole-pairs
#define MAX_RPM 4000.0f                 // no-load max RPM
#define POS_DEADBAND 5.0f               // ±pulses
#define SPD_DEADBAND 1.0f               // ±RPM

PosState_t pos_states[MOTOR_COUNT] = {0};
SpeedState_t speed_states[MOTOR_COUNT] = {0};
static uint8_t pwm_limit[MOTOR_COUNT] = {0};

void motor_control_set_pid(int id, PID_config_t pid)
{
    if (id < 0 || id >= MOTOR_COUNT)
        return;
    pos_states[id].pid = pid;
    speed_states[id].pid = pid;

    ESP_LOGI(TAG,
             "Motor %d PID set → Kp=%.3f  Ki=%.3f  Kd=%.3f  Imax=%.1f  Omax=%.1f",
             id, pid.kp, pid.ki, pid.kd, pid.integral_max, pid.output_max);
}

void motor_control_set_target(int id, int32_t target_pulses)
{
    if (id < 0 || id >= MOTOR_COUNT)
        return;
    pos_states[id].target_pulses = target_pulses;
    pos_states[id].enabled = true;
    pos_states[id].integral = 0;
    pos_states[id].prev_error = 0;
}

void motor_control_set_pwm_limit(int id, uint8_t pct)
{
    if (id < 0 || id >= MOTOR_COUNT)
        return;
    pwm_limit[id] = pct;
}

void motor_control_update_cascade_all(void)
{
    const float dt = CONTROL_PERIOD_MS / 1000.0f;
    const float rpm_fac = 60.0f / (PULSES_PER_REV * dt);

    for (int id = 0; id < MOTOR_COUNT; id++)
    {
        PosState_t *ps = &pos_states[id];
        if (!ps->enabled)
        {
            // not armed → do nothing
            continue;
        }

        // 1) sample once
        int32_t cur = motor_control_get_pulses(id);

        // 2) Position error → speed setpoint
        float pos_err = (float)(pos_states->target_pulses - cur);
        // ESP_LOGI(TAG, "M%d: cur=%ld tgt=%ld pos_err=%.2f",
        //          id, (long)cur, (long)pos_states->target_pulses, pos_err);

        // if within position deadband, shut everything off
        if (fabsf(pos_err) <= POS_DEADBAND)
        {
            // ESP_LOGI(TAG, "  ✔ pos within %.1f, disabling motor", POS_DEADBAND);
            motor_control_set_pwm(id, 0);
            motor_control_set_enable(id, false);
            pos_states->enabled = false;
            speed_states->target_rpm = 0.0f;
            continue;
        }

        // map position error → P-only speed percentage, capped by pwm_limit
        {
            float raw_pct = pos_states->pid.kp * pos_err; // can be >100 or <–100
            float pct = fmaxf(fminf(raw_pct, 100.0f), -100.0f);
            pct = fmaxf(fminf(pct, pwm_limit[id]), -pwm_limit[id]);
            speed_states->target_rpm = pct / 100.0f * MAX_RPM;
        }

        // 3) Speed PID
        if (speed_states->target_rpm != 0.0f)
        {
            int32_t delta = cur - speed_states->last_count;
            speed_states->last_count = cur;
            float actual_rpm = delta * rpm_fac;
            float spd_err = speed_states->target_rpm - actual_rpm;

            // P term
            float P = speed_states->pid.kp * spd_err;
            // I term
            speed_states->integral += speed_states->pid.ki * spd_err * dt;
            speed_states->integral = fmaxf(fminf(speed_states->integral, speed_states->pid.integral_max),
                                           -speed_states->pid.integral_max);
            // D term
            float D = speed_states->pid.kd * (spd_err - speed_states->prev_error) / dt;
            speed_states->prev_error = spd_err;

            // combine
            float u = P + speed_states->integral + D;
            u = fmaxf(fminf(u, speed_states->pid.output_max), -speed_states->pid.output_max);

            bool forward = (u >= 0);
            uint8_t duty = (uint8_t)fminf(fabsf(u), 100.0f);
            motor_control_set_direction(id, forward);
            motor_control_set_pwm(id, duty);

            // 4) Speed deadband → we’re done
            if (fabsf(spd_err) <= SPD_DEADBAND)
            {
                // ESP_LOGI(TAG, "  ✔ speed within %.1f, disabling motor", SPD_DEADBAND);
                motor_control_set_pwm(id, 0);
                motor_control_set_enable(id, false);
                pos_states->enabled = false;
                speed_states->target_rpm = 0.0f;
            }
        }
    }
}
