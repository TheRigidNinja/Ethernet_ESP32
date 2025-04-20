// #include <stdio.h>
// #include "freertos/FreeRTOS.h"
// #include "freertos/task.h"
// #include "driver/gpio.h"
// #include "driver/ledc.h"
// #include "esp_log.h"
// #include <inttypes.h>
// #include "sdkconfig.h"
// #include "freertos/FreeRTOS.h"
// #include "freertos/task.h"

// static const char *TAG = "motor_ctrl";

// #define MOTOR_COUNT       4
// #define LEDC_TIMER        LEDC_TIMER_0
// #define LEDC_MODE         LEDC_HIGH_SPEED_MODE
// #define LEDC_DUTY_RES     LEDC_TIMER_10_BIT
// #define LEDC_FREQ_HZ      20000

// typedef enum { FORWARD = 1, REVERSE = 0 } direction_t;

// // pin mapping for motors 0–3
// static const struct {
//     gpio_num_t pwm, dir, en, pulse;
// } motors[MOTOR_COUNT] = {
//     { .pwm = 32, .dir = 17, .en =  5, .pulse = 39 },
//     { .pwm = 33, .dir = 16, .en = 18, .pulse = 36 },
//     { .pwm = 25, .dir = 27, .en = 19, .pulse = 35 },
//     { .pwm = 26, .dir = 14, .en = 21, .pulse = 34 }
// };

// static volatile uint32_t pulse_count[MOTOR_COUNT] = {0};

// // ISR: count rising edges
// static void IRAM_ATTR pulse_isr(void *arg) {
//     int idx = (int)arg;
//     pulse_count[idx]++;
// }

// // Initialize all motor GPIOs + PWM + pulse ISRs
// void motor_ctrl_init(void) {
//     // 1) Configure LEDC timer
//     ledc_timer_config_t timer = {
//         .speed_mode      = LEDC_MODE,
//         .duty_resolution = LEDC_DUTY_RES,
//         .timer_num       = LEDC_TIMER,
//         .freq_hz         = LEDC_FREQ_HZ,
//         .clk_cfg         = LEDC_AUTO_CLK
//     };
//     ledc_timer_config(&timer);

//     // 2) Prepare dir+en pins
//     gpio_config_t io_conf = {
//         .mode        = GPIO_MODE_OUTPUT,
//         .pin_bit_mask= 0
//     };
//     // we'll OR in later

//     gpio_install_isr_service(0);

//     for (int i = 0; i < MOTOR_COUNT; i++) {
//         // PWM channel
//         ledc_channel_config_t ch = {
//             .gpio_num   = motors[i].pwm,
//             .speed_mode = LEDC_MODE,
//             .channel    = i,
//             .intr_type  = LEDC_INTR_DISABLE,
//             .timer_sel  = LEDC_TIMER,
//             .duty       = 0,
//             .hpoint     = 0
//         };
//         ledc_channel_config(&ch);

//         // mark DIR & EN
//         io_conf.pin_bit_mask |= (1ULL<<motors[i].dir) | (1ULL<<motors[i].en);

//         // setup pulse input + ISR
//         gpio_config_t pulse_conf = {
//             .mode         = GPIO_MODE_INPUT,
//             .pin_bit_mask = (1ULL<<motors[i].pulse),
//             .pull_up_en   = GPIO_PULLUP_DISABLE,
//             .pull_down_en = GPIO_PULLDOWN_DISABLE,
//             .intr_type    = GPIO_INTR_POSEDGE
//         };
//         gpio_config(&pulse_conf);
//         gpio_isr_handler_add(motors[i].pulse, pulse_isr, (void*)i);
//     }
//     // apply DIR+EN config
//     gpio_config(&io_conf);
//     for (int i = 0; i < MOTOR_COUNT; i++) {
//         gpio_set_level(motors[i].dir, FORWARD);
//         gpio_set_level(motors[i].en, 0);
//     }
// }

// void motor_enable(int idx, bool on) {
//     gpio_set_level(motors[idx].en, on);
// }

// void motor_set_direction(int idx, direction_t dir) {
//     gpio_set_level(motors[idx].dir, dir);
// }

// void motor_set_pwm(int idx, uint8_t percent) {
//     if (percent > 100) percent = 100;
//     int max = (1<<LEDC_DUTY_RES) - 1;
//     int duty = (max * percent) / 100;
//     ledc_set_duty(LEDC_MODE, (ledc_channel_t)idx, duty);
//     ledc_update_duty(LEDC_MODE, (ledc_channel_t)idx);
// }

// uint32_t motor_get_pulses(int idx) {
//     return pulse_count[idx];
// }

// // Task to print pulses every second for motor 0
// static void pulse_report_task(void *pv) {
//     while (1) {
//         vTaskDelay(1000 / portTICK_PERIOD_MS);
//         ESP_LOGI(TAG, "Motor0 pulses=%" PRIu32, motor_get_pulses(0));
//     }
// }

// void app_main(void) {
//     ESP_LOGI(TAG, "Starting motor controller...");
//     motor_ctrl_init();

//     // Example: Motor 0 → enabled, forward, 50% PWM
//     motor_enable(0, false);
//     motor_set_direction(0, FORWARD);

//     for (int i = 0; i < 10; i++) {
//         motor_set_pwm(0, i);
//         vTaskDelay(400 / portTICK_PERIOD_MS);
//     }

//     // Start reporting task
//     xTaskCreate(pulse_report_task, "pulse_rpt", 2048, NULL, 5, NULL);
// }

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "motor_control.h"
#include "ethernet_setup.h"

// static const char *TAG = "app_main";

void app_main(void)
{
    ESP_LOGI("main", "1) init all motors");
    // motor_control_init_all(); // configure LEDC timer/channels, GPIO & pulse‐ISR

    ESP_LOGI("main", "2) init ethernet");
    ethernet_setup(); // bring up ETH, static IP, start server task later

    // let the idle task feed its watchdog forever
    while (true)
    {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
