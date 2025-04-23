// main/src/bldc_controller.c
#include "freertos/FreeRTOS.h" // for pdMS_TO_TICKS()
#include "freertos/task.h"     // for vTaskDelay()
#include "esp_task_wdt.h"      // if you’re using esp_task_wdt_reset()
#include "bldc_controller.h"
#include "driver/ledc.h"
#include "driver/gpio.h"
#include "esp_err.h"
#include "esp_log.h"
#include "driver/gpio.h"
static const char *TAG = "bldc_ctrl";

//----------------------------------------------------------------------
// Per-motor pin assignments (fill in with your real GPIOs)
typedef struct
{
    gpio_num_t sv_pwm_gpio; // LEDC PWM output
    gpio_num_t dir_gpio;    // direction output
    gpio_num_t en_gpio;     // enable output
    gpio_num_t pg_gpio;     // pulse‐generator input
} MotorPins_t;

static const MotorPins_t motors[MOTOR_COUNT] = {
    {.sv_pwm_gpio = GPIO_NUM_12, .dir_gpio = GPIO_NUM_16, .en_gpio = GPIO_NUM_32, .pg_gpio = GPIO_NUM_34},
    {.sv_pwm_gpio = GPIO_NUM_13, .dir_gpio = GPIO_NUM_17, .en_gpio = GPIO_NUM_33, .pg_gpio = GPIO_NUM_35},
    {.sv_pwm_gpio = GPIO_NUM_14, .dir_gpio = GPIO_NUM_2, .en_gpio = GPIO_NUM_0, .pg_gpio = GPIO_NUM_36},
    {.sv_pwm_gpio = GPIO_NUM_15, .dir_gpio = GPIO_NUM_4, .en_gpio = GPIO_NUM_10, .pg_gpio = GPIO_NUM_39},
};

//----------------------------------------------------------------------
// LEDC configuration: one timer + one channel per motor
#define BLDC_LED_TIMER LEDC_TIMER_0
#define BLDC_LED_MODE LEDC_HIGH_SPEED_MODE
#define BLDC_LED_DUTY_RES LEDC_TIMER_10_BIT // 10-bit: 0..1023

void motor_control_init(void)
{
    ESP_LOGI(TAG, "Initializing %d BLDC motors", MOTOR_COUNT);

    // 1) configure LEDC timer
    ledc_timer_config_t timer_cfg = {
        .speed_mode = BLDC_LED_MODE,
        .timer_num = BLDC_LED_TIMER,
        .duty_resolution = BLDC_LED_DUTY_RES,
        .freq_hz = 20000, // 20 kHz PWM
        .clk_cfg = LEDC_AUTO_CLK,
    };
    ESP_ERROR_CHECK(ledc_timer_config(&timer_cfg));

    // 2) configure each motor’s channel & GPIOs
    for (int i = 0; i < MOTOR_COUNT; i++)
    {
        const MotorPins_t *m = &motors[i];
        // LEDC channel
        ledc_channel_config_t ch_cfg = {
            .speed_mode = BLDC_LED_MODE,
            .channel = i, // use channel i
            .timer_sel = BLDC_LED_TIMER,
            .intr_type = LEDC_INTR_DISABLE,
            .gpio_num = m->sv_pwm_gpio,
            .duty = 0, // start off
            .hpoint = 0,
        };
        ESP_ERROR_CHECK(ledc_channel_config(&ch_cfg));

        // direction & enable pins
        gpio_config_t io_cfg = {
            .mode = GPIO_MODE_OUTPUT,
            .pin_bit_mask = (1ULL << m->dir_gpio) | (1ULL << m->en_gpio),
        };
        ESP_ERROR_CHECK(gpio_config(&io_cfg));

        // pulse-generator pin
        gpio_set_direction(m->pg_gpio, GPIO_MODE_INPUT);
        // TODO: attach your IRAM_ATTR pulse-count ISR here if needed

        // make sure motor is off
        gpio_set_level(m->en_gpio, 0);

        esp_task_wdt_reset(); // explicit feed
        vTaskDelay(pdMS_TO_TICKS(1));
    }
}

//----------------------------------------------------------------------
// Helpers
static inline bool valid_id(int id)
{
    return (id >= 0 && id < MOTOR_COUNT);
}

//----------------------------------------------------------------------
// Public API
void motor_control_set_enable(int id, bool on)
{
    if (!valid_id(id))
        return;
    gpio_set_level(motors[id].en_gpio, on);
}

void motor_control_set_direction(int id, bool forward)
{
    if (!valid_id(id))
        return;
    gpio_set_level(motors[id].dir_gpio, forward);
}

void motor_control_set_pwm(int id, uint8_t pct)
{
    if (!valid_id(id))
        return;
    // scale 0–100% into 0–1023
    uint32_t duty = (pct * ((1 << BLDC_LED_DUTY_RES) - 1)) / 100;
    ESP_ERROR_CHECK(ledc_set_duty(BLDC_LED_MODE, (ledc_channel_t)id, duty));
    ESP_ERROR_CHECK(ledc_update_duty(BLDC_LED_MODE, (ledc_channel_t)id));
}

uint32_t motor_control_get_pulses(int id)
{
    if (!valid_id(id))
        return 0;
    // if you have a hardware counter, return it; else read GPIO
    return gpio_get_level(motors[id].pg_gpio);
}
