#include "motor_control.h"
#include "esp_attr.h" // for IRAM_ATTR
#include "driver/ledc.h"
#include "driver/gpio.h"
#include "esp_err.h"
#include "esp_log.h"

static const char *TAG = "motor_ctrl";

// your four‑motor pin mapping
static MotorPins_t motors[MOTOR_COUNT] = {
    //   [SV]         [F/R]       [EN]        [PG]
    {GPIO_NUM_12, GPIO_NUM_16, GPIO_NUM_32, GPIO_NUM_34},
    {GPIO_NUM_13, GPIO_NUM_17, GPIO_NUM_33, GPIO_NUM_35},
    {GPIO_NUM_14, GPIO_NUM_2, GPIO_NUM_11, GPIO_NUM_36},
    {GPIO_NUM_15, GPIO_NUM_4, GPIO_NUM_10, GPIO_NUM_39}};

// ISR‑safe pulse counters
static volatile uint32_t pulse_counts[MOTOR_COUNT] = {0};

// pulse ISR
static void IRAM_ATTR pulse_isr_handler(void *arg)
{
    int idx = (int)(intptr_t)arg;
    if (idx >= 0 && idx < MOTOR_COUNT)
    {
        pulse_counts[idx]++;
    }
}

void motor_control_init_all(void)
{
    // 1) configure one shared PWM timer
    ledc_timer_config_t timer_conf = {
        .speed_mode = LEDC_HIGH_SPEED_MODE,
        .duty_resolution = LEDC_TIMER_10_BIT,
        .timer_num = LEDC_TIMER_0,
        .freq_hz = 1000,
        .clk_cfg = LEDC_AUTO_CLK};
    ESP_ERROR_CHECK(ledc_timer_config(&timer_conf));

    // 2) per‐motor setup
    for (int i = 0; i < MOTOR_COUNT; i++)
    {
        // PWM channel
        ledc_channel_config_t ch = {
            .gpio_num = motors[i].pwm_gpio,
            .speed_mode = LEDC_HIGH_SPEED_MODE,
            .channel = (ledc_channel_t)i,
            .timer_sel = LEDC_TIMER_0,
            .duty = 0,
            .hpoint = 0};
        ESP_ERROR_CHECK(ledc_channel_config(&ch));

        // direction + enable as outputs
        gpio_config_t oconf = {
            .pin_bit_mask = (1ULL << motors[i].dir_gpio) | (1ULL << motors[i].en_gpio),
            .mode = GPIO_MODE_OUTPUT,
            .pull_down_en = GPIO_PULLDOWN_DISABLE,
            .pull_up_en = GPIO_PULLUP_DISABLE,
            .intr_type = GPIO_INTR_DISABLE};
        ESP_ERROR_CHECK(gpio_config(&oconf));
        gpio_set_level(motors[i].en_gpio, 0);

        // pulse input
        gpio_config_t ic = {
            .pin_bit_mask = (1ULL << motors[i].pulse_gpio),
            .mode = GPIO_MODE_INPUT,
            .pull_down_en = GPIO_PULLDOWN_DISABLE,
            .pull_up_en = GPIO_PULLUP_DISABLE,
            .intr_type = GPIO_INTR_POSEDGE};
        ESP_ERROR_CHECK(gpio_config(&ic));
    }

    // 3) hook up ISRs
    ESP_ERROR_CHECK(gpio_install_isr_service(0));
    for (int i = 0; i < MOTOR_COUNT; i++)
    {
        ESP_ERROR_CHECK(gpio_isr_handler_add(
            motors[i].pulse_gpio,
            pulse_isr_handler,
            (void *)(intptr_t)i));
    }

    ESP_LOGI(TAG, "Initialized %d motor controllers", MOTOR_COUNT);
}

void motor_control_set_enable(int idx, bool en)
{
    if (idx < 0 || idx >= MOTOR_COUNT)
        return;
    gpio_set_level(motors[idx].en_gpio, en ? 1 : 0);
}

void motor_control_set_direction(int idx, bool forward)
{
    if (idx < 0 || idx >= MOTOR_COUNT)
        return;
    gpio_set_level(motors[idx].dir_gpio, forward ? 1 : 0);
}

void motor_control_set_pwm(int idx, uint8_t pct)
{
    if (idx < 0 || idx >= MOTOR_COUNT)
        return;
    if (pct > 100)
        pct = 100;
    // map 0–100% into 0–1023
    uint32_t duty = ((uint32_t)pct * ((1 << LEDC_TIMER_10_BIT) - 1)) / 100;
    ledc_set_duty(LEDC_HIGH_SPEED_MODE, (ledc_channel_t)idx, duty);
    ledc_update_duty(LEDC_HIGH_SPEED_MODE, (ledc_channel_t)idx);
}

uint32_t motor_control_get_pulses(int idx)
{
    if (idx < 0 || idx >= MOTOR_COUNT)
        return 0;
    return pulse_counts[idx];
}
