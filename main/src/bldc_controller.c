#include "bldc_controller.h"
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "esp_attr.h"

#define PULSE_GPIO   GPIO_NUM_39
#define PWM_GPIO     GPIO_NUM_32
#define EN_GPIO      GPIO_NUM_5
#define DIR_GPIO     GPIO_NUM_17
#define TIMER        LEDC_TIMER_0
#define CHANNEL      LEDC_CHANNEL_0

static volatile uint32_t _pulses;

static void IRAM_ATTR _pulse_isr(void*){
    _pulses++;
}

void bldc_init(void){
    // PWM timer & channel
    ledc_timer_config(&(ledc_timer_config_t){
        .speed_mode = LEDC_HIGH_SPEED_MODE,
        .duty_resolution = LEDC_TIMER_10_BIT,
        .timer_num = TIMER,
        .freq_hz = 1000,
        .clk_cfg = LEDC_AUTO_CLK
    });
    ledc_channel_config(&(ledc_channel_config_t){
        .gpio_num = PWM_GPIO,
        .speed_mode = LEDC_HIGH_SPEED_MODE,
        .channel = CHANNEL,
        .timer_sel = TIMER,
        .duty = 0
    });

    // Dir + En
    gpio_set_direction(DIR_GPIO, GPIO_MODE_OUTPUT);
    gpio_set_direction(EN_GPIO, GPIO_MODE_OUTPUT);
    gpio_set_level(EN_GPIO, 0);

    // Pulse ISR
    gpio_config(&(gpio_config_t){
        .pin_bit_mask = 1ULL<<PULSE_GPIO,
        .mode = GPIO_MODE_INPUT,
        .intr_type = GPIO_INTR_POSEDGE
    });
    gpio_install_isr_service(0);
    gpio_isr_handler_add(PULSE_GPIO, _pulse_isr, NULL);
}

void bldc_set_enable(bool on){
    gpio_set_level(EN_GPIO, on);
}

void bldc_set_direction(bool forward){
    gpio_set_level(DIR_GPIO, forward);
}

void bldc_set_pwm_percent(uint8_t pct){
    if (pct>100) pct=100;
    uint32_t duty = (pct * ((1<<10)-1))/100;
    ledc_set_duty(LEDC_HIGH_SPEED_MODE, CHANNEL, duty);
    ledc_update_duty(LEDC_HIGH_SPEED_MODE, CHANNEL);
}

uint32_t bldc_get_pulse_count(void){
    return _pulses;
}
