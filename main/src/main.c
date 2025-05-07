#include "esp_event.h"
#include "esp_netif.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

#include "ethernet_setup.h"
#include "tcp_com.h"
#include "bldc_controller.h"
#include "esp_task_wdt.h"
#include "bldc_pid.h" // for PID_config_t

static const char *TAG = "main";

static void bldc_init_task(void *arg)
{
    // 1) tell the task‐WDT to watch *this* task
    ESP_ERROR_CHECK(esp_task_wdt_add(NULL));

    motor_control_init(); // your long config routine

    // 2) we're done—unregister
    ESP_ERROR_CHECK(esp_task_wdt_delete(NULL));
    vTaskDelete(NULL);
}


// --------------------------------------------------------------------------------
// This is the control loop task that runs every 10ms
static void control_task(void *arg) {
    const TickType_t xFrequency = pdMS_TO_TICKS(10); // 10ms control loop
    TickType_t xLastWakeTime = xTaskGetTickCount();

    while (1) {
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
        motor_control_update_all();
    }
}


void app_main(void)
{
    // —1— bring up TCP/IP & hook events
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    tcp_com_start(); // register ETH+IP handlers (incl. got_ip->spawn server)

    ESP_LOGI(TAG, "bring up ethernet");
    esp_eth_handle_t *eth_ports;
    uint8_t port_cnt = 0;
    ESP_ERROR_CHECK(ethernet_setup(&eth_ports, &port_cnt));

    //----/2— start the TCP server (after IP is up)
    ESP_LOGI(TAG, "init motors");
    xTaskCreate(bldc_init_task, "bldc_init", 4 * 1024, NULL, 5, NULL);

    //----/3— start the control loop task
    // (this task runs every 10ms and calls motor_control_update_all())
    ESP_LOGI(TAG, "start control task");
    xTaskCreate(control_task, "control", 4096, NULL, 5, NULL);

    while (1)
    {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
