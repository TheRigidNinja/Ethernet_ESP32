#include "esp_event.h"
#include "esp_netif.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

#include "ethernet_setup.h"
#include "tcp_com.h"
#include "esp_task_wdt.h"
#include "get_imu.h" // for PID_config_t
#include "driver/i2c.h" // <— needed for I2C_NUM_0 & i2c_master_write_to_device



static const char *TAG = "MAIN";

// Uncomment to only scan & list I²C devices on boot
// #define ENABLE_I2C_SCAN

void i2c_scan_task(void *pv)
{
    ESP_LOGI(TAG, "I²C Scanner starting...");
    for (uint8_t addr = 1; addr < 127; addr++)
    {
        if (i2c_master_write_to_device(
                I2C_NUM_0, addr << 1, NULL, 0, pdMS_TO_TICKS(100)) == ESP_OK)
        {
            ESP_LOGI(TAG, "Found I2C device at 0x%02X", addr);
        }
    }
    vTaskDelete(NULL);
}

void imu_log_task(void *pv)
{
    imu_data_t d;
    while (1)
    {
        if (get_imu_data(&d) == ESP_OK)
        {
            ESP_LOGI("IMU_TEST",
                     "A: %.2f,%.2f,%.2f  G: %.2f,%.2f,%.2f",
                     d.accel[0], d.accel[1], d.accel[2],
                     d.gyro[0], d.gyro[1], d.gyro[2]);
        }
        else
        {
            ESP_LOGW("IMU_TEST", "read error");
        }
        vTaskDelay(pdMS_TO_TICKS(100)); // 10 Hz
    }
}

void app_main(void)
{
    ESP_LOGI(TAG, "App starting…");

    // —1— bring up TCP/IP & hook events
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    tcp_com_start(); // register ETH+IP handlers (incl. got_ip->spawn server)

    ESP_LOGI(TAG, "bring up ethernet");
    esp_eth_handle_t *eth_ports;
    uint8_t port_cnt = 0;
    ESP_ERROR_CHECK(ethernet_setup(&eth_ports, &port_cnt));

    // 2) Init IMU
    ESP_ERROR_CHECK(imu_init());

#ifdef ENABLE_I2C_SCAN
    xTaskCreate(i2c_scan_task, "i2c_scan", 2048, NULL, 5, NULL);
#else
    // 3a) serial console logger
    xTaskCreate(imu_log_task, "imu_log", 4096, NULL, 5, NULL);
    // 3b) start TCP server + JSON streamer
    tcp_com_start();
#endif
}
