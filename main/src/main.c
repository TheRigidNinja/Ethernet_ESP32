#include "ethernet_setup.h"
#include "tcp_com.h"
#include "bldc_controller.h"
#include "esp_log.h"

void app_main(void)
{
    ESP_LOGI("main", "1) init motors");
    bldc_init();

    ESP_LOGI("main", "2) bring up ethernet");
    esp_eth_handle_t *eth_ports;
    uint8_t port_cnt = 0;

    // bring up Ethernet + static IP
    ESP_ERROR_CHECK(ethernet_setup(&eth_ports, &port_cnt));

    ESP_LOGI("main", "3) start TCP server");
    tcp_com_start();
}
