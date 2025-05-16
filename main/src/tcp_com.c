#include "tcp_com.h"
#include "ethernet_setup.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "esp_eth.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"   // for xTaskCreate
#include "freertos/task.h"
#include "lwip/sockets.h"
#include "lwip/inet.h"
#include <string.h>
#include "get_imu.h"

static const char *TAG = "TCP_COM";
#define TCP_PORT         5000
#define RECV_TIMEOUT_MS  100

static bool tcp_task_running = false;

// forward‐declare before use in got_ip_handler()
static void tcp_server_task(void* pvParameters);

static void eth_event_handler(void* arg, esp_event_base_t eb, int32_t id, void* data) {
    if (id == ETHERNET_EVENT_CONNECTED) {
        ESP_LOGI(TAG, "Ethernet Link Up");
    } else if (id == ETHERNET_EVENT_DISCONNECTED) {
        ESP_LOGI(TAG, "Ethernet Link Down");
        tcp_task_running = false;
    }
}

static void got_ip_handler(void* arg, esp_event_base_t eb, int32_t id, void* data) {
    ip_event_got_ip_t* e = (ip_event_got_ip_t*) data;
    ESP_LOGI(TAG, "Got IP: " IPSTR, IP2STR(&e->ip_info.ip));
    if (!tcp_task_running) {
        tcp_task_running = true;
        xTaskCreate(tcp_server_task, "tcp_srv", 8192, NULL, 5, NULL);
    }
}

static void handle_client(int sock) {
    char rx[64], tx[128];
    imu_data_t imu;
    struct timeval tv = { 0, RECV_TIMEOUT_MS * 1000 };
    setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));

    while (1) {
        int len = recv(sock, rx, sizeof(rx)-1, 0);
        if (len > 0) {
            rx[len] = '\0';
            if (strcmp(rx, "get_imu\n") == 0) {
                if (get_imu_data(&imu) == ESP_OK) {
                    int n = snprintf(tx, sizeof(tx),
                        "{\"accel\":[%.3f,%.3f,%.3f],\"gyro\":[%.3f,%.3f,%.3f]}\n",
                        imu.accel[0], imu.accel[1], imu.accel[2],
                        imu.gyro [0], imu.gyro [1], imu.gyro [2]
                    );
                    send(sock, tx, n, 0);
                }
            }
        }
        // periodic push @50 Hz
        if (get_imu_data(&imu) == ESP_OK) {
            int n = snprintf(tx, sizeof(tx),
                "imu:%.3f,%.3f,%.3f,%.3f,%.3f,%.3f\n",
                imu.accel[0], imu.accel[1], imu.accel[2],
                imu.gyro [0], imu.gyro [1], imu.gyro [2]
            );
            send(sock, tx, n, 0);
        }
        vTaskDelay(pdMS_TO_TICKS(20));
    }
}

static void tcp_server_task(void* pv) {
    int srv = socket(AF_INET, SOCK_STREAM, 0);
    struct sockaddr_in addr = {
        .sin_family      = AF_INET,
        .sin_port        = htons(TCP_PORT),
        .sin_addr.s_addr = INADDR_ANY
    };
    bind(srv, (struct sockaddr*)&addr, sizeof(addr));
    listen(srv, 1);
    ESP_LOGI(TAG, "Listening on TCP port %d", TCP_PORT);

    while (1) {
        struct sockaddr_in cli;
        socklen_t sl = sizeof(cli);
        int sock = accept(srv, (struct sockaddr*)&cli, &sl);
        if (sock < 0) {
            ESP_LOGE(TAG, "Accept failed: %d", sock);
            continue;
        }
        ESP_LOGI(TAG, "Client connected");
        handle_client(sock);
        close(sock);
        ESP_LOGI(TAG, "Client disconnected");
    }
}

void tcp_com_start(void) {
    ESP_ERROR_CHECK(esp_event_handler_register(ETH_EVENT, ESP_EVENT_ANY_ID,    &eth_event_handler, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT,  IP_EVENT_ETH_GOT_IP, &got_ip_handler,  NULL));
}
