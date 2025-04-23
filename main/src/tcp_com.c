#include "tcp_com.h"
#include "ethernet_setup.h"    // for esp_eth_handle_t
#include "esp_event.h"
#include "esp_netif.h"
#include "esp_eth.h"
#include "esp_log.h"
#include "lwip/sockets.h"
#include "lwip/inet.h"
#include <string.h>
#include "bldc_controller.h"     // your motor_control_*() API
#include "cJSON"        // cJSON_*

static const char *TAG         = "tcp_com";
#define TCP_PORT                5000
#define RECV_TIMEOUT_MS         100

static bool tcp_task_running   = false;
static esp_eth_handle_t *eth_h = NULL;  // unused, but evt handler requires

//--------------------------------------------------------------------------------
// ETH event handler: log link up/down
static void eth_event_handler(void* arg, esp_event_base_t eb, int32_t id, void* data)
{
    uint8_t mac[6];
    esp_eth_handle_t eth = *(esp_eth_handle_t*)data;
    switch(id) {
    case ETHERNET_EVENT_CONNECTED:
        esp_eth_ioctl(eth, ETH_CMD_G_MAC_ADDR, mac);
        ESP_LOGI(TAG, "Link Up: %02x:%02x:%02x:%02x:%02x:%02x",
                 mac[0],mac[1],mac[2],mac[3],mac[4],mac[5]);
        break;
    case ETHERNET_EVENT_DISCONNECTED:
        ESP_LOGI(TAG, "Link Down");
        break;
    case ETHERNET_EVENT_START:
        ESP_LOGI(TAG, "Eth Started");
        break;
    case ETHERNET_EVENT_STOP:
        ESP_LOGI(TAG, "Eth Stopped");
        break;
    default:
        break;
    }
}

//--------------------------------------------------------------------------------
// IP event handler: when we get an IP, spin up the TCP server once
static void got_ip_handler(void* arg, esp_event_base_t eb, int32_t id, void* data)
{
    ip_event_got_ip_t *e = (ip_event_got_ip_t*)data;
    ESP_LOGI(TAG, "Got IP: " IPSTR, IP2STR(&e->ip_info.ip));
    if (!tcp_task_running) {
        tcp_task_running = true;
        xTaskCreate(tcp_server_task, "tcp_srv", 8192, NULL, 5, NULL);
    }
}

//--------------------------------------------------------------------------------
// the JSON‐over‐TCP server task
static void tcp_server_task(void *pv)
{
    int srv = socket(AF_INET, SOCK_STREAM, IPPROTO_IP);
    if (srv < 0) {
        ESP_LOGE(TAG, "socket() failed: %d", errno);
        vTaskDelete(NULL);
        return;
    }

    // reuse & non-block
    int yes=1; setsockopt(srv, SOL_SOCKET, SO_REUSEADDR, &yes, sizeof(yes));
    fcntl(srv, F_SETFL, O_NONBLOCK);

    struct sockaddr_in addr = {
        .sin_family      = AF_INET,
        .sin_port        = htons(TCP_PORT),
        .sin_addr.s_addr = htonl(INADDR_ANY)
    };
    bind(srv, (struct sockaddr*)&addr, sizeof(addr));
    listen(srv, 1);
    ESP_LOGI(TAG, "TCP listening on port %d", TCP_PORT);

    while (1) {
        int client = accept(srv, NULL, NULL);
        if (client < 0) {
            vTaskDelay(pdMS_TO_TICKS(10));
            continue;
        }
        ESP_LOGI(TAG, "Client connected");

        // install recv timeout
        struct timeval tv = {
            .tv_sec  = RECV_TIMEOUT_MS/1000,
            .tv_usec = (RECV_TIMEOUT_MS%1000)*1000
        };
        setsockopt(client, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));

        char buf[512];
        while (1) {
            int len = recv(client, buf, sizeof(buf)-1, 0);
            if (len < 0) {
                if (errno==EAGAIN||errno==EWOULDBLOCK) {
                    vTaskDelay(pdMS_TO_TICKS(10));
                    continue;
                }
                break;
            }
            if (len == 0) {
                // clean disconnect
                break;
            }

            buf[len] = '\0';
            ESP_LOGD(TAG, "RX: %s", buf);

            // parse JSON
            cJSON *root = cJSON_Parse(buf);
            if (root) {
                cJSON *arr = cJSON_GetObjectItem(root, "motors");
                if (cJSON_IsArray(arr)) {
                    cJSON *m;
                    cJSON_ArrayForEach(m, arr) {
                        cJSON *jid  = cJSON_GetObjectItem(m, "id");
                        cJSON *jen  = cJSON_GetObjectItem(m, "enable");
                        cJSON *jdir = cJSON_GetObjectItem(m, "direction");
                        cJSON *jpwm = cJSON_GetObjectItem(m, "pwm");
                        if (!cJSON_IsNumber(jid)) continue;
                        int id = jid->valueint;
                        if (id<0 || id>=MOTOR_COUNT) continue;
                        if (cJSON_IsBool(jen)) {
                            motor_control_set_enable(id, cJSON_IsTrue(jen));
                        }
                        if (cJSON_IsString(jdir)) {
                            bool f = (strcmp(jdir->valuestring,"forward")==0);
                            motor_control_set_direction(id, f);
                        }
                        if (cJSON_IsNumber(jpwm)) {
                            motor_control_set_pwm(id, (uint8_t)jpwm->valueint);
                        }
                    }
                }
                cJSON_Delete(root);
            }
            // ack
            const char *ack = "{\"status\":\"ok\"}\n";
            send(client, ack, strlen(ack), 0);
        }

        ESP_LOGI(TAG, "Client disconnected");
        close(client);
    }
    // never reached
    close(srv);
    vTaskDelete(NULL);
}

//--------------------------------------------------------------------------------
void tcp_com_start(void)
{
    // register ETH+IP event handlers
    ESP_ERROR_CHECK( esp_event_handler_register(ETH_EVENT, ESP_EVENT_ANY_ID,  &eth_event_handler, NULL) );
    ESP_ERROR_CHECK( esp_event_handler_register(IP_EVENT,  IP_EVENT_ETH_GOT_IP, &got_ip_handler,     NULL) );
}
