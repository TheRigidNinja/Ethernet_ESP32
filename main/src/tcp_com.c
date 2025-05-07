#include "tcp_com.h"
#include "ethernet_setup.h" // for esp_eth_handle_t
#include "esp_event.h"
#include "esp_netif.h"
#include "esp_eth.h"
#include "esp_log.h"
#include "lwip/sockets.h"
#include "lwip/inet.h"
#include <string.h>
#include "bldc_controller.h" // your motor_control_*() API
#include "cJSON.h"
#include "bldc_pid.h"

static const char *TAG = "tcp_com";
#define TCP_PORT 5000
#define RECV_TIMEOUT_MS 100

static bool tcp_task_running = false;
static esp_eth_handle_t *eth_h = NULL; // unused, but evt handler requires

//--------------------------------------------------------------------------------
// ETH event handler: log link up/down
static void eth_event_handler(void *arg, esp_event_base_t eb, int32_t id, void *data)
{
    uint8_t mac[6];
    esp_eth_handle_t eth = *(esp_eth_handle_t *)data;
    switch (id)
    {
    case ETHERNET_EVENT_CONNECTED:
        esp_eth_ioctl(eth, ETH_CMD_G_MAC_ADDR, mac);
        ESP_LOGI(TAG, "Link Up: %02x:%02x:%02x:%02x:%02x:%02x",
                 mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
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
// the JSON‐over‐TCP server task
static void tcp_server_task(void *pv)
{
    int srv = socket(AF_INET, SOCK_STREAM, IPPROTO_IP);
    if (srv < 0)
    {
        ESP_LOGE(TAG, "socket() failed: %d", errno);
        vTaskDelete(NULL);
        return;
    }

    // reuse & non-block
    int yes = 1;
    setsockopt(srv, SOL_SOCKET, SO_REUSEADDR, &yes, sizeof(yes));
    fcntl(srv, F_SETFL, O_NONBLOCK);

    struct sockaddr_in addr = {
        .sin_family = AF_INET,
        .sin_port = htons(TCP_PORT),
        .sin_addr.s_addr = htonl(INADDR_ANY)};
    bind(srv, (struct sockaddr *)&addr, sizeof(addr));
    listen(srv, 1);
    ESP_LOGI(TAG, "TCP listening on port %d", TCP_PORT);

    while (1)
    {
        int client = accept(srv, NULL, NULL);
        if (client < 0)
        {
            vTaskDelay(pdMS_TO_TICKS(10));
            continue;
        }
        ESP_LOGI(TAG, "Client connected");

        // install recv timeout
        struct timeval tv = {
            .tv_sec = RECV_TIMEOUT_MS / 1000,
            .tv_usec = (RECV_TIMEOUT_MS % 1000) * 1000};
        setsockopt(client, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));

        char buf[512];
        while (1)
        {
            int len = recv(client, buf, sizeof(buf) - 1, 0);
            if (len < 0)
            {
                if (errno == EAGAIN || errno == EWOULDBLOCK)
                {
                    vTaskDelay(pdMS_TO_TICKS(10));
                    continue;
                }
                break;
            }
            if (len == 0)
            {
                // clean disconnect
                break;
            }

            buf[len] = '\0';
            ESP_LOGD(TAG, "RX: %s", buf);

            // parse JSON
            cJSON *root = cJSON_Parse(buf);
            if (root)
            {
                bool handled = false;
                cJSON *arr = cJSON_GetObjectItem(root, "motors");
                if (cJSON_IsArray(arr))
                {
                    cJSON *m;
                    cJSON_ArrayForEach(m, arr)
                    {
                        cJSON *jid = cJSON_GetObjectItem(m, "id");
                        cJSON *jen = cJSON_GetObjectItem(m, "enable");
                        cJSON *jdir = cJSON_GetObjectItem(m, "direction");
                        cJSON *jpwm = cJSON_GetObjectItem(m, "pwm");
                        cJSON *jgp = cJSON_GetObjectItem(m, "pulses");
                        cJSON *target_item = cJSON_GetObjectItem(m, "target");
                        cJSON *pid_item = cJSON_GetObjectItem(m, "pid");
                        cJSON *kp = cJSON_GetObjectItem(pid_item, "kp");
                        cJSON *ki = cJSON_GetObjectItem(pid_item, "ki");
                        cJSON *kd = cJSON_GetObjectItem(pid_item, "kd");

                        // Validate ID
                        if (!cJSON_IsNumber(jid))
                            continue;
                        int id = jid->valueint;
                        if (id < 0 || id >= MOTOR_COUNT)
                            continue;

                        // Enable/disable ##-------// motor enabled has to be to low in order to get motor spinning 
                        if (cJSON_IsBool(jen))
                        {
                            motor_control_set_enable(id, cJSON_IsTrue(jen));

                            for (int i = 0; i < MOTOR_COUNT; i++)
                            {
                                uint32_t pulse_count = motor_control_get_pulses(i);
                                // ESP_LOGI(TAG, "motor[%lu] → pulse_gpio=%lu  pulse_count=%lu",i, motors[i].pg_gpio, pulse_count);

                                // ESP_LOGI(TAG, "VALUE: %lu BLDC motors pulse %d", pulse_count, );
                            }
                        }

                        // Direction
                        if (cJSON_IsString(jdir))
                        {
                            bool f = (strcmp(jdir->valuestring, "forward") == 0);
                            motor_control_set_direction(id, f);
                        }

                        //------// PWM
                        if (cJSON_IsNumber(jpwm))
                        {
                            motor_control_set_pwm(id, (uint8_t)jpwm->valueint);
                        }

                        //------// PID config
                        if (target_item && cJSON_IsNumber(target_item))
                        {
                            motor_control_set_target(id, (int32_t)target_item->valueint);
                        }
                        // PID config: kp, ki, kd
                        if (pid_item && cJSON_IsObject(pid_item))
                        {
                            PID_config_t pid = {
                                .kp = 0.5f,
                                .ki = 0.01f,
                                .kd = 0.1f,
                                .integral_max = 100.0f,
                                .output_max = 100.0f};

                            cJSON *kp = cJSON_GetObjectItemCaseSensitive(pid_item, "kp");
                            cJSON *ki = cJSON_GetObjectItemCaseSensitive(pid_item, "ki");
                            cJSON *kd = cJSON_GetObjectItemCaseSensitive(pid_item, "kd");

                            if (cJSON_IsNumber(kp))
                                pid.kp = (float)kp->valuedouble;
                            if (cJSON_IsNumber(ki))
                                pid.ki = (float)ki->valuedouble;
                            if (cJSON_IsNumber(kd))
                                pid.kd = (float)kd->valuedouble;

                            motor_control_set_pid(id, pid);
                        }

                        //------// Pulse count
                        if (cJSON_IsBool(jgp) && cJSON_IsTrue(jgp))
                        {
                            // Build and send pulse-array response
                            cJSON *resp = cJSON_CreateObject();
                            cJSON *out_arr = cJSON_AddArrayToObject(resp, "pulses");
                            for (int i = 0; i < MOTOR_COUNT; i++)
                            {
                                cJSON_AddItemToArray(
                                    out_arr,
                                    cJSON_CreateNumber(motor_control_get_pulses(i)));
                            }
                            char *s = cJSON_PrintUnformatted(resp);
                            send(client, s, strlen(s), 0);
                            send(client, "\n", 1, 0); // ← add newline so recv_line() sees end
                            free(s);
                            cJSON_Delete(resp);

                            handled = true;
                            break; // stop processing further motors
                        }
                    }
                }

                cJSON_Delete(root); // only once, after the for-each

                if (!handled)
                {
                    // no pulses request: send simple ACK
                    const char *ack = "{\"status\":\"ok\"}\n";
                    send(client, ack, strlen(ack), 0);
                }
            }
        }

        ESP_LOGI(TAG, "Client disconnected");
        close(client);
    }
    // never reached
    close(srv);
    vTaskDelete(NULL);
}

//--------------------------------------------------------------------------------
// IP event handler: when we get an IP, spin up the TCP server once
static void got_ip_handler(void *arg, esp_event_base_t eb, int32_t id, void *data)
{
    ip_event_got_ip_t *e = (ip_event_got_ip_t *)data;
    ESP_LOGI(TAG, "Got IP: " IPSTR, IP2STR(&e->ip_info.ip));
    if (!tcp_task_running)
    {
        tcp_task_running = true;
        xTaskCreate(tcp_server_task, "tcp_srv", 8192, NULL, 5, NULL);
    }
}

//--------------------------------------------------------------------------------
void tcp_com_start(void)
{
    // register ETH+IP event handlers
    ESP_ERROR_CHECK(esp_event_handler_register(ETH_EVENT, ESP_EVENT_ANY_ID, &eth_event_handler, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_ETH_GOT_IP, &got_ip_handler, NULL));
}
