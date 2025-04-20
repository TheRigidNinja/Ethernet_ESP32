#include "ethernet_setup.h"
#include "ethernet_init.h" // from the esp‑idf example component
#include "esp_event.h"
#include "esp_netif.h"
#include "esp_eth.h"
#include "esp_log.h"
#include "lwip/sockets.h"
#include "lwip/inet.h"
#include <string.h>
#include <inttypes.h>
#include "motor_control.h" // your 4‑motor interface
#include "cJSON.h"         // JSON parser

static const char *TAG = "eth_setup";
#define TCP_PORT 5000
#define RECV_TIMEOUT_MS 100 // recv() timeout so we can watchdog‑feed

// guard so we only start the TCP task once
static bool tcp_started = false;

// —————————————————————————————————————————————————————————
// Ethernet event handler
// Ethernet event handler (link up/down/etc)
static void eth_event_handler(void* arg, esp_event_base_t eb, int32_t id, void* data)
{
    esp_eth_handle_t eth = *(esp_eth_handle_t*)data;
    uint8_t mac[6];
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


// forward‐declare our server task
static void tcp_server_task(void *);

// —————————————————————————————————————————————————————————
// Got IP handler: spawn the TCP server once we have an IP
static void got_ip_handler(void* arg, esp_event_base_t eb, int32_t id, void* data)
{
    ip_event_got_ip_t* e = (ip_event_got_ip_t*)data;
    ESP_LOGI(TAG, "Got IP: " IPSTR, IP2STR(&e->ip_info.ip));

    if (!tcp_started) {
        tcp_started = true;
        xTaskCreatePinnedToCore(
            tcp_server_task,       // your accept()/recv()/motor_control_set_* loop
            "tcp_srv",             // task name
            8192,                  // bump the stack a bit
            NULL,                  // params
            5,                     // priority
            NULL,                  // handle
            0                      // run on core 0
        );
    }
}


// —————————————————————————————————————————————————————————
// TCP server task: accept JSON arrays of commands
static void tcp_server_task(void *pvParameters)
{
    int server_fd, client_fd;
    struct sockaddr_in addr = {
        .sin_family = AF_INET,
        .sin_port = htons(TCP_PORT),
        .sin_addr.s_addr = htonl(INADDR_ANY)};
    char recv_buf[512];

    // 1) Create, bind, listen
    server_fd = socket(AF_INET, SOCK_STREAM, IPPROTO_IP);
    if (server_fd < 0)
    {
        ESP_LOGE(TAG, "socket(): errno %d", errno);
        vTaskDelete(NULL);
        return;
    }
    int yes = 1;
    setsockopt(server_fd, SOL_SOCKET, SO_REUSEADDR, &yes, sizeof(yes));
    fcntl(server_fd, F_SETFL, O_NONBLOCK); // make non‑blocking

    if (bind(server_fd, (struct sockaddr *)&addr, sizeof(addr)) != 0)
    {
        ESP_LOGE(TAG, "bind(): errno %d", errno);
        close(server_fd);
        vTaskDelete(NULL);
        return;
    }
    if (listen(server_fd, 1) != 0)
    {
        ESP_LOGE(TAG, "listen(): errno %d", errno);
        close(server_fd);
        vTaskDelete(NULL);
        return;
    }
    ESP_LOGI(TAG, "TCP server listening on port %d", TCP_PORT);

    while (1)
    {
        // 2) accept() in non‑blocking mode
        client_fd = accept(server_fd, NULL, NULL);
        if (client_fd < 0)
        {
            // no incoming connection
            vTaskDelay(pdMS_TO_TICKS(10));
            continue;
        }
        ESP_LOGI(TAG, "Client connected");

        // set a recv timeout so we can break out if client disappears
        struct timeval tv = {.tv_sec = RECV_TIMEOUT_MS / 1000, .tv_usec = (RECV_TIMEOUT_MS % 1000) * 1000};
        setsockopt(client_fd, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));

        // 3) handle one client at a time
        while (1)
        {
            int len = recv(client_fd, recv_buf, sizeof(recv_buf) - 1, 0);
            if (len < 0)
            {
                if (errno == EAGAIN || errno == EWOULDBLOCK)
                {
                    // no data right now
                    vTaskDelay(pdMS_TO_TICKS(10));
                    continue;
                }
                // some other error: disconnect
                break;
            }
            if (len == 0)
            {
                // client cleanly disconnected
                break;
            }

            recv_buf[len] = '\0';
            ESP_LOGD(TAG, "Received: %s", recv_buf);

            // 4) parse JSON
            cJSON *root = cJSON_Parse(recv_buf);
            if (root)
            {
                cJSON *motors = cJSON_GetObjectItem(root, "motors");
                if (cJSON_IsArray(motors))
                {
                    cJSON *mitem;
                    cJSON_ArrayForEach(mitem, motors)
                    {
                        cJSON *jid = cJSON_GetObjectItem(mitem, "id");
                        cJSON *jen = cJSON_GetObjectItem(mitem, "enable");
                        cJSON *jdir = cJSON_GetObjectItem(mitem, "direction");
                        cJSON *jpwm = cJSON_GetObjectItem(mitem, "pwm");
                        if (!cJSON_IsNumber(jid))
                            continue;
                        int id = jid->valueint;
                        if (id < 0 || id >= MOTOR_COUNT)
                            continue;

                        if (cJSON_IsBool(jen))
                        {
                            motor_control_set_enable(id, cJSON_IsTrue(jen));
                        }
                        if (cJSON_IsString(jdir))
                        {
                            bool fwd = strcmp(jdir->valuestring, "forward") == 0;
                            motor_control_set_direction(id, fwd);
                        }
                        if (cJSON_IsNumber(jpwm))
                        {
                            motor_control_set_pwm(id, (uint8_t)jpwm->valueint);
                        }
                    }
                }
                cJSON_Delete(root);
            }

            // 5) send back an ACK
            const char *ack = "{\"status\":\"ok\"}\n";
            send(client_fd, ack, strlen(ack), 0);
        }

        ESP_LOGI(TAG, "Client disconnected");
        close(client_fd);
    }

    // never reached
    close(server_fd);
    vTaskDelete(NULL);
}

// —————————————————————————————————————————————————————————
// Public setup: must be called from your app_main()
void ethernet_setup(void)
{
    esp_eth_handle_t* eth_handles;
    uint8_t count = 0;

    // --- 1) init the low-level ETH driver from the example component
    ESP_ERROR_CHECK(example_eth_init(&eth_handles, &count));

    // --- 2) setup LwIP & event loop
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    // --- 3) create single-port netif & assign static IP
    esp_netif_config_t cfg = ESP_NETIF_DEFAULT_ETH();
    esp_netif_t* eth_netif = esp_netif_new(&cfg);

    // **Stop DHCP so we can set our own address**
    ESP_ERROR_CHECK(esp_netif_dhcpc_stop(eth_netif));

    // **Populate ip_info safely**
    esp_netif_ip_info_t ip_info;
    ip_info.ip.addr      = ipaddr_addr("192.168.2.123");
    ip_info.gw.addr      = ipaddr_addr("192.168.2.1");
    ip_info.netmask.addr = ipaddr_addr("255.255.255.0");
    ESP_ERROR_CHECK(esp_netif_set_ip_info(eth_netif, &ip_info));

    // --- 4) attach and start ETH
    ESP_ERROR_CHECK(esp_netif_attach(eth_netif,
                        esp_eth_new_netif_glue(eth_handles[0])));
    ESP_ERROR_CHECK(esp_event_handler_register(
        ETH_EVENT, ESP_EVENT_ANY_ID, &eth_event_handler, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(
        IP_EVENT, IP_EVENT_ETH_GOT_IP, &got_ip_handler, NULL));
    ESP_ERROR_CHECK(esp_eth_start(eth_handles[0]));
}