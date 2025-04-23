#include "ethernet_setup.h"
#include "esp_eth_macro_default.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "esp_log.h"
#include "ethernet_init.h"    // your board‐specific init (LAN87xx @ addr 0)

static const char *TAG = "eth_setup";

esp_err_t ethernet_setup(esp_eth_handle_t **eth_handles, uint8_t *port_cnt) {
    // 1) bring up the PHY+MAC driver
    esp_err_t err = example_eth_init(eth_handles, port_cnt);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "PHY init failed (%s)", esp_err_to_name(err));
        return err;
    }

    // 2) bring up the TCP/IP stack
    esp_netif_init();
    esp_event_loop_create_default();

    // for each port: create netif, assign static IP, attach
    for (int i = 0; i < *port_cnt; ++i) {
        esp_netif_config_t cfg = ESP_NETIF_DEFAULT_ETH();
        esp_netif_t *netif = esp_netif_new(&cfg);
        // (optional) stop DHCP, set static here...
        esp_netif_attach(netif, esp_eth_new_netif_glue((*eth_handles)[i]));
        ESP_ERROR_CHECK(esp_eth_start((*eth_handles)[i]));
    }

    // 3) register link/ip event handlers if you like
    ESP_LOGI(TAG, "Ethernet initialized (%d ports)", *port_cnt);
    return ESP_OK;
}
