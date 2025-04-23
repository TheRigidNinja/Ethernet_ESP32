#include "ethernet_setup.h"
#include "ethernet_init.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "esp_log.h"
#include "esp_err.h"
#include "esp_eth_mac.h"
#include "esp_eth_phy.h"
#include "esp_eth.h"
#include "lwip/inet.h"       // <-- brings in ipaddr_addr()

static const char *TAG = "eth_setup";

esp_err_t ethernet_setup(esp_eth_handle_t **eth_handles, uint8_t *port_cnt)
{
    esp_err_t ret = example_eth_init(eth_handles, port_cnt);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "example_eth_init() failed: %s", esp_err_to_name(ret));
        return ret;
    }

    // init LwIP + event loop
    // ESP_ERROR_CHECK( esp_netif_init() );
    // ESP_ERROR_CHECK( esp_event_loop_create_default() );

    for (int i = 0; i < *port_cnt; i++) {
        esp_netif_config_t cfg = ESP_NETIF_DEFAULT_ETH();
        esp_netif_t *netif = esp_netif_new(&cfg);

        // stop DHCP so we can assign static
        ESP_ERROR_CHECK( esp_netif_dhcpc_stop(netif) );

        // populate ip_info with ipaddr_addr()
        esp_netif_ip_info_t ip_info;
        ip_info.ip.addr      = ipaddr_addr("192.168.2.123");
        ip_info.netmask.addr = ipaddr_addr("255.255.255.0");
        ip_info.gw.addr      = ipaddr_addr("192.168.2.1");
        ESP_ERROR_CHECK( esp_netif_set_ip_info(netif, &ip_info) );

        ESP_ERROR_CHECK( esp_netif_attach(netif,
                esp_eth_new_netif_glue((*eth_handles)[i])) );
        ESP_ERROR_CHECK( esp_eth_start((*eth_handles)[i]) );
    }

    ESP_LOGI(TAG, "Ethernet configured on %d port(s)", *port_cnt);
    return ESP_OK;
}
