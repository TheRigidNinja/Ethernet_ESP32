idf_component_register(
    SRCS "ethernet_init.c"
    INCLUDE_DIRS "include"
    REQUIRES esp_eth esp_netif esp_event
)
