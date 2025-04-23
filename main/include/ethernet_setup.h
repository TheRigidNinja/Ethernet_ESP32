#pragma once
#include "esp_eth.h"

// initialize the internal EMAC+PHY, register events, start it.
// Returns number of ports in *port_cnt, handles in *eth_handles.
esp_err_t ethernet_setup(esp_eth_handle_t **eth_handles, uint8_t *port_cnt);
