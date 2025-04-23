#pragma once

#include "esp_eth.h"
#include <stdint.h>
#include "esp_err.h"

/**
 * @brief   Bring up the internal EMAC + PHY, assign a static IP,
 *          register events and start the driver.
 *
 * @param[out] eth_handles   Pointer that will receive an array of esp_eth_handle_t
 * @param[out] port_cnt      Number of ports returned in eth_handles[]
 * @return  ESP_OK on success, or any esp_err_t on failure.
 */
esp_err_t ethernet_setup(esp_eth_handle_t **eth_handles, uint8_t *port_cnt);
