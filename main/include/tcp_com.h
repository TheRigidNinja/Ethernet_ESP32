#pragma once

/**
 * @brief  Install ETH/IP event handlers and start JSON‐over‐TCP server.
 *         Must be called _after_ ethernet_setup().
 */
void tcp_com_start(void);
