#pragma once

// call this from your app_main() once the network is up.
// It will spin off a FreeRTOS task that listens on port 5000.
void tcp_com_start(void);
