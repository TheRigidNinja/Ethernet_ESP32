#include "tcp_com.h"
#include "bldc_controller.h"
#include "esp_log.h"
#include "lwip/sockets.h"
#include <stdio.h>
#include <string.h>

static const char *TAG = "tcp_com";
#define PORT 5000

static void _task(void*_) {
    int srv = socket(AF_INET, SOCK_STREAM, 0);
    struct sockaddr_in addr = {
        .sin_family = AF_INET,
        .sin_port   = htons(PORT),
        .sin_addr.s_addr = htonl(INADDR_ANY)
    };
    bind(srv, (struct sockaddr*)&addr, sizeof(addr));
    listen(srv, 1);
    ESP_LOGI(TAG, "listening on %d", PORT);

    while (1) {
        int cli = accept(srv, NULL, NULL);
        if (cli<0) continue;

        char buf[64], reply[64];
        while (1) {
            int len = recv(cli, buf, sizeof(buf)-1, 0);
            if (len<=0) break;
            buf[len]=0;

            if (strcmp(buf,"GET_PULSES")==0) {
                int p = bldc_get_pulse_count();
                snprintf(reply,sizeof(reply),"PULSES=%d\n",p);
                send(cli, reply, strlen(reply),0);
            } else if (sscanf(buf,"SET_PWM %hhu",&reply[0])==1) {
                uint8_t pct = (uint8_t)atoi(buf+8);
                bldc_set_pwm_percent(pct);
            } else if (strcmp(buf,"ENABLE")==0) {
                bldc_set_enable(true);
            } else if (strcmp(buf,"DISABLE")==0) {
                bldc_set_enable(false);
            } else if (strcmp(buf,"FORWARD")==0) {
                bldc_set_direction(true);
            } else if (strcmp(buf,"REVERSE")==0) {
                bldc_set_direction(false);
            } else {
                send(cli,"UNKNOWN\n",8,0);
            }
        }
        close(cli);
    }
}

void tcp_com_start(void){
    xTaskCreate(_task, "tcp", 4096, NULL, 5, NULL);
}
