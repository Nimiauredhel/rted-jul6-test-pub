#ifndef SERVER_H
#define SERVER_H

#include "networking_common.h"
#include "wifi.h"
#include "uart.h"

#define HUB_AP_SSID "SSID"
#define HUB_AP_PASS "PASS"
#define SERVER_PORT (45678)

int server_send(char *message, size_t message_len, struct sockaddr *client_addr, socklen_t *client_addr_len);
void server_task(void *arg);

#endif
