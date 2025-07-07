#ifndef SERVER_H
#define SERVER_H

#include "networking_common.h"
#include "wifi.h"
#include "uart.h"
#include "ap_defs.h"
#include "test_packet_def.h"
#include "packet_ops.h"

#define SERVER_PORT (45678)

int server_send(struct sockaddr *client_addr, socklen_t *client_addr_len);
void server_task(void *arg);

#endif
