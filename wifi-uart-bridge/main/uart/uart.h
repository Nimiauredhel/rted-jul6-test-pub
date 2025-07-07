#ifndef UART_H
#define UART_H

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "driver/uart.h"
#include "hal/uart_hal.h"
#include "hal/uart_types.h"

#include "test_packet_def.h"
#include "packet_ops.h"

extern volatile bool uart_initialized;

void uart_init(void);
void uart_send(const uint8_t *buff);
int uart_receive(uint8_t *buff);

#endif
