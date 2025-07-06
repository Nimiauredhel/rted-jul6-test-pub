#ifndef UART_H
#define UART_H

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "driver/uart.h"
#include "hal/uart_hal.h"
#include "hal/uart_types.h"

extern volatile bool uart_initialized;

void uart_send(const char *message, uint8_t len);
int uart_receive(char *buff);
void uart_task(void *arg);

#endif
