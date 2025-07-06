#include "uart.h"

static const int uart_buffer_size = 2048;

static const uart_port_t uart_num = UART_NUM_2;

static const uart_config_t uart_config =
{
    .baud_rate = 115200,
    .data_bits = UART_DATA_8_BITS,
    .parity = UART_PARITY_DISABLE,
    .stop_bits = UART_STOP_BITS_1,
    .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
};

static QueueHandle_t uart_queue;

volatile bool uart_initialized = false;

static void uart_loop(void)
{
    for(;;)
    {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

static void uart_init(void)
{
    printf("Initializing UART task.\n");
    ESP_ERROR_CHECK(uart_driver_install(UART_NUM_2, uart_buffer_size, uart_buffer_size, 10, &uart_queue, 0));

    // Configure UART parameters
    ESP_ERROR_CHECK(uart_param_config(uart_num, &uart_config));

    // Set UART pins(TX, RX, RTS, CTS)
    ESP_ERROR_CHECK(uart_set_pin(UART_NUM_2, 4, 5, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));

    uart_initialized = true;
}

void uart_send(const char *message, uint8_t len)
{
    uart_write_bytes(uart_num, message, len);
}

int uart_receive(char *buff)
{
    int length = 0;
    ESP_ERROR_CHECK(uart_get_buffered_data_len(uart_num, (size_t*)&length));
    length = uart_read_bytes(uart_num, buff, length, 100);
    return length;
}

void uart_task(void *arg)
{
    uart_init();

    for(;;)
    {
        uart_loop();
    }
}
