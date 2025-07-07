#include "uart.h"

static const int uart_buffer_size = 1024;

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

void uart_init(void)
{
    printf("Initializing UART task.\n");
    ESP_ERROR_CHECK(uart_driver_install(UART_NUM_2, uart_buffer_size, uart_buffer_size, 8, &uart_queue, 0));

    // Configure UART parameters
    ESP_ERROR_CHECK(uart_param_config(uart_num, &uart_config));

    // Set UART pins(TX, RX, RTS, CTS)
    ESP_ERROR_CHECK(uart_set_pin(UART_NUM_2, 4, 5, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));

    uart_initialized = true;
}

void uart_send(const uint8_t *buff)
{
    uart_write_bytes(uart_num, buff, TEST_PACKET_SIZE_BYTES);
}

int uart_receive(uint8_t *buff)
{
    int length = 0;
    ESP_ERROR_CHECK(uart_get_buffered_data_len(uart_num, (size_t*)&length));
    if (length >= TEST_PACKET_SIZE_BYTES)
    {
        length = uart_read_bytes(uart_num, buff, TEST_PACKET_SIZE_BYTES, pdMS_TO_TICKS(10));
        return length;
    }
    else return 0;
}
