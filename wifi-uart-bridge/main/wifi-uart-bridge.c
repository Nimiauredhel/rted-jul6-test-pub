#include <stdio.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_system.h"
#include "nvs_flash.h"

#include "wifi.h"
#include "server.h"
#include "uart.h"

#define SERVER_STACK_SIZE (4096)
#define UART_STACK_SIZE (4096)

static StaticTask_t server_task_buffer;
static TaskHandle_t server_task_handle;
static StackType_t server_task_stack[SERVER_STACK_SIZE];

static StaticTask_t uart_task_buffer;
static TaskHandle_t uart_task_handle;
static StackType_t uart_task_stack[UART_STACK_SIZE];

void app_main(void)
{

    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES)
    {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    esp_netif_init();
    esp_event_loop_create_default();
    esp_netif_create_default_wifi_sta();
    wifi_init();

    server_task_handle = xTaskCreateStatic(server_task, "ServerTask", SERVER_STACK_SIZE, NULL, 10, server_task_stack, &server_task_buffer);
    uart_task_handle = xTaskCreateStatic(uart_task, "UARTTask", UART_STACK_SIZE, NULL, 10, uart_task_stack, &uart_task_buffer);

    for(;;)
    {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
