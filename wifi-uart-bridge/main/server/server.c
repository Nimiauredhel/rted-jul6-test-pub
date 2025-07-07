#include "server.h"

static int server_socket = -1;
static struct sockaddr_in server_addr ={0};

static volatile bool ap_connected = false;
static uint8_t server_tx_buff[TEST_PACKET_SIZE_BYTES] = {0};
static uint8_t server_rx_buff[TEST_PACKET_SIZE_BYTES] = {0};

esp_netif_ip_info_t server_get_ip_info(void)
{
    esp_netif_ip_info_t ip_info;
    esp_netif_get_ip_info(esp_netif_get_default_netif(), &ip_info);
    return ip_info;
}

int server_send(struct sockaddr *client_addr, socklen_t *client_addr_len)
{
    return sendto(server_socket, server_tx_buff, TEST_PACKET_SIZE_BYTES, 0, client_addr, *client_addr_len);
}

static void init_local_data_socket(int *socket_ptr, struct sockaddr_in *address_ptr)
{
    address_ptr->sin_family = AF_INET;
    address_ptr->sin_addr.s_addr = INADDR_ANY;
    address_ptr->sin_port = htons(SERVER_PORT);

    //static const int reuse_flag = 1;
    static const struct timeval socket_timeout = { .tv_sec = 1, .tv_usec = 0 };

    *socket_ptr = socket(AF_INET, SOCK_DGRAM, IPPROTO_IP);

    if (*socket_ptr < 0)
    {
        perror("Failed to create data socket");
        printf("Restarting in 2 seconds...\n");
        vTaskDelay(pdMS_TO_TICKS(2000));
        esp_restart();
    }

    /*
    if(0 > setsockopt(*socket_ptr, SOL_SOCKET, SO_REUSEADDR,  &reuse_flag, sizeof(reuse_flag)))
    {
        perror("Failed to set socket 'reuse address' option");
        exit(EXIT_FAILURE);
    }

    if(0 > setsockopt(*socket_ptr, SOL_SOCKET, SO_REUSEPORT,  &reuse_flag, sizeof(reuse_flag)))
    {
        perror("Failed to set socket 'reuse port' option");
        exit(EXIT_FAILURE);
    }
    */

    if(0 > setsockopt(*socket_ptr, SOL_SOCKET, SO_RCVTIMEO,  &socket_timeout, sizeof(socket_timeout)))
    {
        perror("Failed to set socket timeout");
        printf("Restarting in 1 second...\n");
        vTaskDelay(pdMS_TO_TICKS(1000));
        esp_restart();
    }

    printf("Created new server socket (FD: %d).\n", *socket_ptr);

    int bind_result = bind(*socket_ptr, (struct sockaddr *)address_ptr, sizeof(*address_ptr));

    if (bind_result < 0)
    {
        perror("Failed to bind socket");
        printf("Restarting in 1 second...\n");
        vTaskDelay(pdMS_TO_TICKS(1000));
        esp_restart();
    }
}

static void event_handler(void* arg, esp_event_base_t event_base,
                                int32_t event_id, void* event_data)
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED)
    {
        ap_connected = false;
        printf("Connecting to the AP failed, retrying.\n");
        ESP_ERROR_CHECK(esp_wifi_connect());
        wifi_ap_connect(HUB_AP_SSID, HUB_AP_PASS);
    }
    else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_CONNECTED)
    {
        ap_connected = true;
        printf("Connecting to the AP succeeded.\n");
    }
    else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP)
    {
        ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
        printf("got ip: " IPSTR " \n", IP2STR(&event->ip_info.ip));

        esp_netif_ip_info_t ip_info = server_get_ip_info();
        printf("My IP: " IPSTR "\n", IP2STR(&ip_info.ip));
        printf("My GW: " IPSTR "\n", IP2STR(&ip_info.gw));
        printf("My NETMASK: " IPSTR "\n", IP2STR(&ip_info.netmask));
    }
}

static void server_init(void)
{
    printf("Initializing Server task.\n");

    esp_event_handler_instance_t instance_any_id;
    esp_event_handler_instance_t instance_got_ip;

    ESP_ERROR_CHECK(
    esp_event_handler_instance_register(WIFI_EVENT,
                                        ESP_EVENT_ANY_ID,
                                        &event_handler,
                                        NULL,
                                        &instance_any_id));

    ESP_ERROR_CHECK(
    esp_event_handler_instance_register(IP_EVENT,
                                        IP_EVENT_STA_GOT_IP,
                                        &event_handler,
                                        NULL,
                                        &instance_got_ip));

    printf("Starting WiFi functionality.\n");
    ESP_ERROR_CHECK(esp_wifi_start());

    printf("Calling initial connection to access point (SSID: %s).\n", HUB_AP_SSID);
    wifi_ap_connect(HUB_AP_SSID, HUB_AP_PASS);

    while (!ap_connected) vTaskDelay(pdMS_TO_TICKS(100));

    init_local_data_socket(&server_socket, &server_addr);
}

static void process_request(struct sockaddr *client_addr, socklen_t *client_addr_len)
{
    uint8_t uart_rx_buff[TEST_PACKET_SIZE_BYTES] = {0};

    printf("Processing request from client.\n");
    printf("Request string: %s\n", server_rx_buff+TEST_PACKET_STRING_HEAD_OFFSET);

    packet_init(server_tx_buff);
    server_tx_buff[TEST_PACKET_MSG_BYTE_OFFSET] = TESTMSG_ACK_SERVER;
    server_send(client_addr, client_addr_len);

    uart_send(server_rx_buff);
    printf("Forwarded request via UART, awaiting response from device.\n");

    bool received_results = false;

    while(!received_results)
    {
        explicit_bzero(uart_rx_buff, sizeof(uart_rx_buff));
        size_t received_bytes = 0;

        while (received_bytes <= 0)
        {
            vTaskDelay(pdMS_TO_TICKS(10));
            received_bytes = uart_receive(uart_rx_buff);
        }

        if (uart_rx_buff[TEST_PACKET_MSG_BYTE_OFFSET] == TESTMSG_RESULT)
        {
            received_results = true;
            printf("Forwarding results to client and concluding.\n");
        }

        memcpy(server_tx_buff, uart_rx_buff, TEST_PACKET_SIZE_BYTES);
        while(0 > server_send(client_addr, client_addr_len));
    }
}

static int receive_request()
{
    struct sockaddr client_addr;
    socklen_t client_addr_len;
    explicit_bzero(&server_rx_buff, sizeof(server_rx_buff));
    int received_bytes = recvfrom(server_socket, &server_rx_buff, sizeof(server_rx_buff), 0, &client_addr, &client_addr_len);

    if (received_bytes > 0)
    {
        process_request(&client_addr, &client_addr_len);
    }

    return received_bytes;
}

static void server_loop(void)
{
    vTaskDelay(pdMS_TO_TICKS(10));
    receive_request();
}

void server_task(void *arg)
{
    server_init();

    while(!uart_initialized) vTaskDelay(pdMS_TO_TICKS(100));

    for(;;)
    {
        server_loop();
    }
}
