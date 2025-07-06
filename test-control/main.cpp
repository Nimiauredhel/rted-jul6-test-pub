#include <iostream>
#include <cstring>
#include <unistd.h>
#include <iterator>
#include "networking_common.h"

#define SERVER_PORT (45678)

static int sockfd = 0;
static struct sockaddr_in server_addr;
static socklen_t server_addr_len;
static char tx_buffer[32] = {0};
static char rx_buffer[128] = {0};

static void listen_for_responses(void)
{
    static const uint8_t expected_response_count = 6;

    uint8_t response_count = 0;

    while (response_count < expected_response_count)
    {
        size_t received_bytes = recvfrom(sockfd, rx_buffer, sizeof(rx_buffer), 0, (struct sockaddr*)&server_addr, &server_addr_len);

        if (received_bytes <= 0)
        {
            perror("Receiving failed");
        }
        else
        {
            response_count++;
            std::cout << "Received respone! The response is: " << rx_buffer << '\n';
        }
    }

}

static void send_test_string(char *test_string)
{
    int sent_bytes = sendto(sockfd, test_string, strlen(test_string), 0, (struct sockaddr*)&server_addr, server_addr_len);

    if (sent_bytes <= 0)
    {
        perror("Sending failed");
    }
}

static void init_udp(void)
{
    char server_ip_str[32] = {0};

    std::cout << "Please input server IP.\n";
    std::cin >> server_ip_str;

    if ((sockfd = socket(AF_INET, SOCK_DGRAM, 0)) < 0)
    {
        perror("Socket creation failed");
        exit(EXIT_FAILURE);
    }

    bzero(&server_addr, sizeof(server_addr));
    server_addr.sin_family = AF_INET;
    server_addr.sin_port = htons(SERVER_PORT);
    server_addr.sin_addr.s_addr = inet_addr(server_ip_str);
    server_addr_len = sizeof(server_addr);
}

int main(void)
{
    init_udp();

    std::cin.ignore(256, '\n');

    std::cout << "Please input a test string.\n";
    std::cin.getline(tx_buffer, std::size(tx_buffer));
    std::cout << "Given input: " << tx_buffer << std::endl;

    send_test_string(tx_buffer);
    listen_for_responses();
    close(sockfd);
    return EXIT_SUCCESS;
}
