#include <iostream>
#include <cstring>
#include <iterator>
#include <climits>

#include <limits>
#include <unistd.h>

#include "common.hpp"
#include "networking_common.hpp"
#include "test_packet_def.h"

#define SERVER_PORT (45678)
#define NUM_POSSIBLE_TESTS (3)

static int sockfd = 0;
static struct sockaddr_in server_addr;
static socklen_t server_addr_len = sizeof(server_addr);
static char user_input_buffer[TEST_PACKET_STR_MAX_LEN] = {0};
static uint8_t tx_buffer[TEST_PACKET_SIZE_BYTES] = {0};
static uint8_t rx_buffer[TEST_PACKET_SIZE_BYTES] = {0};

static char test_names[NUM_POSSIBLE_TESTS][8] =
{
    "UART\0", "I2C\0", "SPI\0",
};

static void listen_for_responses(uint8_t test_selection_byte)
{
    bool results_received = false;

    while (!should_terminate && !results_received)
    {
        size_t received_bytes = recvfrom(sockfd, rx_buffer, sizeof(rx_buffer), 0, (struct sockaddr*)&server_addr, &server_addr_len);

        if (received_bytes <= 0)
        {
            perror("Receiving failed");
        }
        else if (rx_buffer[0] == TEST_PACKET_START_BYTE_VALUE && rx_buffer[TEST_PACKET_SIZE_BYTES -1] == TEST_PACKET_END_BYTE_VALUE)
        {
            switch((TestPacketMsg_t)rx_buffer[TEST_PACKET_MSG_BYTE_OFFSET])
            {
            case TESTMSG_ACK_SERVER:
                std::cout << "Received acknowledgement from server.\n";
                break;
            case TESTMSG_ACK_DEVICE:
                std::cout << "Received acknowledgement from device.\n";
                break;
            case TESTMSG_RESULT:
                results_received = true;
                std::cout << "Received results.\n";

                for (uint8_t i = 0; i < NUM_POSSIBLE_TESTS; i++)
                {
                    if (0x01 & (test_selection_byte >> (uint8_t)i))
                    {
                        std::cout << test_names[i] << " Test ";

                        if (0x01 & (rx_buffer[TEST_PACKET_SELECTION_BYTE_OFFSET] >> (uint8_t)i))
                        {
                            std::cout << "Passed.\n";
                        }
                        else
                        {
                            std::cout << "Failed.\n";
                        }
                    }
                }

                break;
            case TESTMSG_NONE:
            case TESTMSG_NEWTEST:
            default:
                std::cout << "Received unexpected response packet.\n";
                break;
            }
        }
        else
        {
            std::cout << "Received invalid packet.\n";
        }
    }
}

static bool send_test_packet(void)
{
    int sent_bytes = sendto(sockfd, tx_buffer, sizeof(tx_buffer), 0, (struct sockaddr*)&server_addr, server_addr_len);

    if (sent_bytes <= 0)
    {
        perror("Sending failed");
        return false;
    }

    return true;
}

static void init_udp(void)
{
    char server_ip_str[16] = {0};

    bzero(&server_addr, sizeof(server_addr));
    server_addr.sin_family = AF_INET;
    server_addr.sin_port = htons(SERVER_PORT);

    std::cout << "Please input server IP.\n";
    std::cin.getline(server_ip_str, std::size(server_ip_str));
    std::cin.clear();

    if (0 == inet_aton(server_ip_str, &server_addr.sin_addr))
    {
        std::cout << "Failed parsing IP.\n";
        exit(EXIT_FAILURE);
    }

    if (should_terminate) return;

    if ((sockfd = socket(AF_INET, SOCK_DGRAM, 0)) < 0)
    {
        perror("Socket creation failed");
        exit(EXIT_FAILURE);
    }

}

static void init_packet(void)
{
    explicit_bzero(tx_buffer, sizeof(tx_buffer));
    tx_buffer[0] = TEST_PACKET_START_BYTE_VALUE;
    tx_buffer[TEST_PACKET_MSG_BYTE_OFFSET] = TESTMSG_NEWTEST;
    tx_buffer[TEST_PACKET_SIZE_BYTES-1] = TEST_PACKET_END_BYTE_VALUE;
}

int main(void)
{
    initialize_signal_handler();
    init_udp();

    while(!should_terminate)
    {
        init_packet();

        std::cout << "Please input a test string.\n";
        std::cin.getline(user_input_buffer, std::size(user_input_buffer));
        std::cin.clear();
        std::cout << "Given input: " << user_input_buffer << std::endl;

        bool selection_valid = false;
        char test_selection_input[2] = {0};
        uint8_t test_selection_byte = 0;

        while (!selection_valid)
        {
            if (should_terminate) goto terminate;

            std::cout << "\nSelect a test combination:\n";
            std::cout << "1. UART\n";
            std::cout << "2. I2C\n";
            std::cout << "3. UART + I2C\n";
            std::cout << "4. SPI\n";
            std::cout << "5. UART + SPI\n";
            std::cout << "6. I2C + SPI\n";
            std::cout << "7. UART + I2C + SPI\n";

            std::cin.getline(test_selection_input, 2);
            std::cin.clear();
            test_selection_byte = std::atoi(test_selection_input);

            selection_valid = test_selection_byte > 0 && test_selection_byte <= 7;
        }

        tx_buffer[TEST_PACKET_SELECTION_BYTE_OFFSET] = test_selection_byte;
        tx_buffer[TEST_PACKET_STRING_LEN_OFFSET] = strlen(user_input_buffer);

        std::snprintf((char *)(tx_buffer + TEST_PACKET_STRING_HEAD_OFFSET), TEST_PACKET_STR_MAX_LEN, "%s", user_input_buffer);

        if(send_test_packet())
        {
            std::cout << "Sent Test Request." << std::endl;
            listen_for_responses(test_selection_byte);
        }
    }

terminate:
    close(sockfd);

    std::cout << "\nTerminated by user.\n";

    return EXIT_SUCCESS;
}
