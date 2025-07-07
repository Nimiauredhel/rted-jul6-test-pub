/*
 * peripheral_tests.c
 *
 *  Created on: Jul 7, 2025
 *      Author: mickey
 */
#include "peripheral_tests.h"

static bool test_i2c(const char *test_string, const uint8_t len);
static bool test_spi(const char *test_string, const uint8_t len);
static bool test_uart(const char *test_string, const uint8_t len);

volatile uint8_t test_string_len = 0;
char test_string_buff[TEST_STRING_MAX_LEN] = {0};

TestDefinition_t test_defs[NUM_POSSIBLE_TESTS] =
{
	{ .name = "UART\0", .state = TESTSTATE_READY, .func = test_uart, },
	{ .name = "I2C\0", .state = TESTSTATE_READY, .func = test_i2c, },
	{ .name = "SPI\0", .state = TESTSTATE_READY, .func = test_spi, },
};

static bool test_uart(const char *test_string, const uint8_t len)
{
	char uart_test_rx_buff[TEST_STRING_MAX_LEN] = {0};

	HAL_UART_Receive_IT(&huart6, (uint8_t *)uart_test_rx_buff, len);
	HAL_UART_Transmit(&huart6, (uint8_t *)test_string, len, TEST_TIMEOUT_TICKS);

	return (0 == strcmp(test_string, uart_test_rx_buff));
}

static bool test_i2c(const char *test_string, const uint8_t len)
{
	char i2c_rx_buff[TEST_STRING_MAX_LEN] = {0};

	HAL_I2C_Master_Transmit_IT(&hi2c1, hi2c2.Init.OwnAddress1, (uint8_t *)test_string, len);
	HAL_I2C_Slave_Receive(&hi2c2, (uint8_t *)i2c_rx_buff, len, TEST_TIMEOUT_TICKS);

	return (0 == strcmp(test_string, i2c_rx_buff));
}

static bool test_spi(const char *test_string, const uint8_t len)
{
	char spi_rx_buff[TEST_STRING_MAX_LEN] = {0};

	HAL_SPI_TransmitReceive(&hspi1, (uint8_t *)test_string, (uint8_t *)spi_rx_buff, len, TEST_TIMEOUT_TICKS);

	return (0 == strcmp(test_string, spi_rx_buff));
}

void test_listener_task_loop(void)
{
	static uint8_t server_uart_rx_buff[TEST_PACKET_SIZE_BYTES] = {0};
	static uint8_t server_uart_tx_buff[TEST_PACKET_SIZE_BYTES] = {0};

	static char debug_tx_buff[64] = {0};

	serial_print_line("UART Listener Task started.", 0);
	serial_print("Awaiting Test Requests.", 0);

	/* Infinite loop */
	for(;;)
	{
		explicit_bzero(server_uart_rx_buff, sizeof(server_uart_rx_buff));
		vTaskDelay(pdMS_TO_TICKS(10));

		if (HAL_OK == HAL_UART_Receive(&huart2, server_uart_rx_buff, 1, HAL_MAX_DELAY))
		{
			if (server_uart_rx_buff[0] == TEST_PACKET_START_BYTE_VALUE
				&& HAL_OK == HAL_UART_Receive(&huart2, server_uart_rx_buff+1, TEST_PACKET_SIZE_BYTES-1, HAL_MAX_DELAY)
				&& (TestPacketMsg_t)server_uart_rx_buff[TEST_PACKET_MSG_BYTE_OFFSET] == TESTMSG_NEWTEST
				&& server_uart_rx_buff[TEST_PACKET_SIZE_BYTES-1] == TEST_PACKET_END_BYTE_VALUE)
			{
				// copy to global test string buff
				test_string_len = server_uart_rx_buff[TEST_PACKET_STRING_LEN_OFFSET];
				explicit_bzero(test_string_buff, sizeof(test_string_buff));
				snprintf(test_string_buff, TEST_STRING_MAX_LEN, (char *)(server_uart_rx_buff+TEST_PACKET_STRING_HEAD_OFFSET));

				// confirm reception
				packet_init(server_uart_tx_buff);
				server_uart_tx_buff[TEST_PACKET_MSG_BYTE_OFFSET] = TESTMSG_ACK_DEVICE;
				HAL_UART_Transmit(&huart2, server_uart_tx_buff, sizeof(server_uart_tx_buff), HAL_MAX_DELAY);
				sprintf(debug_tx_buff, "\r\nDevice received test string: %s", test_string_buff);
				serial_print_line(debug_tx_buff, 0);

				// prepare results packet
				packet_init(server_uart_tx_buff);
				server_uart_tx_buff[TEST_PACKET_MSG_BYTE_OFFSET] = TESTMSG_RESULT;
				server_uart_tx_buff[TEST_PACKET_SELECTION_BYTE_OFFSET] = 0x00;

				// prepare test reference
				uint8_t test_selection_byte = server_uart_rx_buff[TEST_PACKET_SELECTION_BYTE_OFFSET];
				uint8_t ordered_test_count = 0;
				uint8_t completed_tests = 0;

				for (uint8_t i = 0; i < NUM_POSSIBLE_TESTS; i++)
				{
					if (0x01 & (test_selection_byte >> (uint8_t)i))
					{
					    sprintf(debug_tx_buff, "%s Test Ordered.", test_defs[i].name);
					    serial_print_line(debug_tx_buff, 0);
						ordered_test_count++;
						test_defs[i].state = TESTSTATE_PENDING;
					}
					else
					{
						test_defs[i].state = TESTSTATE_READY;
					}
				}


				while(completed_tests < ordered_test_count)
				{
				  vTaskDelay(pdMS_TO_TICKS(250));
					for (uint8_t i = 0; i < NUM_POSSIBLE_TESTS; i++)
					{
					switch(test_defs[i].state)
					{
					case TESTSTATE_SUCCESS:
						  server_uart_tx_buff[TEST_PACKET_SELECTION_BYTE_OFFSET]
						      |= (1 << (uint8_t)i);
						  sprintf(debug_tx_buff, "%s Test Success.", test_defs[i].name);
						  serial_print_line(debug_tx_buff, 0);
						  test_defs[i].state = TESTSTATE_READY;
						  completed_tests++;
						  break;
					case TESTSTATE_FAILURE:
						  sprintf(debug_tx_buff, "%s Test Failure.", test_defs[i].name);
						  serial_print_line(debug_tx_buff, 0);
						  test_defs[i].state = TESTSTATE_READY;
						  completed_tests++;
						  break;
					default:
						  break;
					  }
					}
				}
				HAL_UART_Transmit(&huart2, server_uart_tx_buff, sizeof(server_uart_tx_buff), HAL_MAX_DELAY);
				serial_print_line("Tests concluded.", 0);
				serial_print("Awaiting Test Requests.", 0);
		    }
			else
			{
				serial_print(".", 1);
			}
	  }
  }
}

void test_task_loop(TestDefinition_t *def)
{
	for(;;)
	{
		vTaskDelay(pdMS_TO_TICKS(100));

		if (def->state == TESTSTATE_PENDING)
		{
			def->state = TESTSTATE_BUSY;

			def->state = def->func(test_string_buff, test_string_len)
			? TESTSTATE_SUCCESS : TESTSTATE_FAILURE;
		}
	}
}
