/*
 * peripheral_tests.h
 *
 *  Created on: Jul 7, 2025
 *      Author: mickey
 */

#ifndef INC_PERIPHERAL_TESTS_H_
#define INC_PERIPHERAL_TESTS_H_

#include "main.h"
#include "packet_ops.h"

#define TEST_STRING_MAX_LEN (TEST_PACKET_STR_MAX_LEN)
#define NUM_POSSIBLE_TESTS (3)
#define TEST_TIMEOUT_TICKS (1000)

typedef volatile enum PeripheralTestState
{
	TESTSTATE_READY = 0,
	TESTSTATE_PENDING = 1,
	TESTSTATE_BUSY = 2,
	TESTSTATE_SUCCESS = 3,
	TESTSTATE_FAILURE = 4,
} PeripheralTestState_t;

typedef struct TestDefinition
{
	const char name[16];
	volatile PeripheralTestState_t state;
	bool (*func)(const char*, const uint8_t);
} TestDefinition_t;

extern I2C_HandleTypeDef hi2c1;
extern I2C_HandleTypeDef hi2c2;
extern SPI_HandleTypeDef hspi1;
extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart6;

extern volatile uint8_t test_string_len;
extern char test_string_buff[TEST_STRING_MAX_LEN];

extern TestDefinition_t test_defs[NUM_POSSIBLE_TESTS];

void test_listener_task_loop(void);
void test_task_loop(TestDefinition_t *def);

#endif /* INC_PERIPHERAL_TESTS_H_ */
