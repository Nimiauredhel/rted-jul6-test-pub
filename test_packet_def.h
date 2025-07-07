#ifndef TEST_PACKET_DEF_H
#define TEST_PACKET_DEF_H

/* This ifdef allows the header to be used from both C and C++. */
#ifdef __cplusplus
extern "C" {
#endif

/***
 * Packet Structure:
 * [START(1)][MSG(1)][SELECTION(1)][STRLEN(1)][STRING(32)][PAD(1)][END(1)]
 * 0         1       2             3          4           36      37
 ***/

#define TEST_PACKET_SIZE_BYTES (38)
#define TEST_PACKET_START_BYTE_VALUE (123)
#define TEST_PACKET_END_BYTE_VALUE (132)
#define TEST_PACKET_STR_MAX_LEN (32)

#define TEST_PACKET_MSG_BYTE_OFFSET (1)
#define TEST_PACKET_SELECTION_BYTE_OFFSET (2)
#define TEST_PACKET_STRING_LEN_OFFSET (3)
#define TEST_PACKET_STRING_HEAD_OFFSET (4)

typedef enum PeripheralTestIdx
{
	TESTIDX_UART = 0,
	TESTIDX_I2C = 1,
	TESTIDX_SPI = 2,
} PeripheralTestIdx_t;

typedef enum TestPacketMsg
{
    TESTMSG_NONE = 0,
    TESTMSG_NEWTEST = 1,
    TESTMSG_ACK_SERVER = 2,
    TESTMSG_ACK_DEVICE = 3,
    TESTMSG_RESULT = 4,
} TestPacketMsg_t;

#ifdef __cplusplus
}
#endif

#endif

