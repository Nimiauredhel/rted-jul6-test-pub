#include "packet_ops.h"

void packet_init(uint8_t *buff)
{
    explicit_bzero(buff, sizeof(buff));
    buff[0] = TEST_PACKET_START_BYTE_VALUE;
    buff[TEST_PACKET_MSG_BYTE_OFFSET] = TESTMSG_NEWTEST;
    buff[TEST_PACKET_SIZE_BYTES-1] = TEST_PACKET_END_BYTE_VALUE;
}
