#ifndef COMMON_H
#define COMMON_H

#ifndef _GNU_SOURCE
#define _GNU_SOURCE
#endif

#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <unistd.h>
#include <time.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <signal.h>

/**
 * Global flag set by OS termination signals
 * and polled by functions to allow graceful termination.
 */
extern bool should_terminate;

void initialize_signal_handler(void);
void signal_handler(int signum);
float seconds_since_clock(struct timespec start_clock);

#endif
