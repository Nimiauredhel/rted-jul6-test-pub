#include "common.hpp"

/**
 * Global flag set by OS termination signals
 * and polled by functions to allow graceful termination.
 */
bool should_terminate = false;

/**
 * Hooks up OS signals to our custom handler.
 */
void initialize_signal_handler(void)
{
    should_terminate = false;

    struct sigaction action;
    bzero(&action, sizeof(action));
    action.sa_handler = signal_handler;
    sigaction(SIGINT, &action, NULL);
    sigaction(SIGTERM, &action, NULL);
    sigaction(SIGHUP, &action, NULL);
}

/**
 * Handles selected OS signals.
 * Termination signals are caught to set the should_terminate flag
 * which signals running functions that they should attempt graceful termination.
 */
void signal_handler(int signum)
{
    switch (signum)
    {
        case SIGINT:
        case SIGTERM:
        case SIGHUP:
            should_terminate = true;
            break;
    }
}

/**
 * Returns the seconds elapsed since a given clock value.
 * Used for timing operations!
 */
float seconds_since_clock(struct timespec start_clock)
{
    struct timespec now_clock;
    clock_gettime(CLOCK_MONOTONIC, &now_clock);
    float elapsed_float = (now_clock.tv_nsec - start_clock.tv_nsec) / 1000000000.0;
    elapsed_float += (now_clock.tv_sec - start_clock.tv_sec);
    return elapsed_float;
}
