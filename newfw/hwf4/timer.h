/*
 * timer.h --- STM32F4 tick timer and PPM decoder.
 *
 * Copyright (C) 2012, Galois, Inc.
 * All Rights Reserved.
 *
 * This software is released under the "BSD3" license.  Read the file
 * "LICENSE" for more information.
 */

#ifndef __hwf4_timer_h
#define __hwf4_timer_h

#include <stdint.h>
#include <sys/types.h>

#ifdef __cplusplus
extern "C" {
#endif

/** Maximum number of PPM channels to decode. */
#define PPM_MAX_CHANNELS  12

/**
 * An unsigned integer count of ticks since timer initialization.
 *
 * This value will not wrap for over 500,000 years, so it is pretty
 * safe to rely on the fact that it will not wrap around.
 */
typedef uint64_t timer_tick_t;

/** Initialize the timer subsystem. */
void timer_init(void);

/**
 * Return the number of microsecond ticks since timer initialization.
 *
 * This is safe to call from an ISR.
 */
uint64_t timer_get_ticks(void);

/**
 * Delay for "delay" milliseconds.  This is implemented using an RTOS
 * system call, so it will not block other threads from running.
 *
 * @param delay wait delay in milliseconds
 */
void timer_msleep(uint32_t delay);

/**
 * Busy wait for "delay" (up to 65535) microseconds.  This should only
 * be used for short delays as we prevent FreeRTOS from context
 * switching during the delay.
 *
 * @param delay wait delay in microseconds
 */
void timer_usleep(uint16_t delay);

/**
 * Return true if there is valid PPM available.
 */
bool timer_is_ppm_valid(void);

/**
 * Read one channel from the last PPM sample buffer.
 *
 * @param ch channel number to read
 * @param result pointer to result value
 * @returns true if the channel was valid, false if invalid
 */
bool timer_get_ppm_channel(int ch, uint16_t *result);

/**
 * Get the last PPM sample buffer and number of values.
 *
 * @param buf buffer to copy sample data to
 * @param len length of "buf"
 * @param ticks output timestamp of PPM sample if non-NULL
 * @returns the number of samples copied to "buf"
 */
size_t timer_get_ppm(uint16_t *buf, size_t len, timer_tick_t *ticks);

/**
 * Clear the PPM sample buffer.
 *
 * After calling this function, "timer_is_ppm_valid" will return false
 * (and reading channels will fail) until the next sample arrives.
 */
void timer_clear_ppm(void);

#ifdef __cplusplus
}
#endif

#endif  /* !defined __hwf4_timer_h */
