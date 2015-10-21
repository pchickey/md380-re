/*
 * interrupt.h --- STM32F NVIC/interrupt configuration.
 *
 * Copyright (C) 2012, Galois, Inc.
 * All Rights Reserved.
 *
 * This software is released under the "BSD3" license.  Read the file
 * "LICENSE" for more information.
 */

#ifndef __interrupt_h
#define __interrupt_h

#ifdef __cplusplus__
extern C {
#endif

#include <stdint.h>
#include <FreeRTOSConfig.h>
#include "irq.h"

/**
 * Enable an interrupt
 */
void interrupt_enable(enum IRQn);

/**
 * Disable an interrupt.
 */
void interrupt_disable(enum IRQn);

/*
 * Interrupt Priorities
 *
 * Because of the way the Cortex-M4 NVIC interacts with FreeRTOS, it
 * is very important to set up proper interrupt priorities for any
 * ISRs that use FreeRTOS APIs (of the "xxxFromISR" variety).  The
 * default priority will not do!
 *
 * See the FreeRTOS documentation for the details:
 *
 *   http://www.freertos.org/RTOS-Cortex-M3-M4.html
 */

/** The lowest interrupt priority. */
#define INTERRUPT_PRIORITY_LOWEST   15

/**
 * The highest interrupt priority that can call FreeRTOS APIs.  All
 * priorities from here to "INTERRUPT_PRIORITY_HIGHEST" are safe to
 * use with ISRs that call into FreeRTOS.
 */
#define INTERRUPT_PRIORITY_FREERTOS_SAFE \
  ((configMAX_SYSCALL_INTERRUPT_PRIORITY) >> 4)

/** The highest interrupt priority. */
#define INTERRUPT_PRIORITY_HIGHEST  0

/**
 * Set the priority of an interrupt.
 *
 * "priority" is an unshifted priority number, between
 * INTERRUPT_PRIORITY_LOWEST and INTERRUPT_PRIORITY_HIGHEST,
 * inclusive.
 */
void interrupt_set_priority(enum IRQn, unsigned char priority);

#ifdef __cplusplus__
}
#endif

#endif /* __interrupt.h */
