/*
 * interrupt.c --- STM32F4 NVIC driver.
 *
 * Copyright (C) 2012, Galois, Inc.
 * All Rights Reserved.
 *
 * This software is released under the "BSD3" license.  Read the file
 * "LICENSE" for more information.
 */

#include <stm32f4xx.h>
#include <core_cm4.h>

#include "interrupt.h"


void interrupt_enable(enum IRQn n) {
    NVIC_EnableIRQ(n);
}

void interrupt_disable(enum IRQn n) {
    NVIC_DisableIRQ(n);
}

void interrupt_set_priority(enum IRQn n, uint8_t priority) {
    if (n < 0) {
        /* Negative IRQn for the cortex-m exceptions. */
        SCB->SHP[((uint32_t)(n) & 0xf) - 4] = priority << 4;
    } else {
        /* Positive IRQn for the system exceptions */
        NVIC->IP[(uint32_t)(n)] = priority << 4;
    }
}

