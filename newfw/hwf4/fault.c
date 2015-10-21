/*
 * fault.c --- Cortex-M3 fault handling.
 *
 * Copyright (C) 2012, Galois, Inc.
 * All Rights Reserved.
 *
 * This software is released under the "BSD3" license.  Read the file
 * "LICENSE" for more information.
 */

#include <stdint.h>
#include <stm32f4xx.h>

void fault_get_regs(uint32_t *sp)
{
  /* These are volatile to try and prevent the compiler/linker
   * optimising them away as the variables never actually get used.
   * If the debugger won't show the values of the variables, make them
   * global by moving their declaration outside of this function. */
  volatile uint32_t r0;
  volatile uint32_t r1;
  volatile uint32_t r2;
  volatile uint32_t r3;
  volatile uint32_t r12;
  volatile uint32_t lr;
  volatile uint32_t pc;
  volatile uint32_t psr;

  /* The useless expressions after the assignments are to prevent GCC
   * from warning us that these variables are assigned but not
   * used. */
  r0  = sp[0]; r0;
  r1  = sp[1]; r1;
  r2  = sp[2]; r2;
  r3  = sp[3]; r3;
  r12 = sp[4]; r12;
  lr  = sp[5]; lr;
  pc  = sp[6]; pc;
  psr = sp[7]; psr;

  for (;;)
    ;
}

void HardFault_Handler(void) __attribute__((naked));

void HardFault_Handler(void)
{
  __asm volatile (
    "tst lr, #4\n"
    "ite eq\n"
    "mrseq r0, msp\n"
    "mrsne r0, psp\n"
    "ldr r1, [r0, #24]\n"
    "ldr r2, handler2_address_const\n"
    "bx r2\n"
    "handler2_address_const: .word fault_get_regs\n"
    );
}
