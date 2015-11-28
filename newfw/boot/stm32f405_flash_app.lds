/****************************************************************************
 * configs/px4fmu/common/ld.script
 *
 *   Copyright (C) 2011 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name NuttX nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/* The STM32F405 has 1024Kb of FLASH beginning at address 0x0800:0000 and
 * 192Kb of SRAM. SRAM is split up into three blocks:
 *
 * 1) 112Kb of SRAM beginning at address 0x2000:0000
 * 2)  16Kb of SRAM beginning at address 0x2001:c000
 * 3)  64Kb of TCM SRAM beginning at address 0x1000:0000
 *
 * When booting from FLASH, FLASH memory is aliased to address 0x0000:0000
 * where the code expects to begin execution by jumping to the entry point in
 * the 0x0800:0000 address range.
 *
 * HWF4: In the original linker script, the first 0x4000 of flash was
 * reserved for the bootloader.  For now, we'll place the application
 * at the start of flash until we start using a boot loader again.
 */

MEMORY
{
  flash (rx)   : ORIGIN = 0x0800C000, LENGTH = 976K
  sram (rwx)   : ORIGIN = 0x20000000, LENGTH = 128K
  ccsram (rwx) : ORIGIN = 0x10000000, LENGTH = 64K
}

/* Top of the user mode stack. */
_estack = 0x20020000;           /* top of 128KiB of SRAM */

/* Error in the linker if heap and stack don't fit. */
_min_heap_size = 0;
_min_stack_size = 0x400;

OUTPUT_ARCH(arm)

ENTRY(Reset_Handler)

/*
 * Ensure that abort() is present in the final object.  The exception handling
 * code pulled in by libgcc.a requires it (and that code cannot be easily avoided).
 */
/* EXTERN(abort) */

SECTIONS
{
  /* Place startup code at start of flash. */
  .isr_vector : {
    . = ALIGN(4);
    KEEP(*(.isr_vector))
    . = ALIGN(4);
  } > flash

  /* Program code and constant data. */
  .text : {
    *(.text .text.*)
    *(.fixup)
    *(.gnu.warning)
    *(.rodata .rodata.*)
    *(.glue_7)
    *(.glue_7t)
    *(.eh_frame)
    *(.gnu.linkonce.t.*)
    *(.got)
    *(.gcc_except_table)
    *(.gnu.linkonce.r.*)

    KEEP(*(.init))
    KEEP(*(.fini))

    . = ALIGN(4);
    _etext = ABSOLUTE(.);
  } > flash

  .ARM.extab : {
    *(.ARM.extab*)
  } > flash

  __exidx_start = ABSOLUTE(.);
  .ARM.exidx : {
    *(.ARM.exidx*)
  } > flash
  __exidx_end = ABSOLUTE(.);

  .preinit_array : {
    PROVIDE_HIDDEN(__preinit_array_start = .);
    KEEP(*(.preinit_array))
    PROVIDE_HIDDEN(__preinit_array_end = .);
  } > flash

  .init_array : {
    PROVIDE_HIDDEN(__init_array_start = .);
    KEEP(*(.init_array))
    PROVIDE_HIDDEN(__init_array_end = .);
  } > flash

  .ctors : {
    KEEP(*crtbegin.o(.ctors))
    KEEP(*crtbegin?.o(.ctors))
    KEEP(*(EXCLUDE_FILE(*crtend.o *crtend?.o) .ctors))
    KEEP(*(SORT(.ctors.*)))
    KEEP(*(.ctors))
  } > flash

  .data : {
    _sidata = LOADADDR(.data);
    _sdata = ABSOLUTE(.);
    *(.data .data.*)
    *(.gnu.linkonce.d.*)
    CONSTRUCTORS
    _edata = ABSOLUTE(.);
  } > ccsram AT > flash

  . = ALIGN(4);
  .bss : {
    _sbss = ABSOLUTE(.);
    __bss_start__ = ABSOLUTE(_sbss);
    *(.bss .bss.*)
    *(.gnu.linkonce.b.*)
    *(COMMON)
    . = ALIGN(4);
    _ebss = ABSOLUTE(.);
    __bss_end__ = ABSOLUTE(_ebss);
  } > sram

  ._user_heap_stack : {
    . = ALIGN(4);
    PROVIDE(end = .);
    PROVIDE(_end = .);
    . = . + _min_heap_size;
    . = . + _min_stack_size;
    . = ALIGN(4);
  } > ccsram

  /* Stabs debugging sections. */
  .stab 0 : { *(.stab) }
  .stabstr 0 : { *(.stabstr) }
  .stab.excl 0 : { *(.stab.excl) }
  .stab.exclstr 0 : { *(.stab.exclstr) }
  .stab.index 0 : { *(.stab.index) }
  .stab.indexstr 0 : { *(.stab.indexstr) }
  .comment 0 : { *(.comment) }
  .debug_abbrev 0 : { *(.debug_abbrev) }
  .debug_info 0 : { *(.debug_info) }
  .debug_line 0 : { *(.debug_line) }
  .debug_pubnames 0 : { *(.debug_pubnames) }
  .debug_aranges 0 : { *(.debug_aranges) }

  /DISCARD/ : {
    libc.a (*)
    libm.a (*)
    libgcc.a (*)
  }
}
