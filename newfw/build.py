#!/usr/bin/env python

from build.helpers import *

freertos_lib = C_library(path="freertos/FreeRTOSV8.2.0/Source",
    sources=["tasks.c", "queue.c", "list.c",
             "portable/GCC/ARM_CM4F/port.c",
             "portable/MemMang/heap_1.c"
             ],
    extra_cflags="-I./freertos/FreeRTOSV8.2.0/Source/include " +
                 "-I./freertos/FreeRTOSV8.2.0/Source/portable/GCC/ARM_CM4F " +
                 "-I./freertos/")

md380hw_lib = C_library(path="md380hw",
    sources=[ "eeprom.c",
               "fault.c",
               "gpio.c",
               "i2c.c",
               "interrupt.c",
               "led.c",
               "rcc.c",
               "spi.c",
               "usart.c"],
    dependencies=[boot_lib, freertos_lib])

blink = STM32F4App("blink",
    sources=["blink.c", "rcc.c", "gpio.c"],
    dependencies=boot_lib)


blink2 = STM32F4App("blink2",
    sources="blink.c",
    dependencies=[md380hw_lib])


ninja_build([ blink, blink2 ])
