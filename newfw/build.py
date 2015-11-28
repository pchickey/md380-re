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
              "usart.c",
              "usb_cdc.c",
              "usb/usb_bsp.c",
              "usb/usb_core.c",
              "usb/usb_dcd.c",
              "usb/usb_dcd_int.c",
              "usb/usbd_cdc_core.c",
              "usb/usbd_cdc_vcp.c",
              "usb/usbd_core.c",
              "usb/usbd_desc.c",
              "usb/usbd_ioreq.c",
              "usb/usbd_req.c",
              "usb/usbd_usr.c"],
    dependencies=[boot_lib, freertos_lib],
    extra_cflags="-DUSE_USB_OTG_FS")

blink_baremetal = STM32F4Bootloader("blink_baremetal",
    sources=["blink.c", "rcc.c", "gpio.c"],
    dependencies=boot_lib)

blink_bl = STM32F4Bootloader("blink_bl",
    sources=["blink.c", "rcc.c", "gpio.c"],
    dependencies=boot_lib)

blink_usb_os = STM32F4App("blink_usb_os",
    sources="blink.c",
    dependencies=[md380hw_lib])


ninja_build([ blink_baremetal, blink_bl, blink_usb_os ])
