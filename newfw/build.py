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

blink_sideload= MD380SideloadApp("blink_sideload",
    sources="blink.c",
    dependencies=[md380hw_lib])


class LibOpenCM3(object):
    def __init__(self, path):
        self.name = "libopencm3"
        self.path = path
        self.dependencies = []

    def collect_dependencies(self):
        return self.dependencies

    def include_path(self):
        ownpath = ["-I" + self.path + "/include"]
        deppath = []
        for d in self.collect_dependencies():
            deppath += d.include_path()
        return ownpath + deppath


    def cflags(self):
        return ["-DSTM32F4"]

    def build(self, n):
        # XXX put logic here to make -C {self.path} lib
        pass

    def outputs(self, recursive = False):
        return [ self.path + "/lib/libopencm3_stm32f4.a" ]

libopencm3 = LibOpenCM3("./libopencm3")

class LibOpenCM3Bootloader(STM32F4Bootloader):
    def linker_script(self):
        return "px4_bl/stm32f4.ld"

px4_bl = LibOpenCM3Bootloader("px4_bl",
    sources=[ "main_f4.c",
              "bl.c",
              "cdcacm.c"],
    dependencies=[libopencm3])


ninja_build([ blink_baremetal, blink_bl, blink_usb_os, blink_sideload ])
