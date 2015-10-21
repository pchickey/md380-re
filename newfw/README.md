# New firmware for MD380

Pat Hickey W7PCH, Oct 2015

This is a sandbox where I will build some new firmware for the MD380.

## Building

You will need to have [`ninja`][ninja] installed. Then, from this directory,
run:

```
$ ./build.py
$ ninja
```

This is the first time I've used the [Ninja Build System][ninja]. So far I like
it but there are sure to be ways that I've used it which are not super great so
far. I will eventaully work something out so there are not two steps to build.

## Applications

### Blink

A proof of concept `blink` application that will blink the red and green leds on
the radio. Red LED will blink at 1hz, Green LED will blink in sync with the red
LED every other cycle.

The blink application source is found in `blink/blink.c`.

This application must be loaded into the root of the STM32 flash using the STM32
ROM DFU or the SWD interface.

This application runs directly on the STM32 - no operating system is used, and
no peripherals are used besides the GPIO pins.

Binaries are built in the blink subdirectory:

```
blink/blink.elf
blink/blink.bin
```

### Blink2

A port of the `blink` application to FreeRTOS, using the FreeRTOS scheduler to
sleep instead of spinning. Demonstrates that FreeRTOS is working properly, and
nothing else.

## Libraries

### Boot

The `boot` directory contains all of the support files required for creating a
bare metal application for the STM32F405.

This is the base dependency of all other libraries.

### FreeRTOS

The [FreeRTOS][] library is based on the 8.2.0 release. Only the basic kernel
sources and ARM\_CM4F portable code is checked in.

### HWF4

The HWF4 library was written at [Galois][] in 2012-2013, mostly by James Bielman
with some help from Pat Hickey. It is distributed under a BSD3 license.

It contains a bunch of peripheral drivers we needed for the original PX4FMUv1
autopilot, during early stages of the [SMACCMPilot][] project.  Many of these
depend on FreeRTOS. Currently, only the tiny subset that do not depend on the OS
(RCC and GPIO) will actually build. The rest of them will be brought up shortly.

[ninja]: https://martine.github.io/ninja/
[FreeRTOS]: http://www.freertos.org/
[Galois]: http://galois.com
[SMACCMPilot]: http://smaccmpilot.org

