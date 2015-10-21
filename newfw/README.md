# New firmware for MD380

This is a sandbox where I will build some new firmware for the MD380.

## Building

You will need to have [`ninja`][ninja] installed. Then, from this directory,
run:

```
$ ./build.py
$ ninja
```

This is the first time I've used the [Ninja Build System][ninja]. So far I like
it but there are sure to be things that are not very likable.

## Applications

### Blink

At the moment, there is one application, a proof of concept `blink` application
that will blink the red and green leds on the radio.

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

## Libraries

### Bare Metal Build

The `bare_metal_build` directory contains all of the support files required for
linking a bare metal application for the STM32F405.

This will eventaully be deprecated once I get FreeRTOS running in a satisfactory
way.

### HWF4

The HWF4 library was written at Galois in 2012-2013, mostly by James Bielman
with some help from myself. It is available under a BSD3 license.

It contains a bunch of peripheral drivers we needed for the original PX4FMUv1
autopilot.  Many of these depend on FreeRTOS. Currently, only the tiny subset
that do not depend on the OS (RCC and GPIO) will actually build. The rest of
them will be brought up shortly.


[ninja]: https://martine.github.io/ninja/
