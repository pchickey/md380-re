# New firmware for MD380

Pat Hickey W7PCH, Oct 2015

This is a sandbox where I will build some new firmware for the MD380.

## Building

You will need to have [`ninja`][ninja] installed. Then, from this directory,
run:

```
$ ./build.py
```

This is the first time I've used the [Ninja Build System][ninja]. So far I like
it but there are some dragons in the build.py script yet. Please let me know if
you make a change to the build and things act unreasonably!

## Applications

By convention, application and library sources all get their own directory below
the top level. Applications will build binaries in their directory named
`$app.elf` and `$app.bin`.

### Blink

A proof of concept `blink_baremetal` application that will blink the red and
green leds on the radio. Red LED will blink at 1hz, Green LED will blink in sync
with the red LED every other cycle.

The blink application source is found in `blink_baremetal/blink.c`.

This application must be loaded into the root of the STM32 flash using the STM32
ROM DFU or the SWD interface, at address 0x08000000.

This application runs directly on the STM32 - no operating system is used, and
no peripherals are used besides the GPIO pins.

### Blink BL

A proof of concept `blink_bootloader` application that will blink the red and
green leds four times, then jump to a loaded radio "application". This is the
starting point for porting in a proper bootloader so we can replace the factory
bootloader.

The blink application source is found in `blink_bl/blink.c`.

This application must be loaded into the root of the STM32 flash using the STM32
ROM DFU or the SWD interface, at address 0x80000000.

This application runs directly on the STM32 - no operating system is used, and
no peripherals are used besides the GPIO pins.

### Blink USB OS

A port of the `blink` application to FreeRTOS, using the FreeRTOS scheduler to
sleep instead of spinning. Demonstrates that FreeRTOS is working properly, and
md380hw links properly. It will also create a USB CDC (virtual serial port)
device, to which it will print the state of the LED.

This application must be loaded into the root of the STM32 flash using the STM32
ROM DFU or the SWD interface, at address 0x8000c000. Use the "Blink BL"
application as a bootloader.

## Libraries

### Boot

The `boot` directory contains all of the support files required for creating a
bare metal application for the STM32F405.

This is the base dependency of all other libraries.

### FreeRTOS

The [FreeRTOS][] library is based on the 8.2.0 release. Only the basic kernel
sources and ARM\_CM4F portable code is checked in.

### md380hw

The `md380hw` is derived in part from the `hwf4` library, which was written at
[Galois][] in 2012-2013, mostly by James Bielman with some help from Pat Hickey.
It is distributed under a BSD3 license.

[ninja]: https://martine.github.io/ninja/
[FreeRTOS]: http://www.freertos.org/
[Galois]: http://galois.com
[SMACCMPilot]: http://smaccmpilot.org

