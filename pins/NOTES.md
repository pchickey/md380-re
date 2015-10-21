
## STM32F405

### Clocks:

PIN 8 - OSC32 IN - low frequency RTC oscillator? not enabled by bootloader

PIN 9 - OSC32 OUT - low frequency RTC oscillator? not enabled by bootloader

PIN 12 - OSC IN  - 8MHz crystal oscillator

PIN 13 - OSC OUT - 8MHz crystal oscillator

-----

### Debug and boot control:

PIN 50 - VDD   - Test Pad

PIN 72 - SWDIO - Test Pad

PIN 76 - SWCLK - Test Pad

PIN 94 - BOOT0 - Test Pad

![Labeled test pads](https://github.com/pchickey/md380-re/raw/master/teardown/stm32%20closeup%20annotated.jpg)

------

### LEDs:

PIN 97 - PE0 - Green LED

PIN 98 - PE1 - Red LED

Both LEDs are driven by a transistor located right next to them on the circuit
board. Use these pins as regular push-pull outputs to drive the transistor gate.
