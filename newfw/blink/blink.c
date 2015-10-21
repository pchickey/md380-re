
#include <stdint.h>
#include "gpio.h"

static void led_setup(void);
static void green_led(int);
static void red_led(int);
static void sleep(uint32_t);

int main(void) {

	led_setup();

	for(;;) {

		red_led(1);
		green_led(1);

		sleep(500);

		red_led(0);
		green_led(0);

		sleep(500);

		red_led(1);

		sleep(500);

		red_led(0);

		sleep(500);
	}
}


static void led_pin_setup(struct pin *pin) {
	pin_enable(pin);
	pin_set_mode(pin, PIN_MODE_OUTPUT);
	pin_set_otype(pin, PIN_TYPE_PUSHPULL);
	pin_set_ospeed(pin, PIN_SPEED_2MHZ);
	pin_set_pupd(pin, PIN_PUPD_NONE);
	pin_reset(pin);
}

static void led_setup(void) {
	led_pin_setup(pin_e0);
	led_pin_setup(pin_e1);
}

static void green_led(int on) {
	if (on) {
		pin_set(pin_e0);
	} else {
		pin_reset(pin_e0);
	}
}

static void red_led(int on) {
	if (on) {
		pin_set(pin_e1);
	} else {
		pin_reset(pin_e1);
	}
}

static void sleep(uint32_t ms) {
	for (uint32_t i = 0; i < ms; i++) {
		/* I guess-and-checked this loop bound by measuring a sleep(1)
		 * on an oscope - it will definitely change if we change the
		 * chip clock, optimization level (currently -O0), etc.
		 * this code is just for basic sanity checking, don't use it for
		 * anything important obviously */
		for (uint32_t j = 0; j < 6000; j++) {
			asm volatile ("mov r0,r0");
		}
	}
}

