#include "led.h"
#include "gpio.h"

static void led_pin_setup(struct pin *pin) {
	pin_enable(pin);
	pin_set_mode(pin, PIN_MODE_OUTPUT);
	pin_set_otype(pin, PIN_TYPE_PUSHPULL);
	pin_set_ospeed(pin, PIN_SPEED_2MHZ);
	pin_set_pupd(pin, PIN_PUPD_NONE);
	pin_reset(pin);
}

void led_setup(void) {
	led_pin_setup(pin_e0);
	led_pin_setup(pin_e1);
}

void green_led(int on) {
	if (on) {
		pin_set(pin_e0);
	} else {
		pin_reset(pin_e0);
	}
}

void red_led(int on) {
	if (on) {
		pin_set(pin_e1);
	} else {
		pin_reset(pin_e1);
	}
}
