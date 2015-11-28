
#include <stdint.h>
#include "gpio.h"
#include "stm32f4xx.h"

static void led_setup(void);
static void green_led(int);
static void red_led(int);
static void sleep(uint32_t);
static void jump_to_app(void);

int main(void) {

	led_setup();

	for(int i = 0; i < 4; i++) {

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
	jump_to_app();
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
		/* At 168MHz sysclock, and -Os, this loop will take 4 instructions.
		 * Confirmed with measurement. */
		for (uint32_t j = 0; j < 168000; j += 4) {
			asm volatile ("mov r0,r0");
		}
	}
}


static void do_jump(uint32_t stacktop, uint32_t entrypoint)
{
	asm volatile(
			"msr msp, %0	\n"
			"bx %1	\n"
			: : "r" (stacktop), "r" (entrypoint) : );
	for(;;); // Unreachable
}

#define APP_LOAD_ADDRESS 0x0800C000

static void jump_to_app(void) {

	const uint32_t *app_base = (const uint32_t *)APP_LOAD_ADDRESS;

	// Check that flash has been programmed
	if (app_base[0] == 0xffffffff)
		goto fail;

	// Check that reset vector is somewhere in the app's boundaries
	if (app_base[1] < APP_LOAD_ADDRESS)
		goto fail;
	if (app_base[1] >= (APP_LOAD_ADDRESS + (1024 * 1024)))
			goto fail;

	// LEDs to known state
	green_led(1);
	red_led(0);

	// Set vector table address
	SCB->VTOR = APP_LOAD_ADDRESS;

	// Jump to the application
	do_jump(app_base[0], app_base[1]);

	// LEDs indicate error
	fail:
	green_led(0);
	red_led(0);
	for(;;);
}
