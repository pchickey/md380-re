
#include <stdint.h>
#include "FreeRTOS.h"
#include "task.h"
#include "gpio.h"

static void led_setup(void);
static void green_led(int);
static void red_led(int);
static void sleep(uint32_t);

static void blink_main(void*);


int main (void) {
	xTaskCreate(blink_main, "blink", 256, NULL, 0, NULL);
	vTaskStartScheduler();
	for(;;);
}



static void blink_main(void* machtnichts) {

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

static void sleep(uint32_t ms) {
	vTaskDelay(ms);
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

static void spin_sleep(uint32_t ms) {
	for (uint32_t i = 0; i < ms; i++) {
		/* At 168MHz sysclock, and -Os, this loop will take 4 instructions.
		 * Confirmed with measurement. */
		for (uint32_t j = 0; j < 168000; j += 4) {
			asm volatile ("mov r0,r0");
		}
	}
}

// for real hwf4 support, we'll move this into a library:

void panic(void) {
	led_setup();
	green_led(0);
	for(;;){
		red_led(1);
		spin_sleep(200);
		red_led(0);
		spin_sleep(200);
	}
}

void vApplicationStackOverflowHook(void) {
	panic();
}

void vApplicationMallocFailedHook(void) {
	panic();
}
