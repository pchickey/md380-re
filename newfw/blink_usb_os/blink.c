
#include <stdint.h>
#include <string.h>
#include "FreeRTOS.h"
#include "task.h"
#include "led.h"
#include "usb_cdc.h"

static void sleep(uint32_t);

static void blink_main(void*);

int main (void) {
	xTaskCreate(blink_main, "blink", 256, NULL, 0, NULL);
	vTaskStartScheduler();
	for(;;);
}

static void led_set(int red, int green) {
	red_led(red);
	green_led(green);

	const char red_on[] = "red on,  ";
	const char red_off[] = "red off, ";
	const char green_on[] = "green on\n";
	const char green_off[] = "green off\n";
	#define SEND_STR(s) usb_cdc_write((void*)(s), strlen(s))
	SEND_STR(red?red_on:red_off);
	SEND_STR(green?green_on:green_off);
	#undef SEND_STR
}

static void blink_main(void* machtnichts) {

	led_setup();
	usb_cdc_init();

	for(;;) {

		led_set(1,1);

		sleep(500);

		led_set(0,0);

		sleep(500);

		led_set(1,0);

		sleep(500);

		led_set(0,0);

		sleep(500);
	}
}

static void sleep(uint32_t ms) {
	vTaskDelay(ms);
}

