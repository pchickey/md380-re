
#include <stdint.h>
#include "FreeRTOS.h"
#include "task.h"
#include "led.h"

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

