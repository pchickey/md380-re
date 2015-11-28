/*
 * hw_config.h
 * Tailored to MD380 bootloader
 * Pat Hickey, 27 Nov 15
 */

#ifndef HW_CONFIG_H_
#define HW_CONFIG_H_


# define APP_LOAD_ADDRESS               0x0800c000
# define BOOTLOADER_DELAY               5000
# define BOARD_FMU
# define INTERFACE_USB                  1
# define USBDEVICESTRING                "PX4 BL MD380"
# define USBPRODUCTID                   0x0380
# define BOOT_DELAY_ADDRESS             0x000001a0

# define BOARD_TYPE                     380
# define BOARD_FLASH_SECTORS            11
# define BOARD_FLASH_SIZE               (1024 * 1024)

# define OSC_FREQ                       8

# define BOARD_PIN_LED_ACTIVITY         GPIO0
# define BOARD_PIN_LED_BOOTLOADER       GPIO1
# define BOARD_PORT_LEDS                GPIOE
# define BOARD_CLOCK_LEDS               RCC_AHB1ENR_IOPEEN
# define BOARD_LED_ON                   gpio_set
# define BOARD_LED_OFF                  gpio_clear

#endif // HW_CONFIG_H_
