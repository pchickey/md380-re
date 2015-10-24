/*
 * gpio.h --- STM32 GPIO driver.
 *
 * Copyright (C) 2012, Galois, Inc.
 * All Rights Reserved.
 *
 * This software is released under the "BSD3" license.  Read the file
 * "LICENSE" for more information.
 */

#ifndef __gpio_h
#define __gpio_h

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>

#include "rcc.h"


/* GPIO Bank Interface ********************************************************/

struct gpio;

/**
 * Enable the clock for the gpio bank
 */
void gpio_enable(struct gpio *gpio);

/**
 * Reset a pin state in the mode register.  The input is a mask for the portions
 * of the register to be reset.
 */
void gpio_reset_moder(struct gpio *gpio, uint32_t val);

/**
 * Update the value of the mode register.
 */
void gpio_update_moder(struct gpio *gpio, uint32_t val);

/**
 * Reset a pin state in the output type register.  The input is a mask for the
 * portions of the register to be reset.
 */
void gpio_reset_otyper(struct gpio *gpio, uint32_t val);

/**
 * Update the value of the output type register.
 */
void gpio_update_otyper(struct gpio *gpio, uint32_t val);

/**
 * Reset a pin state in the output speed register.  The input is a mask for the
 * portions of the register to be reset.
 */
void gpio_reset_ospeedr(struct gpio *gpio, uint32_t val);

/**
 * Update the value of the output speed register.
 */
void gpio_update_ospeedr(struct gpio *gpio, uint32_t val);

/**
 * Reset a pin state in the output pupd register.  The input is a mask for the
 * portions of the register to be reset.
 */
void gpio_reset_pupdr(struct gpio *gpio, uint32_t val);

/**
 * Update the value of the pull up/down register.
 */
void gpio_update_pupdr(struct gpio *gpio, uint32_t val);

/**
 * Get the value of the input register.
 */
uint32_t gpio_get_idr(struct gpio *gpio);

/**
 * Update the value of the output data register.
 */
void gpio_modify_odr(struct gpio *gpio, uint32_t val);

void gpio_reset_afr(struct gpio *gpio, int lh, uint32_t val);

void gpio_modify_afr(struct gpio *gpio, int lh, uint32_t val);

void gpio_set_bsrrl(struct gpio *gpio, uint32_t val);

void gpio_set_bsrrh(struct gpio *gpio, uint32_t val);


/* GPIO Pin Interface *********************************************************/

struct pin;

/**
 * Enable the gpio bank containing a pin.
 */
void pin_enable(struct pin *pin);

enum pin_mode {
    PIN_MODE_INPUT  = 0x00,
    PIN_MODE_OUTPUT = 0x01,
    PIN_MODE_AF     = 0x02,
    PIN_MODE_ANALOG = 0x03
};

/**
 * Set the mode of the pin.
 *
 * NOTE: The mode register for each of the 16 pins takes up two bits of the
 * 32-bit mode register in the gpio bank.
 */
void pin_set_mode(struct pin *pin, enum pin_mode mode);

enum pin_type {
    PIN_TYPE_PUSHPULL  = 0x0,
    PIN_TYPE_OPENDRAIN = 0x1
};

/**
 * Set the output type of the pin.
 */
void pin_set_otype(struct pin *pin, enum pin_type type);

enum pin_speed {
    PIN_SPEED_2MHZ   = 0x00,
    PIN_SPEED_25MHZ  = 0x01,
    PIN_SPEED_50MHZ  = 0x02,
    PIN_SPEED_100MHZ = 0x03
};

/**
 * Set the output speed of the pin.
 */
void pin_set_ospeed(struct pin *pin, enum pin_speed speed);

enum pin_pupd {
    PIN_PUPD_NONE = 0x00,
    PIN_PUPD_UP   = 0x01,
    PIN_PUPD_DOWN = 0x02
};

/**
 * Set the pu/pd for the pin.
 */
void pin_set_pupd(struct pin *pin, enum pin_pupd pupd);

enum pin_af {
    PIN_AF0  = 0x0,
    PIN_AF1  = 0x1,
    PIN_AF2  = 0x2,
    PIN_AF3  = 0x3,
    PIN_AF4  = 0x4,
    PIN_AF5  = 0x5,
    PIN_AF6  = 0x6,
    PIN_AF7  = 0x7,
    PIN_AF8  = 0x8,
    PIN_AF9  = 0x9,
    PIN_AF10 = 0xa,
    PIN_AF11 = 0xb,
    PIN_AF12 = 0xc,
    PIN_AF13 = 0xd,
    PIN_AF14 = 0xe,
    PIN_AF15 = 0xf
};

#define PIN_AF_RTC_50HZ         PIN_AF0
#define PIN_AF_MC0              PIN_AF0
#define PIN_AF_TAMPER           PIN_AF0
#define PIN_AF_SWJ              PIN_AF0
#define PIN_AF_TRACE            PIN_AF0

#define PIN_AF_TIM1             PIN_AF1
#define PIN_AF_TIM2             PIN_AF1

#define PIN_AF_TIM3             PIN_AF2
#define PIN_AF_TIM4             PIN_AF2
#define PIN_AF_TIM5             PIN_AF2

#define PIN_AF_TIM8             PIN_AF3
#define PIN_AF_TIM9             PIN_AF3
#define PIN_AF_TIM10            PIN_AF3
#define PIN_AF_TIM11            PIN_AF3

#define PIN_AF_I2C1             PIN_AF4
#define PIN_AF_I2C2             PIN_AF4
#define PIN_AF_I2C3             PIN_AF4
/* Since all I2C use the same PIN_AFn, generic name for convenience. */
#define PIN_AF_I2C              PIN_AF4

#define PIN_AF_SPI1             PIN_AF5
#define PIN_AF_SPI2             PIN_AF5

#define PIN_AF_SPI3             PIN_AF6

#define PIN_AF_USART1           PIN_AF7
#define PIN_AF_USART2           PIN_AF7
#define PIN_AF_USART3           PIN_AF7
#define PIN_AF_I2S3ext          PIN_AF7

#define PIN_AF_UART4            PIN_AF8
#define PIN_AF_UART5            PIN_AF8
#define PIN_AF_USART6           PIN_AF8

#define PIN_AF_CAN1             PIN_AF9
#define PIN_AF_CAN2             PIN_AF9
#define PIN_AF_TIM12            PIN_AF9
#define PIN_AF_TIM13            PIN_AF9
#define PIN_AF_TIM14            PIN_AF9

#define PIN_AF_OTG_FS           PIN_AF10
#define PIN_AF_OTG_HS           PIN_AF10

#define PIN_AF_ETH              PIN_AF11

#define PIN_AF_FSMC             PIN_AF12
#define PIN_AF_OTG_HS_FS        PIN_AF12
#define PIN_AF_SDIO             PIN_AF12

#define PIN_AF_DCMI             PIN_AF13

#define PIN_AF_EVENTOUT         PIN_AF15

/**
 * Enable an alternate function for a pin.
 */
void pin_set_af(struct pin *pin, enum pin_af af);

/**
 * Reset a pin.
 */
void pin_reset(struct pin *pin);

/**
 * Set a pin.
 */
void pin_set(struct pin *pin);

/**
 * Toggle an output pin.
 */
void pin_toggle(struct pin *pin);

/**
 * Read the value of an input pin.
 */
bool pin_read(struct pin *pin);

/* GPIOA **********************************************************************/

extern struct gpio _gpio_a;
#define gpio_a (&_gpio_a)

extern struct pin _pin_a0;
#define pin_a0 (&_pin_a0)

extern struct pin _pin_a1;
#define pin_a1 (&_pin_a1)

extern struct pin _pin_a2;
#define pin_a2 (&_pin_a2)

extern struct pin _pin_a3;
#define pin_a3 (&_pin_a3)

extern struct pin _pin_a4;
#define pin_a4 (&_pin_a4)

extern struct pin _pin_a5;
#define pin_a5 (&_pin_a5)

extern struct pin _pin_a6;
#define pin_a6 (&_pin_a6)

extern struct pin _pin_a7;
#define pin_a7 (&_pin_a7)

extern struct pin _pin_a8;
#define pin_a8 (&_pin_a8)

extern struct pin _pin_a9;
#define pin_a9 (&_pin_a9)

extern struct pin _pin_a10;
#define pin_a10 (&_pin_a10)

extern struct pin _pin_a11;
#define pin_a11 (&_pin_a11)

extern struct pin _pin_a12;
#define pin_a12 (&_pin_a12)

extern struct pin _pin_a13;
#define pin_a13 (&_pin_a13)

extern struct pin _pin_a14;
#define pin_a14 (&_pin_a14)

extern struct pin _pin_a15;
#define pin_a15 (&_pin_a15)


/* GPIOB **********************************************************************/

extern struct gpio _gpio_b;
#define gpio_b (&_gpio_b)

extern struct pin _pin_b0;
#define pin_b0 (&_pin_b0)

extern struct pin _pin_b1;
#define pin_b1 (&_pin_b1)

extern struct pin _pin_b2;
#define pin_b2 (&_pin_b2)

extern struct pin _pin_b3;
#define pin_b3 (&_pin_b3)

extern struct pin _pin_b4;
#define pin_b4 (&_pin_b4)

extern struct pin _pin_b5;
#define pin_b5 (&_pin_b5)

extern struct pin _pin_b6;
#define pin_b6 (&_pin_b6)

extern struct pin _pin_b7;
#define pin_b7 (&_pin_b7)

extern struct pin _pin_b8;
#define pin_b8 (&_pin_b8)

extern struct pin _pin_b9;
#define pin_b9 (&_pin_b9)

extern struct pin _pin_b10;
#define pin_b10 (&_pin_b10)

extern struct pin _pin_b11;
#define pin_b11 (&_pin_b11)

extern struct pin _pin_b12;
#define pin_b12 (&_pin_b12)

extern struct pin _pin_b13;
#define pin_b13 (&_pin_b13)

extern struct pin _pin_b14;
#define pin_b14 (&_pin_b14)

extern struct pin _pin_b15;
#define pin_b15 (&_pin_b15)


/* GPIOC **********************************************************************/

extern struct gpio _gpio_c;
#define gpio_c (&_gpio_c)

extern struct pin _pin_c0;
#define pin_c0 (&_pin_c0)

extern struct pin _pin_c1;
#define pin_c1 (&_pin_c1)

extern struct pin _pin_c2;
#define pin_c2 (&_pin_c2)

extern struct pin _pin_c3;
#define pin_c3 (&_pin_c3)

extern struct pin _pin_c4;
#define pin_c4 (&_pin_c4)

extern struct pin _pin_c5;
#define pin_c5 (&_pin_c5)

extern struct pin _pin_c6;
#define pin_c6 (&_pin_c6)

extern struct pin _pin_c7;
#define pin_c7 (&_pin_c7)

extern struct pin _pin_c8;
#define pin_c8 (&_pin_c8)

extern struct pin _pin_c9;
#define pin_c9 (&_pin_c9)

extern struct pin _pin_c10;
#define pin_c10 (&_pin_c10)

extern struct pin _pin_c11;
#define pin_c11 (&_pin_c11)

extern struct pin _pin_c12;
#define pin_c12 (&_pin_c12)

extern struct pin _pin_c13;
#define pin_c13 (&_pin_c13)

extern struct pin _pin_c14;
#define pin_c14 (&_pin_c14)

extern struct pin _pin_c15;
#define pin_c15 (&_pin_c15)


/* GPIOD **********************************************************************/

extern struct gpio _gpio_d;
#define gpio_d (&_gpio_d)

extern struct pin _pin_d0;
#define pin_d0 (&_pin_d0)

extern struct pin _pin_d1;
#define pin_d1 (&_pin_d1)

extern struct pin _pin_d2;
#define pin_d2 (&_pin_d2)

extern struct pin _pin_d3;
#define pin_d3 (&_pin_d3)

extern struct pin _pin_d4;
#define pin_d4 (&_pin_d4)

extern struct pin _pin_d5;
#define pin_d5 (&_pin_d5)

extern struct pin _pin_d6;
#define pin_d6 (&_pin_d6)

extern struct pin _pin_d7;
#define pin_d7 (&_pin_d7)

extern struct pin _pin_d8;
#define pin_d8 (&_pin_d8)

extern struct pin _pin_d9;
#define pin_d9 (&_pin_d9)

extern struct pin _pin_d10;
#define pin_d10 (&_pin_d10)

extern struct pin _pin_d11;
#define pin_d11 (&_pin_d11)

extern struct pin _pin_d12;
#define pin_d12 (&_pin_d12)

extern struct pin _pin_d13;
#define pin_d13 (&_pin_d13)

extern struct pin _pin_d14;
#define pin_d14 (&_pin_d14)

extern struct pin _pin_d15;
#define pin_d15 (&_pin_d15)


/* GPIOE **********************************************************************/

extern struct gpio _gpio_e;
#define gpio_e (&_gpio_e)

extern struct pin _pin_e0;
#define pin_e0 (&_pin_e0)

extern struct pin _pin_e1;
#define pin_e1 (&_pin_e1)

extern struct pin _pin_e2;
#define pin_e2 (&_pin_e2)

extern struct pin _pin_e3;
#define pin_e3 (&_pin_e3)

extern struct pin _pin_e4;
#define pin_e4 (&_pin_e4)

extern struct pin _pin_e5;
#define pin_e5 (&_pin_e5)

extern struct pin _pin_e6;
#define pin_e6 (&_pin_e6)

extern struct pin _pin_e7;
#define pin_e7 (&_pin_e7)

extern struct pin _pin_e8;
#define pin_e8 (&_pin_e8)

extern struct pin _pin_e9;
#define pin_e9 (&_pin_e9)

extern struct pin _pin_e10;
#define pin_e10 (&_pin_e10)

extern struct pin _pin_e11;
#define pin_e11 (&_pin_e11)

extern struct pin _pin_e12;
#define pin_e12 (&_pin_e12)

extern struct pin _pin_e13;
#define pin_e13 (&_pin_e13)

extern struct pin _pin_e14;
#define pin_e14 (&_pin_e14)

extern struct pin _pin_e15;
#define pin_e15 (&_pin_e15)


/* GPIOF **********************************************************************/

extern struct gpio _gpio_f;
#define gpio_f (&_gpio_f)

extern struct pin _pin_f0;
#define pin_f0 (&_pin_f0)

extern struct pin _pin_f1;
#define pin_f1 (&_pin_f1)

extern struct pin _pin_f2;
#define pin_f2 (&_pin_f2)

extern struct pin _pin_f3;
#define pin_f3 (&_pin_f3)

extern struct pin _pin_f4;
#define pin_f4 (&_pin_f4)

extern struct pin _pin_f5;
#define pin_f5 (&_pin_f5)

extern struct pin _pin_f6;
#define pin_f6 (&_pin_f6)

extern struct pin _pin_f7;
#define pin_f7 (&_pin_f7)

extern struct pin _pin_f8;
#define pin_f8 (&_pin_f8)

extern struct pin _pin_f9;
#define pin_f9 (&_pin_f9)

extern struct pin _pin_f10;
#define pin_f10 (&_pin_f10)

extern struct pin _pin_f11;
#define pin_f11 (&_pin_f11)

extern struct pin _pin_f12;
#define pin_f12 (&_pin_f12)

extern struct pin _pin_f13;
#define pin_f13 (&_pin_f13)

extern struct pin _pin_f14;
#define pin_f14 (&_pin_f14)

extern struct pin _pin_f15;
#define pin_f15 (&_pin_f15)


/* GPIOG **********************************************************************/

extern struct gpio _gpio_g;
#define gpio_g (&_gpio_g)

extern struct pin _pin_g0;
#define pin_g0 (&_pin_g0)

extern struct pin _pin_g1;
#define pin_g1 (&_pin_g1)

extern struct pin _pin_g2;
#define pin_g2 (&_pin_g2)

extern struct pin _pin_g3;
#define pin_g3 (&_pin_g3)

extern struct pin _pin_g4;
#define pin_g4 (&_pin_g4)

extern struct pin _pin_g5;
#define pin_g5 (&_pin_g5)

extern struct pin _pin_g6;
#define pin_g6 (&_pin_g6)

extern struct pin _pin_g7;
#define pin_g7 (&_pin_g7)

extern struct pin _pin_g8;
#define pin_g8 (&_pin_g8)

extern struct pin _pin_g9;
#define pin_g9 (&_pin_g9)

extern struct pin _pin_g10;
#define pin_g10 (&_pin_g10)

extern struct pin _pin_g11;
#define pin_g11 (&_pin_g11)

extern struct pin _pin_g12;
#define pin_g12 (&_pin_g12)

extern struct pin _pin_g13;
#define pin_g13 (&_pin_g13)

extern struct pin _pin_g14;
#define pin_g14 (&_pin_g14)

extern struct pin _pin_g15;
#define pin_g15 (&_pin_g15)


/* GPIOH **********************************************************************/

extern struct gpio _gpio_h;
#define gpio_h (&_gpio_h)

extern struct pin _pin_h0;
#define pin_h0 (&_pin_h0)

extern struct pin _pin_h1;
#define pin_h1 (&_pin_h1)

extern struct pin _pin_h2;
#define pin_h2 (&_pin_h2)

extern struct pin _pin_h3;
#define pin_h3 (&_pin_h3)

extern struct pin _pin_h4;
#define pin_h4 (&_pin_h4)

extern struct pin _pin_h5;
#define pin_h5 (&_pin_h5)

extern struct pin _pin_h6;
#define pin_h6 (&_pin_h6)

extern struct pin _pin_h7;
#define pin_h7 (&_pin_h7)

extern struct pin _pin_h8;
#define pin_h8 (&_pin_h8)

extern struct pin _pin_h9;
#define pin_h9 (&_pin_h9)

extern struct pin _pin_h10;
#define pin_h10 (&_pin_h10)

extern struct pin _pin_h11;
#define pin_h11 (&_pin_h11)

extern struct pin _pin_h12;
#define pin_h12 (&_pin_h12)

extern struct pin _pin_h13;
#define pin_h13 (&_pin_h13)

extern struct pin _pin_h14;
#define pin_h14 (&_pin_h14)

extern struct pin _pin_h15;
#define pin_h15 (&_pin_h15)


/* GPIOI **********************************************************************/

extern struct gpio _gpio_i;
#define gpio_i (&_gpio_i)

extern struct pin _pin_i0;
#define pin_i0 (&_pin_i0)

extern struct pin _pin_i1;
#define pin_i1 (&_pin_i1)

extern struct pin _pin_i2;
#define pin_i2 (&_pin_i2)

extern struct pin _pin_i3;
#define pin_i3 (&_pin_i3)

extern struct pin _pin_i4;
#define pin_i4 (&_pin_i4)

extern struct pin _pin_i5;
#define pin_i5 (&_pin_i5)

extern struct pin _pin_i6;
#define pin_i6 (&_pin_i6)

extern struct pin _pin_i7;
#define pin_i7 (&_pin_i7)

extern struct pin _pin_i8;
#define pin_i8 (&_pin_i8)

extern struct pin _pin_i9;
#define pin_i9 (&_pin_i9)

extern struct pin _pin_i10;
#define pin_i10 (&_pin_i10)

extern struct pin _pin_i11;
#define pin_i11 (&_pin_i11)

extern struct pin _pin_i12;
#define pin_i12 (&_pin_i12)

extern struct pin _pin_i13;
#define pin_i13 (&_pin_i13)

extern struct pin _pin_i14;
#define pin_i14 (&_pin_i14)

extern struct pin _pin_i15;
#define pin_i15 (&_pin_i15)

#ifdef __cplusplus
}
#endif

#endif /* __gpio_h */
