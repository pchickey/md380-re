/*
 * gpio.c --- STM32F4 GPIO driver.
 *
 * Copyright (C) 2012, Galois, Inc.
 * All Rights Reserved.
 *
 * This software is released under the "BSD3" license.  Read the file
 * "LICENSE" for more information.
 */

#include <stm32f4xx.h>

#include "rcc.h"
#include "gpio.h"


/* External GPIO Interface ****************************************************/

struct gpio {
    GPIO_TypeDef *dev;
    enum RCCDevice rcc_dev;
};


void gpio_enable(struct gpio *gpio) {
    rcc_enable(gpio->rcc_dev);
}

void gpio_reset_moder(struct gpio *gpio, uint32_t val) {
    gpio->dev->MODER &= ~val;
}

void gpio_update_moder(struct gpio *gpio, uint32_t val) {
    gpio->dev->MODER |= val;
}

void gpio_reset_otyper(struct gpio *gpio, uint32_t val) {
    gpio->dev->OTYPER &= ~val;
}

void gpio_update_otyper(struct gpio *gpio, uint32_t val) {
    gpio->dev->OTYPER |= val;
}

void gpio_reset_ospeedr(struct gpio *gpio, uint32_t val) {
    gpio->dev->OSPEEDR &= ~val;
}

void gpio_update_ospeedr(struct gpio *gpio, uint32_t val) {
    gpio->dev->OSPEEDR |= val;
}

void gpio_reset_pupdr(struct gpio *gpio, uint32_t val) {
    gpio->dev->PUPDR &= ~val;
}

void gpio_update_pupdr(struct gpio *gpio, uint32_t val) {
    gpio->dev->PUPDR |= val;
}

uint32_t gpio_get_idr(struct gpio *gpio) {
    return gpio->dev->IDR;
}

void gpio_modify_odr(struct gpio *gpio, uint32_t val) {
    gpio->dev->ODR |= val;
}

void gpio_reset_afr(struct gpio *gpio, int lh, uint32_t val) {
    gpio->dev->AFR[lh] &= ~val;
}

void gpio_modify_afr(struct gpio *gpio, int lh, uint32_t val) {
    gpio->dev->AFR[lh] |= val;
}

void gpio_set_bsrrl(struct gpio *gpio, uint32_t val) {
    gpio->dev->BSRRL = val;
}

void gpio_set_bsrrh(struct gpio *gpio, uint32_t val) {
    gpio->dev->BSRRH = val;
}



/* External GPIO Pin Interface ************************************************/

struct pin {
    struct gpio *bank;
    uint8_t pin_num;
};

void pin_enable(struct pin *pin) {
    gpio_enable(pin->bank);
}

void pin_set_mode(struct pin *pin, enum pin_mode mode) {
    gpio_reset_moder(pin->bank, GPIO_MODER_MODER0 << (pin->pin_num * 2));

    uint32_t mask = (uint32_t)mode << (pin->pin_num * 2);
    gpio_update_moder(pin->bank, mask);
}

void pin_set_otype(struct pin *pin, enum pin_type type) {
    gpio_reset_otyper(pin->bank, GPIO_OTYPER_OT_0 << pin->pin_num);

    uint32_t mask = (uint32_t)type << pin->pin_num;
    gpio_update_otyper(pin->bank, mask);
}

void pin_set_ospeed(struct pin *pin, enum pin_speed speed) {
    gpio_reset_ospeedr(pin->bank, GPIO_OSPEEDER_OSPEEDR0 << (pin->pin_num * 2));

    uint32_t mask = (uint32_t)speed << (pin->pin_num * 2);
    gpio_update_ospeedr(pin->bank, mask);
}

void pin_set_pupd(struct pin *pin, enum pin_pupd pupd) {
    gpio_reset_pupdr(pin->bank, GPIO_PUPDR_PUPDR0 << pin->pin_num);

    uint32_t mask = (uint32_t)pupd << pin->pin_num;
    gpio_update_pupdr(pin->bank, mask);
}

void pin_set_af(struct pin *pin, enum pin_af af) {
    int reg = (pin->pin_num & 0x7) * 4;    // [0,7]
    int lh  = (pin->pin_num >> 0x3) & 0x1; // [0,1]

    gpio_reset_afr(pin->bank, lh, 0xf << reg);

    uint32_t mask = ((uint32_t)af & 0xf) << reg;
    gpio_modify_afr(pin->bank, lh, mask);
}


void pin_reset(struct pin *pin) {
    gpio_set_bsrrh(pin->bank, 0x1 << pin->pin_num);
}

void pin_set(struct pin *pin) {
    gpio_set_bsrrl(pin->bank, 0x1 << pin->pin_num);
}

void pin_toggle(struct pin *pin) {
    uint16_t odr = pin->bank->dev->ODR;

    if (odr & (1 << pin->pin_num))
        pin_reset(pin);
    else
        pin_set(pin);
}

bool pin_read(struct pin *pin) {
    uint16_t idr = gpio_get_idr(pin->bank);
    return (idr & (1 << pin->pin_num)) != 0;
}

/* GPIO Bank Definition *******************************************************/

/**
 * Define a pin, assuming the presence of a gpio bank.
 */
#define DEFINE_PIN(b,n)                           \
struct pin _pin_ ## b ## n = {                    \
    .bank    = & _gpio_ ## b,                     \
    .pin_num = n                                  \
};                                                \

/**
 * Define a GPIO bank
 */
#define DEFINE_BANK(d,b)                  \
struct gpio _gpio_ ## b = {               \
    .dev     = d,                         \
    .rcc_dev = RCCDEV_ ## d               \
};                                        \
DEFINE_PIN(b,0);                          \
DEFINE_PIN(b,1);                          \
DEFINE_PIN(b,2);                          \
DEFINE_PIN(b,3);                          \
DEFINE_PIN(b,4);                          \
DEFINE_PIN(b,5);                          \
DEFINE_PIN(b,6);                          \
DEFINE_PIN(b,7);                          \
DEFINE_PIN(b,8);                          \
DEFINE_PIN(b,9);                          \
DEFINE_PIN(b,10);                         \
DEFINE_PIN(b,11);                         \
DEFINE_PIN(b,12);                         \
DEFINE_PIN(b,13);                         \
DEFINE_PIN(b,14);                         \
DEFINE_PIN(b,15);

DEFINE_BANK(GPIOA,a);
DEFINE_BANK(GPIOB,b);
DEFINE_BANK(GPIOC,c);
DEFINE_BANK(GPIOD,d);
DEFINE_BANK(GPIOE,e);
DEFINE_BANK(GPIOF,f);
DEFINE_BANK(GPIOG,g);
DEFINE_BANK(GPIOH,h);
DEFINE_BANK(GPIOI,i);

#undef DEFINE_BANK
#undef DEFINE_PIN
