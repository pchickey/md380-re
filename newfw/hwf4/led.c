/* -*- Mode: C; indent-tabs-mode: nil; c-basic-offset: 2 -*- */
/*
 * led.c --- Debug LED driver.
 *
 * Copyright (C) 2012, Galois, Inc.
 * All Rights Reserved.
 *
 * This software is released under the "BSD3" license.  Read the file
 * "LICENSE" for more information.
 *
 * Written by James Bielman <jamesjb@galois.com>, 13 December 2012
 */

#include "gpio.h"
#include "led.h"

/* We make an assumption here that all the LEDs on the board are
 * driven the same way.  This is true for our current boards, but will
 * need to be redesigned if that changes.  We might need to support
 * LEDs that need internal pullups/pulldowns someday as well. */
#if defined(CONFIG_BOARD_STM32F4_DISCOVERY)
# define LED_COUNT 4
# define LED_PINS  { pin_d12, pin_d13, pin_d14, pin_d15 }
# define LED_ON_HIGH            /* drive high to turn on */
# define LED_OFF_LOW            /* drive log to turn off */
#elif defined(CONFIG_BOARD_PX4)
# define LED_COUNT 1
# define LED_PINS  { pin_b15 }
# define LED_ON_LOW             /* drive low to turn on */
# define LED_OFF_FLOAT          /* float to turn off */
#else
# define LED_COUNT 0
# define LED_PINS  {}
# define LED_ON_HIGH
# define LED_OFF_LOW
#endif

/** Array of GPIO pins by LED number. */
struct pin *g_leds[LED_COUNT] = LED_PINS; /* XXX should be const */

/** Configure the GPIO pin for an LED. */
static void led_setup_pin(struct pin *p)
{
  pin_enable(p);
  pin_set_otype(p, PIN_TYPE_PUSHPULL);
  pin_set_ospeed(p, PIN_SPEED_2MHZ);
  pin_set_pupd(p, PIN_PUPD_NONE);
}

/*
 * {Public Interface}
 */

void led_init(void)
{
  for (int i = 0; i < LED_COUNT; ++i)
    led_setup_pin(g_leds[i]);
}

int led_count(void)
{
  return LED_COUNT;
}

bool led_set(int led, bool state)
{
  if (led < 0 || led >= LED_COUNT)
    return false;

  struct pin *p;
  p = g_leds[led];

  if (state) {
#if defined(LED_ON_HIGH)
    pin_set(p);
    pin_set_mode(p, PIN_MODE_OUTPUT);
#elif defined(LED_ON_LOW)
    pin_reset(p);
    pin_set_mode(p, PIN_MODE_OUTPUT);
#endif
  } else {
#if defined(LED_OFF_LOW)
    pin_reset(p);
    pin_set_mode(p, PIN_MODE_OUTPUT);
#elif defined(LED_OFF_FLOAT)
    pin_set_mode(p, PIN_MODE_ANALOG);
#endif
  }

  return true;
}
