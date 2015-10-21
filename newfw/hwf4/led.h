/* -*- Mode: C; indent-tabs-mode: nil; c-basic-offset: 2 -*- */
/*
 * led.h --- Debug LED driver.
 *
 * Copyright (C) 2012, Galois, Inc.
 * All Rights Reserved.
 *
 * This software is released under the "BSD3" license.  Read the file
 * "LICENSE" for more information.
 *
 * Written by James Bielman <jamesjb@galois.com>, 13 December 2012
 */

#ifndef __hwf4_led_h
#define __hwf4_led_h

#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Initialize the LED driver.
 *
 * Configures all LED GPIOs and sets them all off.
 */
void led_init(void);

/** Return the number of LEDs present on this board. */
int led_count(void);

/**
 * Set the state of an LED by number.
 *
 * If the LED does not exist on the board, this does nothing.
 *
 * @returns true if the LED exists and was set
 */
bool led_set(int led, bool state);

#ifdef __cplusplus
}
#endif

#endif   /* !defined __hwf4_led_h */
