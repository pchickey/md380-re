/* -*- Mode: C; indent-tabs-mode: nil; c-basic-offset: 2 -*- */
/*
 * ardrone.h --- AR.Drone motor control protocol.
 *
 * Copyright (C) 2012, PX4 Development Team.
 * Copyright (C) 2012, Galois, Inc.
 * All Rights Reserved.
 *
 * This software is released under the "BSD3" license.  Read the file
 * "LICENSE" for more information.
 *
 * Written by Lorenz Meier <lm@inf.ethz.ch>
 * Written by James Bielman <jamesjb@galois.com>, 07 December 2012
 */

#ifndef __ardrone_h
#define __ardrone_h

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Initialize the AR.Drone driver.
 *
 * For now, this just initializes the GPIOs and UART.
 *
 * Requires the HWF4 timer and USART2 to be initialized.
 */
void ardrone_init(void);

/**
 * Send the init sequence to all motors.
 */
void ardrone_motor_init(void);

/**
 * Set the speed of all four motors.
 */
void ardrone_motor_set(uint16_t m1, uint16_t m2, uint16_t m3, uint16_t m4);

/**
 * Read the motor controller IRQ pin.
 *
 * TODO: Should this really be an interrupt instead?
 */
bool ardrone_read_irq(void);

struct ardrone_led {
  uint8_t red1, green1;
  uint8_t red2, green2;
  uint8_t red3, green3;
  uint8_t red4, green4;
};

void ardrone_led_set(const struct ardrone_led *leds);

#ifdef __cplusplus
}
#endif

#endif   /* !defined __ardrone_h */
