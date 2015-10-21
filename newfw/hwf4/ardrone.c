/* -*- Mode: C; indent-tabs-mode: nil; c-basic-offset: 2 -*- */
/*
 * ardrone.c --- AR.Drone motor control protocol.
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

#include <stdbool.h>
#include <stdint.h>

#include <FreeRTOS.h>
#include <task.h>

#include "gpio.h"
#include "timer.h"
#include "usart.h"
#include "ardrone.h"

/** The number of motors on the AR.Drone. */
#define NUM_MOTORS 4

/** 9usec per byte at 115200bps plus overhead. */
#define UART_BYTE_US (9 + 50)

/**
 * Motor select GPIO pins, output active low.
 *
 * XXX should be const
 */
static struct pin *g_motor_select_pin[NUM_MOTORS] = {
  pin_c4,                       /* GPIO_EXT1 */
  pin_c5,                       /* GPIO_EXT2 */
  pin_a0,                       /* USART2_CTS */
  pin_a1,                       /* USART2_RTS */
};

/** Shared IRQ pin for all 4 motors. */
#define IRQ_PIN pin_b12         /* CAN2_RX */

/** Direction pin for the level shifter on GPIO_EXT1 and GPIO_EXT2. */
#define GPIO_DIR_PIN pin_c13

/**
 * Select a motor (1-4).  Selecting 0 selects all motors.
 */
static void ardrone_motor_select(uint8_t motor)
{
  int i;

  if (motor == 0) {
    for (i = 0; i < NUM_MOTORS; ++i)
      pin_reset(g_motor_select_pin[i]);
  } else if (motor <= NUM_MOTORS) {
    pin_reset(g_motor_select_pin[motor - 1]);
  }
}

/**
 * Deselect a motor.  Selecting 0 deselects all motors.
 */
static void ardrone_motor_deselect(uint8_t motor)
{
  int i;

  if (motor == 0) {
    for (i = 0; i < NUM_MOTORS; ++i)
      pin_set(g_motor_select_pin[i]);
  } else if (motor <= NUM_MOTORS) {
    pin_set(g_motor_select_pin[motor - 1]);
  }
}

/**
 * Generate the 5-byte motor set packet.
 */
static void ardrone_get_motor_packet(uint8_t *buf, uint16_t speed[4])
{
  uint16_t temp;

  buf[0] = 0x20;
  buf[1] = 0x00;
  buf[2] = 0x00;
  buf[3] = 0x00;
  buf[4] = 0x00;

  temp = (speed[0] & 0x1FF) << 4;
  buf[0] |= (temp >> 8) & 0xFF;
  buf[1] |= (temp >> 0) & 0xFF;

  temp = (speed[1] & 0x1FF) << 3;
  buf[1] |= (temp >> 8) & 0xFF;
  buf[2] |= (temp >> 0) & 0xFF;

  temp = (speed[2] & 0x1FF) << 2;
  buf[2] |= (temp >> 8) & 0xFF;
  buf[3] |= (temp >> 0) & 0xFF;

  temp = (speed[3] & 0x1FF) << 1;
  buf[3] |= (temp >> 8) & 0xFF;
  buf[4] |= (temp >> 0) & 0xFF;
}

/*
 * {Public Interface}
 */

void ardrone_init(void)
{
  int i;

  /* Configure motor selects as outputs and set high. */
  for (i = 0; i < NUM_MOTORS; ++i) {
    struct pin *p = g_motor_select_pin[i];

    pin_enable(p);
    pin_set(p);
    pin_set_otype(p, PIN_TYPE_PUSHPULL);
    pin_set_ospeed(p, PIN_SPEED_2MHZ);
    pin_set_mode(p, PIN_MODE_OUTPUT);
  }

  pin_enable(IRQ_PIN);
  pin_set_pupd(IRQ_PIN, PIN_PUPD_NONE);
  pin_set_mode(IRQ_PIN, PIN_MODE_INPUT);

  /* Set GPIO_DIR high to configure level shifter for output. */
  pin_enable(GPIO_DIR_PIN);
  pin_set(GPIO_DIR_PIN);
  pin_set_otype(GPIO_DIR_PIN, PIN_TYPE_PUSHPULL);
  pin_set_ospeed(GPIO_DIR_PIN, PIN_SPEED_2MHZ);
  pin_set_mode(GPIO_DIR_PIN, PIN_MODE_OUTPUT);

  usart_init(usart2, 115200);
  usart_enable(usart2);
}

bool ardrone_read_irq(void)
{
  return pin_read(IRQ_PIN);
}

void ardrone_motor_init(void)
{
  uint8_t init[] = { 0xE0, 0x91, 0xA1, 0x00, 0x40 };
  uint8_t multi[] = { 0xA0, 0xA0, 0xA0, 0xA0, 0xA0, 0xA0 };
  int i;

  ardrone_motor_deselect(0);

  for (i = 1; i <= NUM_MOTORS; ++i) {
    ardrone_motor_select(i);
    timer_usleep(200);

    usart_write_blocking(usart2, &init[0], 1);
    timer_usleep(UART_BYTE_US * 1);

    usart_write_blocking(usart2, &init[1], 1);
    timer_usleep(UART_BYTE_US * 120);

    usart_write_blocking(usart2, &init[2], 1);
    timer_usleep(UART_BYTE_US * 1);

    init[3] = (uint8_t)i;
    usart_write_blocking(usart2, &init[3], 1);

    usart_write_blocking(usart2, &init[4], 1);
    timer_usleep(UART_BYTE_US * 11);

    ardrone_motor_deselect(i);
    vTaskDelay(200);
  }

  ardrone_motor_select(0);
  timer_usleep(200);

  usart_write_blocking(usart2, multi, sizeof(multi));
  timer_usleep(UART_BYTE_US * sizeof(multi));

  usart_write_blocking(usart2, multi, sizeof(multi));
  timer_usleep(UART_BYTE_US * sizeof(multi));

  ardrone_motor_set(0, 0, 0, 0);
}

void ardrone_motor_set(uint16_t m1, uint16_t m2, uint16_t m3, uint16_t m4)
{
  uint16_t speeds[4] = { m1, m2, m3, m4 };
  uint8_t buf[5];

  ardrone_get_motor_packet(buf, speeds);
  usart_write_blocking(usart2, buf, sizeof(buf));
}

void ardrone_led_set(const struct ardrone_led *leds)
{
  /*
   * 2 bytes are sent. The first 3 bits describe the command: 011
   * means led control the following 4 bits are the red leds for motor
   * 4, 3, 2, 1 then 4 bits with unknown function, then 4 bits for
   * green leds for motor 4, 3, 2, 1 the last bit is unknown.
   *
   * The packet is therefore:
   *
   *   011 rrrr 0000 gggg 0
   */
  uint8_t buf[2];
  buf[0] = 0x60 | ((leds->red4 & 0x01) << 4) | ((leds->red3 & 0x01) << 3) |
           ((leds->red2 & 0x01) << 2) | ((leds->red1 & 0x01) << 1);
  buf[1] = ((leds->green4 & 0x01) << 4) | ((leds->green3 & 0x01) << 3) |
           ((leds->green2 & 0x01) << 2) | ((leds->green1 & 0x01) << 1);

  usart_write_blocking(usart2, buf, 2);
}
