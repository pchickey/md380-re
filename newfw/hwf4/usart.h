/*
 * usart.h --- STM32F4 USART driver.
 *
 * Copyright (C) 2012, Galois, Inc.
 * All Rights Reserved.
 *
 * This software is released under the "BSD3" license.  Read the file
 * "LICENSE" for more information.
 */

#ifndef __hwf4_usart_h
#define __hwf4_usart_h

#ifdef __cplusplus
extern "C" {
#endif

#include <FreeRTOS.h>

#include <stdbool.h>
#include <stdint.h>
#include <sys/types.h>

struct usart;

/**
 * Initialize a usart device.
 *
 * @param usart One of usart[1,2,3,6] or uart[4,5]
 *
 * @return true on success
 */
bool usart_init(struct usart *usart, uint32_t baud);

/**
 * Enable a usart device.
 */
void usart_enable(struct usart *usart);

/**
 * Write to the usart, delaying for at most timeout*len ms.
 */
int32_t usart_write_timeout(struct usart *usart, uint32_t timeout,
                            const uint8_t *buf, uint32_t len);

/**
 * Read from the usart, delaying for at most timeout*len ms.
 */
int32_t usart_read_timeout(struct usart *usart, uint32_t timeout, uint8_t *buf,
                           uint32_t len);

/**
 * Write to the usart device, blocking until the entire buffer has been written.
 */
static inline int32_t usart_write(struct usart *usart, const uint8_t *buf,
                                  uint32_t len) {
    return usart_write_timeout(usart, portMAX_DELAY, buf, len);
}

/**
 * Write to a USART from an ISR, dropping bytes if there is no room in
 * the internal buffer to hold the data.  This is useful for
 * debugging, but should not be relied upon in normal operation.
 */
void usart_write_from_isr(struct usart *usart, const uint8_t *buf, uint32_t len);

/**
 * Perform a blocking write directly to the USART without interrupts.
 * Do not combine calls to this with normal writes.
 */
int32_t usart_write_blocking(struct usart *usart, const uint8_t *buf,
                             uint32_t len);

/**
 * Read from the usart device, blocking until the entire buffer has been
 * written.
 */
static inline int32_t usart_read(struct usart *usart, uint8_t *buf,
                                 uint32_t len) {
    return usart_read_timeout(usart, portMAX_DELAY, buf, len);
}

/** Return true if there is pending output in the TX buffer. */
bool usart_is_tx_pending(struct usart *usart);

/**
 * Return the number of bytes available for reading.
 *
 * Note that this number should not be relied upon exactly, as data
 * becomes available asynchronously via the RX interrupt.  It can be
 * used to poll for input.
 */
uint32_t usart_available(struct usart *usart);

/**
 * Read the first byte of input without removing it.
 *
 * Returns true if an input byte was available and placed in "buf", or
 * false if no input was available.
 */
bool usart_peek(struct usart *usart, uint8_t *buf);

/**
 * Return the number of bytes that can be written without blocking.
 *
 * This number is useful as a lower bound on the value---it will
 * decrease asynchronously via the TX interrupt.
 */
uint32_t usart_txspace(struct usart *usart);

/* USART Handles **************************************************************/

extern struct usart _usart1;
#define usart1 (&_usart1)

extern struct usart _usart2;
#define usart2 (&_usart2)

extern struct usart _usart3;
#define usart3 (&_usart3)

extern struct usart _uart4;
#define uart4 (&_uart4)

extern struct usart _uart5;
#define uart5 (&_uart5)

extern struct usart _usart6;
#define usart6 (&_usart6)

#ifdef __cplusplus
}
#endif

#endif /* __hwf4_usart_h */
