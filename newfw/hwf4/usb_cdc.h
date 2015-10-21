/* -*- Mode: C; indent-tabs-mode: nil; c-basic-offset: 2 -*- */
/*
 * usb_cdc.h --- USB serial driver.
 *
 * Copyright (C) 2012, Galois, Inc.
 * All Rights Reserved.
 *
 * This software is released under the "BSD3" license.  Read the file
 * "LICENSE" for more information.
 *
 * Written by James Bielman <jamesjb@galois.com>, 13 December 2012
 */

#ifndef __hwf4_usb_cdc_h

#ifdef __cplusplus
extern "C" {
#endif

#include <FreeRTOS.h>

#include <stdbool.h>
#include <stdint.h>
#include <sys/types.h>

/**
 * Initialize the USB stack and CDC driver.
 */
void usb_cdc_init(void);

/**
 * Write to the CDC port.  This function will never block, so there is
 * no version that accepts a timeout.
 *
 * @returns the number of bytes written, or -1 on failure
 */
ssize_t usb_cdc_write(const void *buf, size_t len);

/**
 * Read from the CDC port, delaying for at most timeout*len ms.
 *
 * @returns the number of bytes read into "buf", or -1 on failure
 */
ssize_t usb_cdc_read_timeout(void *buf, size_t len, uint32_t timeout);

/**
 * Read from the CDC port, blocking until the entire buffer is read.
 *
 * @returns the number of bytes read, or -1 on failure
 */
static inline ssize_t usb_cdc_read(void *buf, size_t len)
{
  return usb_cdc_read_timeout(buf, len, portMAX_DELAY);
}

#ifdef __cplusplus
}
#endif

#endif   /* !defined __hwf4_usb_cdc_h */
