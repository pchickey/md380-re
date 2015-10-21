/* -*- Mode: C; indent-tabs-mode: nil; c-basic-offset: 2 -*- */
/*
 * eeprom.h --- I2C EEPROM driver.
 *
 * Copyright (C) 2012, Galois, Inc.
 * All Rights Reserved.
 *
 * This software is released under the "BSD3" license.  Read the file
 * "LICENSE" for more information.
 *
 * Written by James Bielman <jamesjb@galois.com>, 13 December 2012
 */

#ifndef __hwf4_eeprom_h
#define __hwf4_eeprom_h

#include <stdbool.h>
#include <stdint.h>
#include <sys/types.h>

#include "i2c.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Initialize the EEPROM driver on an I2C bus.
 *
 * @param bus  I2C bus device, must be initialized already
 * @param addr I2C address of EEPROM device
 */
void eeprom_init(struct i2cdrv_t *bus, uint8_t addr);

/**
 * Read a single byte from the EEPROM.
 *
 * @param addr byte address to read from
 * @param buf  buffer to write output byte to
 * @returns true if the read was successful, false on error
 */
bool eeprom_read_byte(uint16_t addr, uint8_t *buf);

/**
 * Write a single byte to the EEPROM.
 *
 * @param addr byte address to write to
 * @param byte data byte to write
 * @returns true if the write was successful, false on error
 */
bool eeprom_write_byte(uint16_t addr, uint8_t byte);

/**
 * Read multiple bytes from the EEPROM.
 *
 * @param addr start byte address to begin reading
 * @param buf  buffer to place data read into
 * @param len  number of bytes to read
 * @returns the number of bytes read, or -1 on error
 */
ssize_t eeprom_read(uint16_t addr, uint8_t *buf, size_t len);

/**
 * Write multiple bytes to the EEPROM.
 *
 * This breaks the transfer up into multiple page writes as
 * needed, so the caller need not worry about it.
 *
 * @param addr start byte address for writing
 * @param buf  buffer to write data from
 * @param len  number of bytes to write
 * @returns the number of bytes written, or -1 on error
 */
ssize_t eeprom_write(uint16_t addr, const uint8_t *buf, size_t len);

#ifdef __cplusplus
}
#endif

#endif   /* !defined __hwf4_eeprom_h */
