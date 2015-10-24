/* -*- Mode: C; indent-tabs-mode: nil; c-basic-offset: 2 -*- */
/*
 * eeprom.c --- I2C EEPROM driver.
 *
 * Copyright (C) 2012, Galois, Inc.
 * All Rights Reserved.
 *
 * This software is released under the "BSD3" license.  Read the file
 * "LICENSE" for more information.
 *
 * Written by James Bielman <jamesjb@galois.com>, 13 December 2012
 */

#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <sys/types.h>

#include "i2c.h"
#include "eeprom.h"

#include <FreeRTOS.h>
#include <task.h>

/**
 * State of the EEPROM driver.
 *
 * This contains everything needed to support multiple EEPROM devices
 * by adding a declaration to the header file and adding it as a
 * parameter to each API function, rather than declaring a single
 * global instance.
 */
struct eeprom_dev {
  struct i2cdrv_t *bus;         /* i2c bus */
  uint8_t i2c_addr;             /* i2c address */
} g_dev;

/** Maximum length of the EEPROM write cycle. */
#define EEPROM_WRITE_CYCLE_MS 5

/** Size of an EEPROM data page. */
#define EEPROM_PAGE_SIZE 64

/*
 * {Public Interface}
 */

void eeprom_init(struct i2cdrv_t *bus, uint8_t addr)
{
  g_dev.bus = bus;
  g_dev.i2c_addr = addr;
}

bool eeprom_read_byte(uint16_t addr, uint8_t *buf)
{
  uint8_t addr_bytes[2];

  addr_bytes[0] = (addr >> 8) & 0xFF;
  addr_bytes[1] = (addr >> 0) & 0xFF;

  return i2c_transfer(g_dev.bus, g_dev.i2c_addr, addr_bytes, 2, buf, 1);
}

ssize_t eeprom_read(uint16_t addr, uint8_t *buf, size_t len)
{
  uint8_t addr_bytes[2];

  addr_bytes[0] = (addr >> 8) & 0xFF;
  addr_bytes[1] = (addr >> 0) & 0xFF;

  if (!i2c_transfer(g_dev.bus, g_dev.i2c_addr, addr_bytes, 2, buf, len))
    return -1;

  return len;
}

bool eeprom_write_byte(uint16_t addr, uint8_t byte)
{
  uint8_t buf[3];
  bool result;

  buf[0] = (addr >> 8) & 0xFF;
  buf[1] = (addr >> 0) & 0xFF;
  buf[2] = byte;

  result = i2c_transfer(g_dev.bus, g_dev.i2c_addr, buf, 3, NULL, 0);
  vTaskDelay(EEPROM_WRITE_CYCLE_MS);

  return result;
}

ssize_t eeprom_write(uint16_t addr, const uint8_t *buf, size_t len)
{
  /* It would be nice to do this without needing a buffer on the stack
   * and a memcpy, but we don't have scatter/gather I/O for I2C, and
   * we have to send the address somehow. */
  uint8_t tx_buf[EEPROM_PAGE_SIZE + 2];
  ssize_t result = 0;

  while (len > 0) {
    /* Figure out how many bytes we can write using a page write
     * starting at "addr". */
    uint16_t page_start = addr & ~(EEPROM_PAGE_SIZE - 1);
    uint16_t page_end   = page_start + EEPROM_PAGE_SIZE;
    uint16_t write_len  = page_end - addr;

    if (write_len > len)        /* handle final partial write */
      write_len = len;

    tx_buf[0] = (addr >> 8) & 0xFF;
    tx_buf[1] = (addr >> 0) & 0xFF;
    memcpy(&tx_buf[2], buf, write_len);

    if (!i2c_transfer(g_dev.bus, g_dev.i2c_addr, tx_buf, write_len + 2, NULL, 0))
      break;

    vTaskDelay(EEPROM_WRITE_CYCLE_MS);

    addr   += write_len;
    buf    += write_len;
    result += write_len;
    len    -= write_len;
  }

  return result;
}
