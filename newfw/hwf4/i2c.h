/*
 * i2c.h --- STM32 I2C driver.
 *
 * Copyright (C) 2012, Galois, Inc.
 * All Rights Reserved.
 *
 * This software is released under the "BSD3" license.  Read the file
 * "LICENSE" for more information.
 */

#ifndef __i2c_h
#define __i2c_h

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>

#include "gpio.h"
#include "rcc.h"

struct i2cdrv_t;

void i2c_init(struct i2cdrv_t *drv, struct pin* sda, struct pin* scl);

/**
 * Perform a generic I2C write/read transfer.
 *
 * First, "tx_len" bytes are written from "tx_buf".  Then "rx_len"
 * bytes are read into "rx_buf".
 *
 * If this interface turns out to not be general enough, we should
 * refactor the interface to accept an array of "iovec"-like
 * structures that specify a buffer/length/direction.
 */
bool i2c_transfer(struct i2cdrv_t *drv, uint8_t addr,
                  const uint8_t *tx_buf, size_t tx_len,
                  uint8_t *rx_buf, size_t rx_len);

bool i2c_read_reg(struct i2cdrv_t *drv, uint8_t addr, uint8_t reg,
        uint8_t len, uint8_t *buf);

bool i2c_write(struct i2cdrv_t *drv, uint8_t addr, uint8_t *buf, size_t len);

bool i2c_write_reg(struct i2cdrv_t *drv, uint8_t addr,
        uint8_t reg, uint8_t data);

/** Write multiple registers, assuming auto-incrementing. */
bool i2c_write_regs(struct i2cdrv_t *drv, uint8_t addr, uint8_t reg,
                    uint8_t *data, uint8_t len);

extern struct i2cdrv_t _i2c1_drv;
#define i2c1 (&_i2c1_drv)

extern struct i2cdrv_t _i2c2_drv;
#define i2c2 (&_i2c2_drv)

extern struct i2cdrv_t _i2c3_drv;
#define i2c3 (&_i2c3_drv)

#ifdef __cplusplus
}
#endif

#endif /* __i2c.h */
