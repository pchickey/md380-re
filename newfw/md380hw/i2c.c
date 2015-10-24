/* -*- Mode: C; indent-tabs-mode: nil; c-basic-offset: 4 -*- */
/*
 * i2c.c --- STM32F4 I2C driver.
 *
 * Copyright (C) 2012, Galois, Inc.
 * All Rights Reserved.
 *
 * This software is released under the "BSD3" license.  Read the file
 * "LICENSE" for more information.
 */

#include <stddef.h>
#include <string.h>

#include <stm32f4xx.h>

#include <FreeRTOS.h>
#include <queue.h>
#include <semphr.h>
#include <task.h>

#include "i2c.h"
#include "gpio.h"
#include "rcc.h"
#include "interrupt.h"

struct i2chw_t {
    I2C_TypeDef* reg;
    enum RCCDevice dev;
    enum IRQn irq_ev;
    enum IRQn irq_er;
};

const struct i2chw_t i2c1_hw =
    { .reg = I2C1,
      .dev = RCCDEV_I2C1,
      .irq_ev = I2C1_EV_IRQn,
      .irq_er = I2C1_ER_IRQn
    };

const struct i2chw_t i2c2_hw =
    { .reg = I2C2,
      .dev = RCCDEV_I2C2,
      .irq_ev = I2C2_EV_IRQn,
      .irq_er = I2C2_ER_IRQn
    };

const struct i2chw_t i2c3_hw =
    { .reg = I2C3,
      .dev = RCCDEV_I2C3,
      .irq_ev = I2C3_EV_IRQn,
      .irq_er = I2C3_ER_IRQn
    };

struct i2cdrv_t {
    struct i2chw_t const * hw;
    volatile uint8_t addr;
    volatile size_t  writing;
    volatile size_t  reading;
    volatile uint8_t* write_p;
    volatile uint8_t* read_p;

    volatile bool error;

    xSemaphoreHandle lock;      /* bus mutex */
    xSemaphoreHandle complete;  /* completion from ISR */
};

struct i2cdrv_t _i2c1_drv = { &i2c1_hw, 0, 0, 0, NULL, NULL, false, NULL, NULL };
struct i2cdrv_t _i2c2_drv = { &i2c2_hw, 0, 0, 0, NULL, NULL, false, NULL, NULL };
struct i2cdrv_t _i2c3_drv = { &i2c3_hw, 0, 0, 0, NULL, NULL, false, NULL, NULL };


static inline void __i2c_reset_cr2(struct i2cdrv_t *i2c, uint32_t val) {
    i2c->hw->reg->CR2 &= ~val;
}

static inline void __i2c_update_cr2(struct i2cdrv_t *i2c, uint32_t val) {
    i2c->hw->reg->CR2 |= val;
}

/**
 * Generate an I2C START condition.  Per the reference manual, we wait
 * for the hardware to clear the start bit after setting it before
 * allowing any further writes to CR1.  This prevents random lockups.
 */
static inline void __i2c_set_start(struct i2cdrv_t *i2c) {
    i2c->hw->reg->CR1 |= I2C_CR1_START;
    while (i2c->hw->reg->CR1 & I2C_CR1_START);
}

/**
 * Generate an I2C STOP condition.  Per the reference manual, we wait
 * for the hardware to clear the stop bit after setting it before
 * allowing any further writes to CR1.  This prevents random lockups.
 */
static inline void __i2c_set_stop(struct i2cdrv_t *i2c) {
    i2c->hw->reg->CR1 |= I2C_CR1_STOP;
    while (i2c->hw->reg->CR1 & I2C_CR1_STOP);
}

/**
 * Set the peripheral frequency.
 */
static inline void __i2c_set_freq(struct i2cdrv_t *i2c,
        struct rcc_clocks_freq *freq) {

    __i2c_reset_cr2(i2c, I2C_CR2_FREQ);
    __i2c_update_cr2(i2c, freq->pclk1 / 1000000);
}

void i2c_init(struct i2cdrv_t *drv, struct pin* sda, struct pin* scl) {
    pin_enable(sda);
    pin_enable(scl);
    rcc_enable(drv->hw->dev);

    drv->lock = xSemaphoreCreateMutex();
    vSemaphoreCreateBinary(drv->complete);
    xSemaphoreTake(drv->complete, portMAX_DELAY);

    /* sda : set open drain, alternate function mode, alternate function i2c */
    pin_set_otype(sda, PIN_TYPE_OPENDRAIN);
    pin_set_pupd (sda, PIN_PUPD_NONE);
    pin_set_af   (sda, PIN_AF_I2C);
    pin_set_mode (sda, PIN_MODE_AF);
    /* scl : open drain, alternate function mode, alternate function i2c */
    pin_set_otype(scl, PIN_TYPE_OPENDRAIN);
    pin_set_pupd (scl, PIN_PUPD_NONE);
    pin_set_af   (scl, PIN_AF_I2C);
    pin_set_mode (scl, PIN_MODE_AF);

    I2C_TypeDef* reg = drv->hw->reg;
    /* Reset and clear peripheral. */
    reg->CR1 = I2C_CR1_SWRST;
    reg->CR1 = 0;

    struct rcc_clocks_freq freq;
    rcc_get_clocks_freq(&freq);

    // enable the peripheral clock
    __i2c_set_freq(drv, &freq);

    // enable interrupts
    __i2c_update_cr2(drv,
          I2C_CR2_ITERREN  /* Error interrupt enable */
        | I2C_CR2_ITEVTEN  /* Event interrupt enable */
        | I2C_CR2_ITBUFEN  /* Buffer interrupt enable */
        );

    // fast speed init
    uint16_t result = (uint16_t)(freq.pclk1 / (400000 * 25));
    if (result < 1)
      result = 1;

    reg->CCR |= I2C_CCR_DUTY | I2C_CCR_FS | result;
    reg->TRISE = (uint16_t)((((freq.pclk1 / 1000000) * 300) / 1000) + 1);

    // standard speed init
#if 0
    uint16_t result = (uint16_t)(freq.pclk1 / (100000 << 1));
    if(result < 0x4) {
        result = 0x04;
    }
    reg->CCR |= result;
    reg->TRISE = ((freq.pclk1 / 1000000 + 1) & I2C_TRISE_TRISE);
#endif

    /* Enable interrupts globally */
    interrupt_set_priority(drv->hw->irq_ev, INTERRUPT_PRIORITY_FREERTOS_SAFE);
    interrupt_set_priority(drv->hw->irq_er, INTERRUPT_PRIORITY_FREERTOS_SAFE);
    interrupt_enable(drv->hw->irq_ev);
    interrupt_enable(drv->hw->irq_er);

    /* Enable the I2C peripheral */
    reg->CR1 = I2C_CR1_PE;
}

bool i2c_transfer(struct i2cdrv_t *drv, uint8_t addr,
                  const uint8_t *tx_buf, size_t tx_len,
                  uint8_t *rx_buf, size_t rx_len) {
    bool result = true;

    if (!xSemaphoreTake(drv->lock, portMAX_DELAY))
        return false;

    drv->addr    = addr;
    drv->writing = tx_len;
    drv->write_p = (uint8_t *)tx_buf;
    drv->reading = rx_len;
    drv->read_p  = rx_buf;

    __i2c_set_start(drv);
    if (!xSemaphoreTake(drv->complete, 1000)) {
        asm volatile("bkpt");
        result = false;
    }

    if (drv->error) {
        drv->error = false;
        result = false;
    }

    xSemaphoreGive(drv->lock);
    return result;
}

bool i2c_write(struct i2cdrv_t *drv, uint8_t addr, uint8_t *buf, size_t len) {
    return i2c_transfer(drv, addr, buf, len, NULL, 0);
}

bool i2c_write_reg(struct i2cdrv_t *drv, uint8_t addr,
        uint8_t reg, uint8_t data) {
    uint8_t buf[2] = { reg, data };
    return i2c_write(drv, addr, buf, sizeof(buf));
}

bool i2c_write_regs(struct i2cdrv_t *drv, uint8_t addr, uint8_t reg,
                    uint8_t *data, uint8_t len) {
    uint8_t buf[len + 1];    // requires C99, careful of stack overflow!
    buf[0] = reg;
    memcpy(&buf[1], data, len);
    return i2c_transfer(drv, addr, buf, len + 1, NULL, 0);
}

bool i2c_read_reg(struct i2cdrv_t *drv, uint8_t addr, uint8_t reg,
        uint8_t len, uint8_t *buf) {
    return i2c_transfer(drv, addr, &reg, 1, buf, len);
}

static void __i2c_event_irq_handler(struct i2cdrv_t *drv) {
    portBASE_TYPE should_yield = pdFALSE;

    /* Read both status registers*/
    uint16_t sr1 = drv->hw->reg->SR1;
    drv->hw->reg->SR2;

    /* Start bit sent. */
    if (sr1 & I2C_SR1_SB) {
        drv->hw->reg->DR = (drv->addr << 1) | (drv->writing ? 0 : 1);
    }

    /* Address sent. */
    if (sr1 & I2C_SR1_ADDR) {
        if (drv->writing) {
            /* Send a byte off the write buffer. */
            drv->hw->reg->DR = *(drv->write_p);
            drv->write_p++;
            drv->writing--;
        } else {
            if (drv->reading > 1) {
                /* Send ack on next read byte */
                drv->hw->reg->CR1 |= I2C_CR1_ACK;
            } else {
                /* One byte left to read, send stop afterwards. */
                __i2c_set_stop(drv);
            }
        }
    }

    /* RX Not empty (got new byte) */
    if (sr1 & I2C_SR1_RXNE) {
        /* Read into read buffer. */
        *(drv->read_p) = drv->hw->reg->DR;
        drv->read_p++;
        drv->reading--;

        if (drv->reading == 1) {
            /* Unset Ack, set Stop */
            drv->hw->reg->CR1 &= ~I2C_CR1_ACK;
            __i2c_set_stop(drv);
        }

        if (drv->reading == 0) {
            xSemaphoreGiveFromISR(drv->complete, &should_yield);
        }
    }

    /* TXE set, BTF clear: tx buffer empty, still writing. */
    if (sr1 & I2C_SR1_TXE && !(sr1 & I2C_SR1_BTF)) {
        if (drv->writing) {
            /* send next byte from write buffer. */
            drv->hw->reg->DR = *(drv->write_p);
            drv->write_p++;
            drv->writing--;
        } else {
            if (drv->reading) {
                /* done writing, now reading: send repeated stat */
                __i2c_set_start(drv);
            } else {
                /* done reading: send stop */
                __i2c_set_stop(drv);
                xSemaphoreGiveFromISR(drv->complete, &should_yield);
            }
        }
    }

    if (should_yield)
        taskYIELD();
}

static void __i2c_error_irq_handler(struct i2cdrv_t *drv) {
    portBASE_TYPE should_yield = pdFALSE;

    /* Read SRs to clear them */
    drv->hw->reg->SR1;
    drv->hw->reg->SR2;

    /* Write 0 to SR1 ?? XXX why  */
    drv->hw->reg->SR1 = 0;
    /* Send stop */
    __i2c_set_stop(drv);
    drv->error = 1;

    xSemaphoreGiveFromISR(drv->complete, &should_yield);

    if (should_yield)
        taskYIELD();
}

/* Static per-hw event handlers */

void I2C1_EV_IRQHandler (void) {
    __i2c_event_irq_handler(i2c1);
}

void I2C2_EV_IRQHandler (void) {
    __i2c_event_irq_handler(i2c2);
}

void I2C3_EV_IRQHandler (void) {
    __i2c_event_irq_handler(i2c3);
}

/* Static per-hw error handlers */

void I2C1_ER_IRQHandler (void) {
    __i2c_error_irq_handler(i2c1);
}

void I2C2_ER_IRQHandler (void) {
    __i2c_error_irq_handler(i2c2);
}

void I2C3_ER_IRQHandler (void) {
    __i2c_error_irq_handler(i2c3);
}


