/*
 * spi.c --- STM32F4 SPI driver.
 *
 * Copyright (C) 2012, Galois, Inc.
 * All Rights Reserved.
 *
 * This software is released under the "BSD3" license.  Read the file
 * "LICENSE" for more information.
 */

#include <stdint.h>

#include <stm32f4xx.h>

#include <FreeRTOS.h>
#include <queue.h>
#include <task.h>
#include <semphr.h>

#include "interrupt.h"
#include "rcc.h"
#include "gpio.h"
#include "spi.h"

/*
 * Utility macros to make busy waits obvious.  These could be moved to
 * a general utility header.
 */

/** Busy wait until a condition is true. */
#define BUSY_UNTIL(cond)                        \
  do {                                          \
    while (!(cond)) {}                          \
  } while (0)

/** Busy wait until a condition is false. */
#define BUSY_WHILE(cond) BUSY_UNTIL(!(cond))

/** Busy wait until a bit in a register is set. */
#define BUSY_UNTIL_SET(reg, bit) BUSY_UNTIL((reg) & (bit))

/** Busy wait until a bit in a register is clear. */
#define BUSY_UNTIL_CLEAR(reg, bit) BUSY_WHILE((reg) & (bit))

/** Debug LED for testing the SPI IRQ. */
// #define DEBUG_LED (pin_d15)

/** Error LED for testing. */
#define ERROR_LED (pin_d14)

/** SPI error bits in the status register. */
#define SPI_ERROR_BITS (SPI_SR_OVR | SPI_SR_MODF | \
                        SPI_SR_CRCERR | SPI_SR_UDR)

/**
 * Information about the current transfer in progress.
 *
 * The "spi_transfer" function fills in these fields, and they are
 * updated by the SPI ISR as characters are transmitted and received.
 */
struct spi_transfer {
  volatile const uint8_t *tx_buf; /* transmit buffer */
  volatile uint8_t *rx_buf;       /* receive buffer */
  volatile size_t tx_len;         /* tx length */
  volatile size_t rx_len;         /* rx length */
};

struct spi_bus {
  // Peripheral configuration.
  SPI_TypeDef *dev;             /* peripheral registers */
  enum RCCDevice rcc_dev;       /* RCC clock */
  enum IRQn irq;                /* IRQ number */

  // GPIO pin configuration.
  enum pin_af af;               /* GPIO alternate function */
  struct pin *mosi_pin;         /* MOSI GPIO pin */
  struct pin *miso_pin;         /* MISO GPIO pin */
  struct pin *sck_pin;          /* SCK GPIO pin */

  // Current transfer in progress.
  struct spi_transfer transfer;

  // Synchronization.
  xSemaphoreHandle lock;        /* bus mutex */
  xSemaphoreHandle complete;    /* completion signal from ISR */
};

/** Clear some bits in the SPI CR1 register. */
static inline void spi_cr1_clear(struct spi_bus *bus, uint32_t val)
{
  bus->dev->CR1 &= ~val;
}

/** Set some bits in the SPI CR1 register. */
static inline void spi_cr1_set(struct spi_bus *bus, uint32_t val)
{
  bus->dev->CR1 |= val;
}

/** Clear some bits in the SPI CR2 register. */
static inline void spi_cr2_clear(struct spi_bus *bus, uint32_t val)
{
  bus->dev->CR2 &= ~val;
}

/** Set some bits in the SPI CR2 register. */
static inline void spi_cr2_set(struct spi_bus *bus, uint32_t val)
{
  bus->dev->CR2 |= val;
}

/** Configure the baud rate setting for a SPI bus. */
static inline void spi_set_baud(struct spi_bus *bus, enum spi_baud baud)
{
  spi_cr1_clear(bus, SPI_CR1_BR);
  spi_cr1_set(bus, (uint32_t)baud << 3);
}

/** Configure the clock polarity for a SPI bus. */
static inline void spi_set_clock_polarity(struct spi_bus *bus,
                                          enum spi_clock_polarity cpol)
{
  spi_cr1_clear(bus, SPI_CR1_CPOL);
  spi_cr1_set(bus, (uint32_t)cpol << 1);
}

/** Configure the clock phase for a SPI bus. */
static inline void spi_set_clock_phase(struct spi_bus *bus,
                                       enum spi_clock_phase cpha)
{
  spi_cr1_clear(bus, SPI_CR1_CPHA);
  spi_cr1_set(bus, (uint32_t)cpha);
}

/** Configure the bit order for a SPI bus. */
static inline void spi_set_bit_order(struct spi_bus *bus,
                                     enum spi_bit_order val)
{
  spi_cr1_clear(bus, SPI_CR1_LSBFIRST);
  spi_cr1_set(bus, (uint32_t)val << 7);
}

/**
 * Enable and configure a SPI bus for a device.
 */
static void spi_enable(struct spi_bus *bus, struct spi_device *dev)
{
  /* Ensure the peripheral is disabled, then configure it. */
  spi_cr1_clear(bus, SPI_CR1_SPE);

  bus->dev->CR1 = 0;
  bus->dev->CR2 = 0;

  /* The SSM and SSI bits are necessary to prevent the SPI peripheral
   * from disabling SPI if the NSS pin is not high.  Since we are
   * assuming all devices use GPIOs for slave selection, this should
   * be the right thing.  If that changes, we will need to make this
   * configurable. */
  spi_cr1_set(bus, SPI_CR1_MSTR | SPI_CR1_SSM | SPI_CR1_SSI);
  spi_set_baud(bus, dev->baud);
  spi_set_clock_polarity(bus, dev->clock_polarity);
  spi_set_clock_phase(bus, dev->clock_phase);
  spi_set_bit_order(bus, dev->bit_order);
  spi_cr1_set(bus, SPI_CR1_SPE);
}

/**
 * Disable the SPI peripheral for the given bus.
 */
static void spi_disable(struct spi_bus *bus)
{
  spi_cr1_clear(bus, SPI_CR1_SPE);
}

/** Select a device, asserting its chip select pin. */
static inline void spi_device_select(struct spi_device *dev)
{
  if (dev->chip_select_active)
    pin_set(dev->chip_select);
  else
    pin_reset(dev->chip_select);
}

/** Deselect a device, de-asserting its chip select pin. */
static inline void spi_device_deselect(struct spi_device *dev)
{
  if (dev->chip_select_active)
    pin_reset(dev->chip_select);
  else
    pin_set(dev->chip_select);
}

/**
 * Handle an IRQ for the given SPI bus.
 *
 * This is actually quite tricky---be very careful making changes to
 * this ISR.
 *
 * It's important to note here that both RXNE and TXE can be set in
 * the status register in the same interrupt.  This can happen when
 * another task disables interrupts during a transfer.
 *
 * To make sure that we never drop any received bytes, we always
 * alternate between TXing and RXing by toggling the TX/RX interrupts
 * as they occur.
 *
 * This makes us slightly less than optimal, as we could load the
 * second byte into the TX register before we've RX'ed the first byte,
 * but the additional complexity doesn't warrant complicating this ISR
 * any further until we need the extra performance.
 */
static void spi_irq(struct spi_bus *bus)
{
  portBASE_TYPE should_yield = pdFALSE;
  uint32_t sr = bus->dev->SR;

#ifdef DEBUG_LED
  pin_set(DEBUG_LED);
#endif

  if (sr & SPI_SR_RXNE) {
    *bus->transfer.rx_buf = bus->dev->DR;
    ++bus->transfer.rx_buf;
    --bus->transfer.rx_len;

    if (bus->transfer.rx_len == 0) {
      spi_cr2_clear(bus, SPI_CR2_RXNEIE);
      xSemaphoreGiveFromISR(bus->complete, &should_yield);
    } else {
      spi_cr2_set(bus, SPI_CR2_TXEIE);
    }
  } else if (sr & SPI_SR_TXE) {
    spi_cr2_clear(bus, SPI_CR2_TXEIE);

    if (bus->transfer.tx_len != 0) {
      spi_cr2_set(bus, SPI_CR2_RXNEIE);
      bus->dev->DR = *bus->transfer.tx_buf;

      ++bus->transfer.tx_buf;
      --bus->transfer.tx_len;
    }
  }

  if (sr & SPI_ERROR_BITS) {
#ifdef ERROR_LED
    pin_set(ERROR_LED);
#endif
  }

#ifdef DEBUG_LED
  pin_reset(DEBUG_LED);
#endif

  if (should_yield == pdTRUE)
    taskYIELD();
}

/*
 * Public Interface
 */

bool spi_init(struct spi_bus *bus)
{
#ifdef DEBUG_LED
  /* Grab a debug LED for testing. */
  pin_enable(DEBUG_LED);
  pin_reset(DEBUG_LED);
  pin_set_mode(DEBUG_LED, PIN_MODE_OUTPUT);
  pin_set_otype(DEBUG_LED, PIN_TYPE_PUSHPULL);
  pin_set_ospeed(DEBUG_LED, PIN_SPEED_2MHZ);
#endif

#ifdef ERROR_LED
  pin_enable(ERROR_LED);
  pin_reset(ERROR_LED);
  pin_set_mode(ERROR_LED, PIN_MODE_OUTPUT);
  pin_set_otype(ERROR_LED, PIN_TYPE_PUSHPULL);
  pin_set_ospeed(ERROR_LED, PIN_SPEED_2MHZ);
#endif

  rcc_enable(bus->rcc_dev);

  /* Configure GPIO pins for SPI operation. */
  pin_enable(bus->miso_pin);
  pin_set_af(bus->miso_pin, bus->af);
  pin_set_mode(bus->miso_pin, PIN_MODE_AF);
  pin_set_pupd(bus->miso_pin, PIN_PUPD_NONE);

  pin_enable(bus->mosi_pin);
  pin_set_af(bus->mosi_pin, bus->af);
  pin_set_mode(bus->mosi_pin, PIN_MODE_AF);
  pin_set_otype(bus->mosi_pin, PIN_TYPE_PUSHPULL);

  pin_enable(bus->sck_pin);
  pin_set_af(bus->sck_pin, bus->af);
  pin_set_mode(bus->sck_pin, PIN_MODE_AF);
  pin_set_otype(bus->sck_pin, PIN_TYPE_PUSHPULL);

  bus->lock = xSemaphoreCreateMutex();
  if (bus->lock == NULL)
    goto fail;

  vSemaphoreCreateBinary(bus->complete);
  if (bus->complete == NULL)
    goto fail;

  /* Take the semaphore initially so we will block next time. */
  xSemaphoreTake(bus->complete, portMAX_DELAY);

  interrupt_set_priority(bus->irq, INTERRUPT_PRIORITY_FREERTOS_SAFE);
  return true;

fail:
  if (bus->lock != NULL)
    vSemaphoreDelete(bus->lock);
  if (bus->complete != NULL)
    vSemaphoreDelete(bus->complete);

  return false;
}

void spi_device_init(struct spi_device *device)
{
  pin_enable(device->chip_select);
  spi_device_deselect(device);

  pin_set_mode(device->chip_select, PIN_MODE_OUTPUT);
  pin_set_otype(device->chip_select, PIN_TYPE_PUSHPULL);
  pin_set_ospeed(device->chip_select, PIN_SPEED_2MHZ);
}

/**
 * Perform a SPI read/write transfer using interrupts.
 */
ssize_t spi_transfer(struct spi_bus *bus, struct spi_device *dev,
                     uint32_t timeout, const uint8_t *tx_buf,
                     uint8_t *rx_buf, size_t len)
{
  bool error = false;

  if (len == 0)                /* ensure at least one byte of TX/RX */
    return 0;

  if (xSemaphoreTake(bus->lock, timeout) == pdFALSE)
    return -1;

  spi_enable(bus, dev);
  spi_device_select(dev);

  bus->transfer.tx_buf = tx_buf;
  bus->transfer.rx_buf = rx_buf;
  bus->transfer.tx_len = len;
  bus->transfer.rx_len = len;

  interrupt_enable(bus->irq);
  spi_cr2_set(bus, SPI_CR2_TXEIE | SPI_CR2_ERRIE);

  /* XXX We may not be able to successfully communicate on this bus
   * after we time out like this without doing something special to
   * clear the error state.  Most likely we're going to panic anyway,
   * though. */
  if (!xSemaphoreTake(bus->complete, timeout))
    error = true;

  spi_cr2_clear(bus, SPI_CR2_TXEIE | SPI_CR2_RXNEIE);
  interrupt_disable(bus->irq);

  /* According to the reference manual, we must wait until TXE=1 and
   * BSY=0 before disabling the SPI.  Don't bother doing this if the
   * transfer has timed out. */
  if (error) {
    BUSY_UNTIL_SET(bus->dev->SR, SPI_SR_TXE);
    BUSY_UNTIL_CLEAR(bus->dev->SR, SPI_SR_BSY);
  }

  spi_device_deselect(dev);
  spi_disable(bus);

  xSemaphoreGive(bus->lock);
  return error ? -1 : len - bus->transfer.rx_len;
}

struct spi_bus _spi1 = {
  SPI1,                         /* dev */
  RCCDEV_SPI1,                  /* rcc_dev */
  SPI1_IRQn,                    /* irq */
  PIN_AF_SPI1,                  /* af */
  pin_a7,                       /* mosi_pin */
  pin_a6,                       /* miso_pin */
  pin_a5,                       /* sck_pin */
  {                             /* transfer */
    NULL,                       /*  tx_buf */
    NULL,                       /*  rx_buf */
    0,                          /*  tx_len */
    0                           /*  rx_len */
  },
  NULL,                         /* lock */
  NULL                          /* complete */
};

struct spi_bus _spi2 = {
  SPI2,                         /* dev */
  RCCDEV_SPI2,                  /* rcc_dev */
  SPI2_IRQn,                    /* irq */
  PIN_AF_SPI2,                  /* af */
  pin_c3,                       /* mosi_pin */
  pin_c2,                       /* miso_pin */
  pin_b10,                      /* sck_pin */
  {                             /* transfer */
    NULL,                       /*  tx_buf */
    NULL,                       /*  rx_buf */
    0,                          /*  tx_len */
    0                           /*  rx_len */
  },
  NULL,                         /* lock */
  NULL                          /* complete */
};

struct spi_bus _spi3 = {
  SPI3,                         /* dev */
  RCCDEV_SPI3,                  /* rcc_dev */
  SPI3_IRQn,                    /* irq */
  PIN_AF_SPI3,                  /* af */
  pin_c12,                      /* mosi_pin */
  pin_c11,                      /* miso_pin */
  pin_c10,                      /* sck_pin */
  {                             /* transfer */
    NULL,                       /*  tx_buf */
    NULL,                       /*  rx_buf */
    0,                          /*  tx_len */
    0                           /*  rx_len */
  },
  NULL,                         /* lock */
  NULL                          /* complete */
};

/*
 * Interrupt Handlers
 */

void SPI1_IRQHandler(void)
{
  spi_irq(spi1);
}

void SPI2_IRQHandler(void)
{
  spi_irq(spi2);
}

void SPI3_IRQHandler(void)
{
  spi_irq(spi3);
}
