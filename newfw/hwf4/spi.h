/*
 * spi.h --- STM32F4 SPI driver.
 *
 * Copyright (C) 2012, Galois, Inc.
 * All Rights Reserved.
 *
 * This software is released under the "BSD3" license.  Read the file
 * "LICENSE" for more information.
 */

#ifndef __hwf4_spi_h
#define __hwf4_spi_h

#ifdef __cplusplus
extern "C" {
#endif

#include <FreeRTOS.h>

#include <stdbool.h>
#include <stdint.h>
#include <sys/types.h>

#include "gpio.h"

/**
 * Abstract data type representing one SPI bus in the STM32F4.
 */
struct spi_bus;

/**
 * Divisors for SPI device baud rate selection.
 *
 * The SPI frequency will be fPCLK divided by this divisor.
 */
enum spi_baud {
  SPI_BAUD_DIV_2   = 0x00,
  SPI_BAUD_DIV_4   = 0x01,
  SPI_BAUD_DIV_8   = 0x02,
  SPI_BAUD_DIV_16  = 0x03,
  SPI_BAUD_DIV_32  = 0x04,
  SPI_BAUD_DIV_64  = 0x05,
  SPI_BAUD_DIV_128 = 0x06,
  SPI_BAUD_DIV_256 = 0x07
};

/**
 * Clock polarity for SPI devices.
 */
enum spi_clock_polarity {
  SPI_CLOCK_POLARITY_LOW  = 0x00, /* clock to 0 when idle */
  SPI_CLOCK_POLARITY_HIGH = 0x01  /* clock to 1 when idle */
};

/**
 * Clock phase for SPI devices.
 */
enum spi_clock_phase {
  SPI_CLOCK_PHASE_1 = 0x00,     /* latch on 1st clock edge */
  SPI_CLOCK_PHASE_2 = 0x01      /* latch on 2nd clock edge */
};

/**
 * Bit order for SPI devices.
 */
enum spi_bit_order {
  SPI_BIT_ORDER_MSB_FIRST = 0x00, /* MSB transmitted first */
  SPI_BIT_ORDER_LSB_FIRST = 0x01  /* LSB transmitted first */
};

/**
 * Information about a device connected to a SPI bus.
 *
 * Each device driver will fill in an instance of this structure
 * containing the configuration of its device.  The SPI functions
 * accept both the bus to communicate over and the device containing
 * the device-specific configuration.
 */
struct spi_device {
  struct pin *chip_select;                /* chip select pin */
  bool chip_select_active;                /* CS active high if true */
  enum spi_baud baud;                     /* baud rate divisor */
  enum spi_clock_polarity clock_polarity; /* SCLK polarity */
  enum spi_clock_phase clock_phase;       /* SCLK phase */
  enum spi_bit_order bit_order;           /* frame format (bit order) */
};

/**
 * Initialize a SPI bus.
 *
 * This starts a task to manage SPI transfers for this bus.  The task
 * wakes up when transfers are started by "spi_transfer", and sleeps
 * when there are no transfers to process.
 *
 * @returns true on success, false if an error occurs
 */
bool spi_init(struct spi_bus *bus);

/**
 * Initialize a SPI device.
 *
 * This configures the chip select pin and deselects the chip.
 */
void spi_device_init(struct spi_device *device);

/**
 * Perform a read/write SPI transfer for a bus and device.
 *
 * The driver will write "len" bytes from "tx_buf" over the specified
 * bus, writing the bytes received to "rx_buf".  It is permissible for
 * "tx_buf" and "rx_buf" to point to the same buffer to do an in-place
 * read/write.
 *
 * If more than "timeout" milliseconds elapse without the SPI bus
 * becoming available, the function will time out and return -1.
 *
 * @returns the number of bytes read/written, -1 if an error or
 *          timeout occurs.
 */
ssize_t spi_transfer(struct spi_bus *bus, struct spi_device *dev,
                     uint32_t timeout, const uint8_t *tx_buf,
                     uint8_t *rx_buf, size_t len);

/* SPI Bus Handles */

extern struct spi_bus _spi1;
#define spi1 (&_spi1)

extern struct spi_bus _spi2;
#define spi2 (&_spi2)

extern struct spi_bus _spi3;
#define spi3 (&_spi3)

#ifdef __cplusplus
}
#endif

#endif  /* __hwf4_spi_h */
