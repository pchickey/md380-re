/*
 * usart.c --- STM32F4 USART driver.
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

#include "interrupt.h"
#include "rcc.h"
#include "gpio.h"
#include "usart.h"

#define BUFFER_SIZE     128

struct usart {
    USART_TypeDef *dev;

    // Clock config
    enum RCCDevice rcc_dev;

    // IRQ config
    enum IRQn irq;

    // GPIO config
    enum pin_af af;
    struct pin *tx_pin;
    struct pin *rx_pin;

    xQueueHandle rx_buf;
    xQueueHandle tx_buf;
};

#define INIT_USART   \
    .dev     = NULL, \
    .rcc_dev = 0,    \
    .irq     = 0,    \
    .af      = 0,    \
    .tx_pin  = NULL, \
    .rx_pin  = NULL, \
    .rx_buf  = NULL, \
    .tx_buf  = NULL

/**
 * @return true if the device is already enabled
 */
static inline bool __is_enabled(struct usart *usart) {
    return usart->dev->CR1 & USART_CR1_UE;
}

/**
 * Initialize a pin for the USART.
 */
static void __usart_init_pin(struct pin *pin, enum pin_af af) {
    pin_enable(pin);

    pin_set_ospeed(pin, PIN_SPEED_50MHZ);
    pin_set_otype (pin, PIN_TYPE_PUSHPULL);
    pin_set_pupd  (pin, PIN_PUPD_UP);

    pin_set_af(pin, af);
    pin_set_mode(pin, PIN_MODE_AF);
}

/**
 * Reset a value in CR1.
 *
 * @param usart USART device to use
 * @param val   Mask for the bits to reset
 */
static inline void __usart_reset_cr1(struct usart *usart, uint32_t val) {
    usart->dev->CR1 &= ~val;
}

/**
 * Update a value in CR1
 *
 * @param usart USART device to use
 * @param val   Maks to be added to CR1
 */
static inline void __usart_update_cr1(struct usart *usart, uint32_t val) {
    usart->dev->CR1 |= val;
}

/**
 * Reset a value in CR2.
 *
 * @param usart USART device to use
 * @param val   Mask for the bits to reset
 */
static inline void __usart_reset_cr2(struct usart *usart, uint32_t val) {
    usart->dev->CR2 &= ~val;
}

/**
 * Update a value in CR2
 *
 * @param usart USART device to use
 * @param val   Maks to be added to CR2
 */
static inline void __usart_update_cr2(struct usart *usart, uint32_t val) {
    usart->dev->CR2 |= val;
}

/**
 * Set the content of the data register
 */
static inline void __usart_set_dr(struct usart *usart, uint16_t val) {
    usart->dev->DR = val & 0x01ff;
}

/**
 * Get the content of the data register
 */
static inline uint16_t __usart_get_dr(struct usart *usart) {
    return (uint16_t)usart->dev->DR & 0x01ff;
}


enum usart_word_len {
    USART_WORD_LEN_8 = 0x0,
    USART_WORD_LEN_9 = 0x1
};

static inline void __usart_set_word_length(struct usart *usart,
        enum usart_word_len val) {

    __usart_reset_cr1(usart, USART_CR1_M);
    __usart_update_cr1(usart, (uint32_t)val << 12);

}


enum usart_stop_bits {
    USART_STOP_BITS_1_0 = 0x00, // 1   stop bit
    USART_STOP_BITS_0_5 = 0x01, // 0.5 stop bits
    USART_STOP_BITS_2_0 = 0x02, // 2.0 stop bits
    USART_STOP_BITS_1_5 = 0x03  // 1.5 stop bits
};

static inline void __usart_set_stop_bits(struct usart *usart,
        enum usart_stop_bits val) {

    __usart_reset_cr2(usart, USART_CR2_STOP);
    __usart_update_cr2(usart, (uint32_t)val << 12);
}


static inline void __usart_parity_enable(struct usart *usart) {
    __usart_update_cr1(usart, USART_CR1_PCE);
}

static inline void __usart_parity_disable(struct usart *usart) {
    __usart_reset_cr1(usart, USART_CR1_PCE);
}

enum usart_parity_mode {
    USART_PARITY_MODE_EVEN = 0x0,
    USART_PARITY_MODE_ODD  = 0x1
};

static inline void __usart_parity_mode(struct usart *usart,
        enum usart_parity_mode mode) {
    __usart_reset_cr1(usart, USART_CR1_PS);
    __usart_reset_cr1(usart, (uint32_t)mode << 9);
}

static inline void __usart_transmit_enable(struct usart *usart) {
    __usart_update_cr1(usart, USART_CR1_TE);
}

static inline void __usart_receive_enable(struct usart *usart) {
    __usart_update_cr1(usart, USART_CR1_RE);
}

/**
 * NOTE: this function was mostly adapted from ST's implementation in
 * stm32f4xx_usart.c
 */
static void __usart_set_baud_rate(struct usart *usart, uint32_t baud) {
    struct rcc_clocks_freq freq = {0};
    uint32_t apbclock, ipart, fpart, mask;

    rcc_get_clocks_freq(&freq);

    if(usart == usart1 || usart == usart6) {
        apbclock = freq.pclk2;
    } else {
        apbclock = freq.pclk1;
    }

    // integer part
    if(usart->dev->CR1 & USART_CR1_OVER8) {
        ipart = (25 * apbclock) / (2 * baud);
    } else {
        ipart = (25 * apbclock) / (4 * baud);
    }
    mask = (ipart / 100) << 4;

    // fractional part
    fpart = ipart - (100 * (mask >> 4));

    // implement the fractional part
    if(usart->dev->CR1 & USART_CR1_OVER8) {
        mask |= (((fpart * 8)  + 50) / 100) & 0x07;
    } else {
        mask |= (((fpart * 16) + 50) / 100) & 0x0f;
    }

    usart->dev->BRR = mask;
}

/**
 * Disable the transmit buffer empty interrupt.
 */
static inline void __usart_disable_txe(struct usart *usart) {
    __usart_reset_cr1(usart, USART_CR1_TXEIE);
}

/**
 * Enable the transmit buffer empty interrupt.
 */
static inline void __usart_enable_txe(struct usart *usart) {
    __usart_update_cr1(usart, USART_CR1_TXEIE);
}

/**
 * Enable the receive buffer not empty interrupt.
 */
static inline void __usart_enable_rxne(struct usart *usart) {
    __usart_update_cr1(usart, USART_CR1_RXNEIE);
}


/* External Interface *********************************************************/

bool usart_init(struct usart *usart, uint32_t baud) {

    if(__is_enabled(usart)) {
        return false;
    }

    // enable the clock
    rcc_enable(usart->rcc_dev);

    // initialize pins
    __usart_init_pin(usart->tx_pin, usart->af);
    __usart_init_pin(usart->rx_pin, usart->af);

    // initialize the uart, this may need to be abstracted out into parameters
    // to usart_init at some point, to allow for different config options.
    __usart_set_baud_rate  (usart, baud);
    __usart_set_stop_bits  (usart, USART_STOP_BITS_1_0);
    __usart_set_word_length(usart, USART_WORD_LEN_8);
    __usart_parity_disable (usart);
    __usart_transmit_enable(usart);
    __usart_receive_enable (usart);

    // setup IRQ
    interrupt_set_priority(usart->irq, INTERRUPT_PRIORITY_FREERTOS_SAFE);
    interrupt_enable(usart->irq);

    usart->tx_buf = xQueueCreate(BUFFER_SIZE, sizeof(uint8_t));
    usart->rx_buf = xQueueCreate(BUFFER_SIZE, sizeof(uint8_t));

    return true;
}

void usart_enable(struct usart *usart) {
    __usart_enable_rxne(usart);
    __usart_update_cr1(usart, USART_CR1_UE);
}

int32_t usart_write_timeout(struct usart *usart, uint32_t timeout,
                            const uint8_t *buf, uint32_t len) {

    int32_t i = 0;

    for(; i<len; ++i, ++buf) {
        if(pdTRUE != xQueueSend(usart->tx_buf, buf, timeout)) {
            break;
        }
    }

    // enable the transmit interrupt
    __usart_enable_txe(usart);

    return i;
}

int32_t usart_write_blocking(struct usart *usart, const uint8_t *buf,
                             uint32_t len)
{
  __usart_disable_txe(usart);

  for (uint32_t i = 0; i < len; ++i) {
    while (!(usart->dev->SR & USART_SR_TXE))
      ;
    usart->dev->DR = buf[i];
  }

  while (!(usart->dev->SR & USART_SR_TC))
    ;

  return len;
}

void usart_write_from_isr(struct usart *usart, const uint8_t *buf, uint32_t len)
{
  portBASE_TYPE should_yield = pdFALSE;

  for (uint32_t i = 0; i < len; ++i) {
    if (!xQueueSendFromISR(usart->tx_buf, &buf[i], &should_yield))
      break;
  }

  __usart_enable_txe(usart);

  if (should_yield)
    taskYIELD();
}

int32_t usart_read_timeout(struct usart *usart, uint32_t timeout, uint8_t *buf,
                           uint32_t len) {

    int32_t i = 0;

    for(; i<len; ++i, ++buf) {
        if(pdTRUE != xQueueReceive(usart->rx_buf, buf, timeout)) {
            break;
        }
    }

    return i;
}

bool usart_is_tx_pending(struct usart *usart)
{
  return uxQueueMessagesWaiting(usart->tx_buf) != 0;
}

uint32_t usart_available(struct usart *usart)
{
  return uxQueueMessagesWaiting(usart->rx_buf);
}

uint32_t usart_txspace(struct usart *usart)
{
  return BUFFER_SIZE - uxQueueMessagesWaiting(usart->tx_buf);
}

bool usart_peek(struct usart *usart, uint8_t *buf)
{
  if (xQueuePeek(usart->rx_buf, buf, 0))
    return true;
  return false;
}

/* IRQ Handlers ***************************************************************/

static void __usart_irq_handler(struct usart *usart) {

    int32_t shouldYield = pdFALSE;
    uint8_t byte = 0;

    uint32_t sr = usart->dev->SR;

    // TODO:  at this point in nuttx, they loop for a while to try to do as much
    // work as possible.  this seems like a good plan to avoid triggering an
    // interrupt for each character transmitted.

    // receive buffer is not empty
    if(sr & USART_SR_RXNE) {
        byte = (uint8_t)__usart_get_dr(usart);
        xQueueSendFromISR(usart->rx_buf, &byte, &shouldYield);
    }

    // transmit buffer is empty
    else if(sr & USART_SR_TXE) {
        if(pdTRUE == xQueueReceiveFromISR(usart->tx_buf, &byte, &shouldYield)) {
            __usart_set_dr(usart, byte);
        } else {
            __usart_disable_txe(usart);
        }
    }

    if(pdTRUE == shouldYield) {
        taskYIELD();
    }
}

void USART1_IRQHandler(void) {
    return __usart_irq_handler(usart1);
}

void USART2_IRQHandler(void) {
    return __usart_irq_handler(usart2);
}

void USART3_IRQHandler(void) {
    return __usart_irq_handler(usart3);
}

void UART4_IRQHandler(void) {
    return __usart_irq_handler(uart4);
}

void UART5_IRQHandler(void) {
    return __usart_irq_handler(uart5);
}

void USART6_IRQHandler(void) {
    return __usart_irq_handler(usart6);
}


/* USART Handle Definition ****************************************************/

struct usart _usart1 = {
    INIT_USART,
    .dev     = USART1,
    .rcc_dev = RCCDEV_USART1,
    .irq     = USART1_IRQn,
    .af      = PIN_AF_USART1,
    .tx_pin  = pin_b6,
    .rx_pin  = pin_b7,
};

struct usart _usart2 = {
    INIT_USART,
    .dev     = USART2,
    .rcc_dev = RCCDEV_USART2,
    .irq     = USART2_IRQn,
    .af      = PIN_AF_USART2,
    .tx_pin  = pin_a2,
    .rx_pin  = pin_a3,
};

struct usart _usart3 = {
    INIT_USART,
    .dev     = USART3,
    .rcc_dev = RCCDEV_USART3,
    .irq     = USART3_IRQn,
    .af      = PIN_AF_USART3,
    .tx_pin  = pin_b10,
    .rx_pin  = pin_b11,
};

struct usart _uart4 = {
    INIT_USART,
    .dev     = UART4,
    .rcc_dev = RCCDEV_UART4,
    .irq     = UART4_IRQn,
    .af      = PIN_AF_UART4,
    .tx_pin  = pin_c10,
    .rx_pin  = pin_c11,
};

struct usart _uart5 = {
    INIT_USART,
    .dev     = UART5,
    .rcc_dev = RCCDEV_UART5,
    .irq     = UART5_IRQn,
    .af      = PIN_AF_UART5,
    .tx_pin  = pin_c12,
    .rx_pin  = pin_d2,
};

struct usart _usart6 = {
    INIT_USART,
    .dev     = USART6,
    .rcc_dev = RCCDEV_USART6,
    .irq     = USART6_IRQn,
    .af      = PIN_AF_USART6,
    .tx_pin  = pin_c6,
    .rx_pin  = pin_c7,
};
