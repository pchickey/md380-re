/*
 * rcc.h --- STM32F4 peripheral clock driver.
 *
 * Copyright (C) 2012, Galois, Inc.
 * All Rights Reserved.
 *
 * This software is released under the "BSD3" license.  Read the file
 * "LICENSE" for more information.
 */

#ifndef __rcc_h
#define __rcc_h

#ifdef __cplusplus__
extern C {
#endif


enum RCCDevice {
    /* AHB1: */
    RCCDEV_GPIOA,
    RCCDEV_GPIOB,
    RCCDEV_GPIOC,
    RCCDEV_GPIOD,
    RCCDEV_GPIOE,
    RCCDEV_GPIOF,
    RCCDEV_GPIOG,
    RCCDEV_GPIOH,
    RCCDEV_GPIOI,
    RCCDEV_CRC,
    RCCDEV_DMA1,
    RCCDEV_DMA2,
    RCCDEV_ETHMAC,
    RCCDEV_OTGHS,
    RCCDEV_OTGHSULPI,
    /* AHB2: */
    RCCDEV_DCMI,
    RCCDEV_CRYP,
    RCCDEV_HASH,
    RCCDEV_RNG,
    RCCDEV_OTGFS,
    /* AHB3: */
    RCCDEV_FSMC,
    /* APB1: */
    RCCDEV_TIM2,
    RCCDEV_TIM3,
    RCCDEV_TIM4,
    RCCDEV_TIM5,
    RCCDEV_TIM6,
    RCCDEV_TIM7,
    RCCDEV_TIM12,
    RCCDEV_TIM13,
    RCCDEV_TIM14,
    RCCDEV_WWDG,
    RCCDEV_SPI2,
    RCCDEV_SPI3,
    RCCDEV_USART2,
    RCCDEV_USART3,
    RCCDEV_UART4,
    RCCDEV_UART5,
    RCCDEV_I2C1,
    RCCDEV_I2C2,
    RCCDEV_I2C3,
    RCCDEV_CAN1,
    RCCDEV_CAN2,
    RCCDEV_PWR,
    RCCDEV_DAC,
    /* APB2: */
    RCCDEV_TIM1,
    RCCDEV_TIM8,
    RCCDEV_USART1,
    RCCDEV_USART6,
    RCCDEV_ADC1,
    RCCDEV_ADC2,
    RCCDEV_ADC3,
    RCCDEV_SDIO,
    RCCDEV_SPI1,
    RCCDEV_SYSCFG,
    RCCDEV_TIM9,
    RCCDEV_TIM10,
    RCCDEV_TIM11
};

void rcc_enable(enum RCCDevice);
void rcc_disable(enum RCCDevice);

struct rcc_clocks_freq {
    uint32_t sysclk;
    uint32_t hclk;
    uint32_t pclk1;
    uint32_t pclk2;
};

void rcc_get_clocks_freq(struct rcc_clocks_freq *freq);

#ifdef __cplusplus__
}
#endif

#endif /* __rcc.h */
