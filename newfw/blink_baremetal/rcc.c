/*
 * rcc.c --- STM32F4 peripheral clock driver.
 *
 * Copyright (C) 2012, Galois, Inc.
 * All Rights Reserved.
 *
 * This software is released under the "BSD3" license.  Read the file
 * "LICENSE" for more information.
 */

#include <stdbool.h>
#include <stdint.h>

#include <stm32f4xx.h>
#include "rcc.h"

void rcc_enable(enum RCCDevice d) {
    switch (d) {
    /* AHB1: */
    case RCCDEV_GPIOA:
        RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
        break;
    case RCCDEV_GPIOB:
        RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;
        break;
    case RCCDEV_GPIOC:
        RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN;
        break;
    case RCCDEV_GPIOD:
        RCC->AHB1ENR |= RCC_AHB1ENR_GPIODEN;
        break;
    case RCCDEV_GPIOE:
        RCC->AHB1ENR |= RCC_AHB1ENR_GPIOEEN;
        break;
    case RCCDEV_GPIOF:
        RCC->AHB1ENR |= RCC_AHB1ENR_GPIOFEN;
        break;
    case RCCDEV_GPIOG:
        RCC->AHB1ENR |= RCC_AHB1ENR_GPIOGEN;
        break;
    case RCCDEV_GPIOH:
        RCC->AHB1ENR |= RCC_AHB1ENR_GPIOHEN;
        break;
    case RCCDEV_GPIOI:
        RCC->AHB1ENR |= RCC_AHB1ENR_GPIOIEN;
        break;
    case RCCDEV_CRC:
        RCC->AHB1ENR |= RCC_AHB1ENR_CRCEN;
        break;
    case RCCDEV_DMA1:
        RCC->AHB1ENR |= RCC_AHB1ENR_DMA1EN;
        break;
    case RCCDEV_DMA2:
        RCC->AHB1ENR |= RCC_AHB1ENR_DMA2EN;
        break;
    case RCCDEV_ETHMAC:
        RCC->AHB1ENR |= RCC_AHB1ENR_ETHMACEN;
        break;
    case RCCDEV_OTGHS:
        RCC->AHB1ENR |= RCC_AHB1ENR_OTGHSEN;
        break;
    case RCCDEV_OTGHSULPI:
        RCC->AHB1ENR |= RCC_AHB1ENR_OTGHSULPIEN;
        break;
    /* AHB2: */
    case RCCDEV_DCMI:
        RCC->AHB2ENR |= RCC_AHB2ENR_DCMIEN;
        break;
    case RCCDEV_CRYP:
        RCC->AHB2ENR |= RCC_AHB2ENR_CRYPEN;
        break;
    case RCCDEV_HASH:
        RCC->AHB2ENR |= RCC_AHB2ENR_HASHEN;
        break;
    case RCCDEV_RNG:
        RCC->AHB2ENR |= RCC_AHB2ENR_RNGEN;
        break;
    case RCCDEV_OTGFS:
        RCC->AHB2ENR |= RCC_AHB2ENR_OTGFSEN;
        break;
    /* AHB3: */
    case RCCDEV_FSMC:
        RCC->AHB3ENR |= RCC_AHB3ENR_FSMCEN;
        break;
    /* APB1: */
    case RCCDEV_TIM2:
        RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;
        break;
    case RCCDEV_TIM3:
        RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;
        break;
    case RCCDEV_TIM4:
        RCC->APB1ENR |= RCC_APB1ENR_TIM4EN;
        break;
    case RCCDEV_TIM5:
        RCC->APB1ENR |= RCC_APB1ENR_TIM5EN;
        break;
    case RCCDEV_TIM6:
        RCC->APB1ENR |= RCC_APB1ENR_TIM6EN;
        break;
    case RCCDEV_TIM7:
        RCC->APB1ENR |= RCC_APB1ENR_TIM7EN;
        break;
    case RCCDEV_TIM12:
        RCC->APB1ENR |= RCC_APB1ENR_TIM12EN;
        break;
    case RCCDEV_TIM13:
        RCC->APB1ENR |= RCC_APB1ENR_TIM13EN;
        break;
    case RCCDEV_TIM14:
        RCC->APB1ENR |= RCC_APB1ENR_TIM14EN;
        break;
    case RCCDEV_WWDG:
        RCC->APB1ENR |= RCC_APB1ENR_WWDGEN;
        break;
    case RCCDEV_SPI2:
        RCC->APB1ENR |= RCC_APB1ENR_SPI2EN;
        break;
    case RCCDEV_SPI3:
        RCC->APB1ENR |= RCC_APB1ENR_SPI3EN;
        break;
    case RCCDEV_USART2:
        RCC->APB1ENR |= RCC_APB1ENR_USART2EN;
        break;
    case RCCDEV_USART3:
        RCC->APB1ENR |= RCC_APB1ENR_USART3EN;
        break;
    case RCCDEV_UART4:
        RCC->APB1ENR |= RCC_APB1ENR_UART4EN;
        break;
    case RCCDEV_UART5:
        RCC->APB1ENR |= RCC_APB1ENR_UART5EN;
        break;
    case RCCDEV_I2C1:
        RCC->APB1ENR |= RCC_APB1ENR_I2C1EN;
        break;
    case RCCDEV_I2C2:
        RCC->APB1ENR |= RCC_APB1ENR_I2C2EN;
        break;
    case RCCDEV_I2C3:
        RCC->APB1ENR |= RCC_APB1ENR_I2C3EN;
        break;
    case RCCDEV_CAN1:
        RCC->APB1ENR |= RCC_APB1ENR_CAN1EN;
        break;
    case RCCDEV_CAN2:
        RCC->APB1ENR |= RCC_APB1ENR_CAN2EN;
        break;
    case RCCDEV_PWR:
        RCC->APB1ENR |= RCC_APB1ENR_PWREN;
        break;
    case RCCDEV_DAC:
        RCC->APB1ENR |= RCC_APB1ENR_DACEN;
        break;
    /* APB2: */
    case RCCDEV_TIM1:
        RCC->APB2ENR |= RCC_APB2ENR_TIM1EN;
        break;
    case RCCDEV_TIM8:
        RCC->APB2ENR |= RCC_APB2ENR_TIM8EN;
        break;
    case RCCDEV_USART1:
        RCC->APB2ENR |= RCC_APB2ENR_USART1EN;
        break;
    case RCCDEV_USART6:
        RCC->APB2ENR |= RCC_APB2ENR_USART6EN;
        break;
    case RCCDEV_ADC1:
        RCC->APB2ENR |= RCC_APB2ENR_ADC1EN;
        break;
    case RCCDEV_ADC2:
        RCC->APB2ENR |= RCC_APB2ENR_ADC1EN;
        break;
    case RCCDEV_ADC3:
        RCC->APB2ENR |= RCC_APB2ENR_ADC1EN;
        break;
    case RCCDEV_SDIO:
        RCC->APB2ENR |= RCC_APB2ENR_SDIOEN;
        break;
    case RCCDEV_SPI1:
        RCC->APB2ENR |= RCC_APB2ENR_SPI1EN;
        break;
    case RCCDEV_SYSCFG:
        RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;
        break;
    case RCCDEV_TIM9:
        RCC->APB2ENR |= RCC_APB2ENR_TIM9EN;
        break;
    case RCCDEV_TIM10:
        RCC->APB2ENR |= RCC_APB2ENR_TIM10EN;
        break;
    case RCCDEV_TIM11:
        RCC->APB2ENR |= RCC_APB2ENR_TIM11EN;
        break;
    }
}

void rcc_disable(enum RCCDevice d) {
    switch (d) {
    /* AHB1: */
    case RCCDEV_GPIOA:
        RCC->AHB1ENR &= ~RCC_AHB1ENR_GPIOAEN;
        break;
    case RCCDEV_GPIOB:
        RCC->AHB1ENR &= ~RCC_AHB1ENR_GPIOBEN;
        break;
    case RCCDEV_GPIOC:
        RCC->AHB1ENR &= ~RCC_AHB1ENR_GPIOCEN;
        break;
    case RCCDEV_GPIOD:
        RCC->AHB1ENR &= ~RCC_AHB1ENR_GPIODEN;
        break;
    case RCCDEV_GPIOE:
        RCC->AHB1ENR &= ~RCC_AHB1ENR_GPIOEEN;
        break;
    case RCCDEV_GPIOF:
        RCC->AHB1ENR &= ~RCC_AHB1ENR_GPIOFEN;
        break;
    case RCCDEV_GPIOG:
        RCC->AHB1ENR &= ~RCC_AHB1ENR_GPIOGEN;
        break;
    case RCCDEV_GPIOH:
        RCC->AHB1ENR &= ~RCC_AHB1ENR_GPIOHEN;
        break;
    case RCCDEV_GPIOI:
        RCC->AHB1ENR &= ~RCC_AHB1ENR_GPIOIEN;
        break;
    case RCCDEV_CRC:
        RCC->AHB1ENR &= ~RCC_AHB1ENR_CRCEN;
        break;
    case RCCDEV_DMA1:
        RCC->AHB1ENR &= ~RCC_AHB1ENR_DMA1EN;
        break;
    case RCCDEV_DMA2:
        RCC->AHB1ENR &= ~RCC_AHB1ENR_DMA2EN;
        break;
    case RCCDEV_ETHMAC:
        RCC->AHB1ENR &= ~RCC_AHB1ENR_ETHMACEN;
        break;
    case RCCDEV_OTGHS:
        RCC->AHB1ENR &= ~RCC_AHB1ENR_OTGHSEN;
        break;
    case RCCDEV_OTGHSULPI:
        RCC->AHB1ENR &= ~RCC_AHB1ENR_OTGHSULPIEN;
        break;
    /* AHB2: */
    case RCCDEV_DCMI:
        RCC->AHB2ENR &= ~RCC_AHB2ENR_DCMIEN;
        break;
    case RCCDEV_CRYP:
        RCC->AHB2ENR &= ~RCC_AHB2ENR_CRYPEN;
        break;
    case RCCDEV_HASH:
        RCC->AHB2ENR &= ~RCC_AHB2ENR_HASHEN;
        break;
    case RCCDEV_RNG:
        RCC->AHB2ENR &= ~RCC_AHB2ENR_RNGEN;
        break;
    case RCCDEV_OTGFS:
        RCC->AHB2ENR &= ~RCC_AHB2ENR_OTGFSEN;
        break;
    /* AHB3: */
    case RCCDEV_FSMC:
        RCC->AHB3ENR &= ~RCC_AHB3ENR_FSMCEN;
        break;
    /* APB1: */
    case RCCDEV_TIM2:
        RCC->APB1ENR &= ~RCC_APB1ENR_TIM2EN;
        break;
    case RCCDEV_TIM3:
        RCC->APB1ENR &= ~RCC_APB1ENR_TIM3EN;
        break;
    case RCCDEV_TIM4:
        RCC->APB1ENR &= ~RCC_APB1ENR_TIM4EN;
        break;
    case RCCDEV_TIM5:
        RCC->APB1ENR &= ~RCC_APB1ENR_TIM5EN;
        break;
    case RCCDEV_TIM6:
        RCC->APB1ENR &= ~RCC_APB1ENR_TIM6EN;
        break;
    case RCCDEV_TIM7:
        RCC->APB1ENR &= ~RCC_APB1ENR_TIM7EN;
        break;
    case RCCDEV_TIM12:
        RCC->APB1ENR &= ~RCC_APB1ENR_TIM12EN;
        break;
    case RCCDEV_TIM13:
        RCC->APB1ENR &= ~RCC_APB1ENR_TIM13EN;
        break;
    case RCCDEV_TIM14:
        RCC->APB1ENR &= ~RCC_APB1ENR_TIM14EN;
        break;
    case RCCDEV_WWDG:
        RCC->APB1ENR &= ~RCC_APB1ENR_WWDGEN;
        break;
    case RCCDEV_SPI2:
        RCC->APB1ENR &= ~RCC_APB1ENR_SPI2EN;
        break;
    case RCCDEV_SPI3:
        RCC->APB1ENR &= ~RCC_APB1ENR_SPI3EN;
        break;
    case RCCDEV_USART2:
        RCC->APB1ENR &= ~RCC_APB1ENR_USART2EN;
        break;
    case RCCDEV_USART3:
        RCC->APB1ENR &= ~RCC_APB1ENR_USART3EN;
        break;
    case RCCDEV_UART4:
        RCC->APB1ENR &= ~RCC_APB1ENR_UART4EN;
        break;
    case RCCDEV_UART5:
        RCC->APB1ENR &= ~RCC_APB1ENR_UART5EN;
        break;
    case RCCDEV_I2C1:
        RCC->APB1ENR &= ~RCC_APB1ENR_I2C1EN;
        break;
    case RCCDEV_I2C2:
        RCC->APB1ENR &= ~RCC_APB1ENR_I2C2EN;
        break;
    case RCCDEV_I2C3:
        RCC->APB1ENR &= ~RCC_APB1ENR_I2C3EN;
        break;
    case RCCDEV_CAN1:
        RCC->APB1ENR &= ~RCC_APB1ENR_CAN1EN;
        break;
    case RCCDEV_CAN2:
        RCC->APB1ENR &= ~RCC_APB1ENR_CAN2EN;
        break;
    case RCCDEV_PWR:
        RCC->APB1ENR &= ~RCC_APB1ENR_PWREN;
        break;
    case RCCDEV_DAC:
        RCC->APB1ENR &= ~RCC_APB1ENR_DACEN;
        break;
    /* APB2: */
    case RCCDEV_TIM1:
        RCC->APB2ENR &= ~RCC_APB2ENR_TIM1EN;
        break;
    case RCCDEV_TIM8:
        RCC->APB2ENR &= ~RCC_APB2ENR_TIM8EN;
        break;
    case RCCDEV_USART1:
        RCC->APB2ENR &= ~RCC_APB2ENR_USART1EN;
        break;
    case RCCDEV_USART6:
        RCC->APB2ENR &= ~RCC_APB2ENR_USART6EN;
        break;
    case RCCDEV_ADC1:
        RCC->APB2ENR &= ~RCC_APB2ENR_ADC1EN;
        break;
    case RCCDEV_ADC2:
        RCC->APB2ENR &= ~RCC_APB2ENR_ADC1EN;
        break;
    case RCCDEV_ADC3:
        RCC->APB2ENR &= ~RCC_APB2ENR_ADC1EN;
        break;
    case RCCDEV_SDIO:
        RCC->APB2ENR &= ~RCC_APB2ENR_SDIOEN;
        break;
    case RCCDEV_SPI1:
        RCC->APB2ENR &= ~RCC_APB2ENR_SPI1EN;
        break;
    case RCCDEV_SYSCFG:
        RCC->APB2ENR &= ~RCC_APB2ENR_SYSCFGEN;
        break;
    case RCCDEV_TIM9:
        RCC->APB2ENR &= ~RCC_APB2ENR_TIM9EN;
        break;
    case RCCDEV_TIM10:
        RCC->APB2ENR &= ~RCC_APB2ENR_TIM10EN;
        break;
    case RCCDEV_TIM11:
        RCC->APB2ENR &= ~RCC_APB2ENR_TIM11EN;
        break;
    }
}

static uint8_t apb_presc_table[16] = { 0, 0, 0, 0,
                                       1, 2, 3, 4 };

static uint8_t ahb_presc_table[16] = { 0, 0, 0, 0, 0, 0, 0, 0,
                                       1, 2, 3, 4, 6, 7, 8, 9 };

/**
 * NOTE: This function is mostly adapted from the ST bundle for the stm32f4xx
 */
void rcc_get_clocks_freq(struct rcc_clocks_freq *freq) {

    uint32_t sws = RCC->CFGR & RCC_CFGR_SWS;
    switch(sws) {
        case 0x00: // HSI is the system clock source
            freq->sysclk = HSI_VALUE;
            break;

        case 0x04: // HSE is the system clock source
            freq->sysclk = HSE_VALUE;
            break;

        case 0x08: // pll is the system clock source
            {
                bool pllsource = (RCC->PLLCFGR & RCC_PLLCFGR_PLLSRC) >> 22;
                uint32_t source;

                if(pllsource) {
                    source = HSE_VALUE;
                } else {
                    source = HSI_VALUE;
                }

                uint32_t pllm   =  RCC->PLLCFGR & RCC_PLLCFGR_PLLM;
                uint32_t plln   = (RCC->PLLCFGR & RCC_PLLCFGR_PLLN) >> 6;
                uint32_t pllvco = (source / pllm) * plln;

                // the pllp register is in [0,3], so adding one and doubling it
                // puts it in [2,4,6,8]
                uint32_t pllp = (((RCC->PLLCFGR & RCC_PLLCFGR_PLLP) >> 16) + 1) * 2;

                freq->sysclk = pllvco / pllp;
            }
            break;

        default:
            freq->sysclk = HSI_VALUE;
            break;
    }

    uint32_t presc = 0;

    // AHB prescalar
    presc      = (RCC->CFGR & RCC_CFGR_HPRE) >> 4;
    freq->hclk = freq->sysclk >> ahb_presc_table[presc];

    // low-speed prescalar (APB1)
    presc       = (RCC->CFGR & RCC_CFGR_PPRE1) >> 10;
    freq->pclk1 = freq->hclk >> apb_presc_table[presc];

    // high-speed prescalar (APB2)
    presc       = (RCC->CFGR & RCC_CFGR_PPRE2) >> 13;
    freq->pclk2 = freq->hclk >> apb_presc_table[presc];
}
