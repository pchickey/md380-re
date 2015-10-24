/*
 * timer.c --- STM32F4 tick timer and PPM decoder.
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

#include <FreeRTOS.h>
#include <task.h>

#include "gpio.h"
#include "interrupt.h"
#include "timer.h"

/**
 * Timer to use for tick counting and PPM decoding.  We also define
 * the timer clock frequency here, and it must match the SYSCLK and
 * APB prescalar settings in the startup code.
 *
 * Note that the timer clock is 2x the APB peripheral clock when the
 * APB prescalar is not /1, according to the reference manual.
 *
 * On the PX4FMU, the PPM input is on TIM1_CH3, so we use TIM1 for
 * this, even though one of the less capable timers would do.  Also,
 * TIM1 has multiple IRQs that are shared with other timers, so this
 * starts to get rather complex.  For now, let's assume nobody is
 * using TIM10 so we don't need to worry about IRQ sharing.
 */
#define TIMER_DEV    TIM1
#define TIMER_F_CLK  168000000UL
#define TIMER_RCCDEV RCCDEV_TIM1

/** Capture/compare channel to use for PPM decoding. */
#define TIMER_PPM_CHANNEL 3
#define TIMER_PPM_PIN_AF  PIN_AF_TIM1

/**
 * Priority of timer interrupts.  This does not need to be a FreeRTOS
 * API safe priority.
 */
#define TIMER_IRQ_PRIORITY INTERRUPT_PRIORITY_HIGHEST

/* For timers with separate IRQs: */
#define TIMER_UP_IRQ TIM1_UP_TIM10_IRQn
#define TIMER_UP_ISR TIM1_UP_TIM10_IRQHandler
#define TIMER_CC_IRQ TIM1_CC_IRQn
#define TIMER_CC_ISR TIM1_CC_IRQHandler

/* For timers with a single IRQ: */
/* #define TIMER_IRQ TIMx_IRQn */
/* #define TIMER_ISR TIMx_IRQHandler */

/* Ensure that TIMER_F_CLK is a multiple of 1MHz. */
#if ((TIMER_F_CLK % 1000000) != 0)
# error "TIMER_F_CLK is not a multiple of 1MHz."
#endif

/** Prescalar value used to run a timer at 1MHz. */
#define TIMER_PRESCALAR (((TIMER_F_CLK) / 1000000UL) - 1)

/*
 * Capture/compare registers and bits for our PPM channel.
 *
 * Currently only channel 3 is supported, since that is where the PPM
 * pin is connected on the PX4IO board.
 *
 * Note also that CC1 is used to implement "timer_usleep".
 */
#if (TIMER_PPM_CHANNEL == 3)
# define PPM_PIN      pin_a10
# define CCR_PPM      CCR3
# define DIER_PPM     TIM_DIER_CC3IE
# define SR_CCIF_PPM  TIM_SR_CC3IF
# define SR_CCOF_PPM  TIM_SR_CC3OF
# define CCMR1_PPM    0
# define CCMR2_PPM    1
# define CCER_PPM     (TIM_CCER_CC3E | TIM_CCER_CC3P | TIM_CCER_CC3NP)
#else
# error "Unsupported timer PPM channel."
#endif

/**
 * Running tick count in microseconds.  This is incremented on each
 * timer overflow by 2**16 and should be added to TIMER_DEV->CNT to
 * get the current time in ticks.
 */
static volatile uint64_t g_ticks;

/** Boolean signalling completion of a microsecond delay. */
static volatile bool g_delay_complete;

/* PPM timings, in microseconds. */
#define PPM_LOW_MAX   500       /* max width of low pulse */
#define PPM_HIGH_MIN  800       /* min width of high data pulse */
#define PPM_HIGH_MAX  2200      /* max width of high data pulse */
#define PPM_START_MIN 2500      /* min width of start high pulse */

/** State of the PPM decoder. */
volatile struct {
  uint16_t last_edge;           /* last capture time */
  uint16_t last_mark;           /* last low->high edge */

  /* Sample buffer being written to from the decoder. */
  uint16_t current_buf[PPM_MAX_CHANNELS];
  size_t   current_len;

  /* Last full sample buffer returned to the caller. */
  uint16_t sample_buf[PPM_MAX_CHANNELS];
  size_t   sample_len;

  timer_tick_t sample_time;     /* sample timestamp */

  enum {
    SYNC = 0,                   /* waiting for start pulse */
    ARMING,                     /* waiting for first low edge */
    ACTIVE,                     /* waiting for low edge */
    INACTIVE                    /* waiting for high edge */
  } state;                      /* current decoder state */
} g_ppm_state;

void timer_init(void)
{
  g_ticks = 0;

  rcc_enable(TIMER_RCCDEV);

  /* Configure the PPM input pin for input capture. */
  pin_enable(PPM_PIN);
  pin_set_af(PPM_PIN, TIMER_PPM_PIN_AF);
  pin_set_pupd(PPM_PIN, PIN_PUPD_UP);
  pin_set_mode(PPM_PIN, PIN_MODE_AF);

#if 0
  /* Configure the debugger to freeze TIM1 when the core is halted. */
  DBGMCU->APB2FZ |= DBGMCU_APB1_FZ_DBG_TIM1_STOP;
#endif

  TIMER_DEV->CR1   = 0;
  TIMER_DEV->CR2   = 0;
  TIMER_DEV->SMCR  = 0;
  TIMER_DEV->DIER  = TIM_DIER_UIE | DIER_PPM;
  TIMER_DEV->CCER  = 0;
  TIMER_DEV->CCMR1 = CCMR1_PPM;
  TIMER_DEV->CCMR2 = CCMR2_PPM;
  TIMER_DEV->CCER  = CCER_PPM;
  TIMER_DEV->CNT   = 0;
  TIMER_DEV->PSC   = TIMER_PRESCALAR;
  TIMER_DEV->ARR   = 0xFFFF;

#if defined(TIMER_IRQ)          /* single IRQ timer */
  interrupt_set_priority(TIMER_IRQ, TIMER_IRQ_PRIORITY);
  interrupt_enable(TIMER_IRQ);
#else  /* multi-IRQ timer */
  interrupt_set_priority(TIMER_UP_IRQ, TIMER_IRQ_PRIORITY);
  interrupt_set_priority(TIMER_CC_IRQ, TIMER_IRQ_PRIORITY);
  interrupt_enable(TIMER_UP_IRQ);
  interrupt_enable(TIMER_CC_IRQ);
#endif

  TIMER_DEV->CR1 |= TIM_CR1_CEN;
}

uint64_t timer_get_ticks(void)
{
  uint64_t ticks;
  uint16_t cnt;

  /* We read "g_ticks" in a loop here to avoid running into an edge
   * case where the timer rolls over during the read.
   *
   * Without doing this, even if we disable the update interrupt, the
   * counter register will still reload and the total sum will be
   * incorrect. */
  do {
    ticks = g_ticks;
    cnt   = TIMER_DEV->CNT;
  } while (ticks != g_ticks);

  return ticks + (uint64_t)cnt;
}

void timer_usleep(uint16_t delay)
{
  portENTER_CRITICAL();

  g_delay_complete = false;
  TIMER_DEV->CCR1   = (uint16_t)(TIMER_DEV->CNT + delay);
  TIMER_DEV->DIER  |= TIM_DIER_CC1IE;
  TIMER_DEV->CCMR1 |= TIM_CCMR1_OC1M_0;

  while (!g_delay_complete)
    ;

  portEXIT_CRITICAL();
}

void timer_msleep(uint32_t delay)
{
  vTaskDelay(delay / portTICK_RATE_MS);
}

bool timer_is_ppm_valid(void)
{
  uint16_t sample_len;

  portDISABLE_INTERRUPTS();
  sample_len = g_ppm_state.sample_len;
  portENABLE_INTERRUPTS();

  return sample_len != 0;
}

bool timer_get_ppm_channel(int channel, uint16_t *result)
{
  bool ret = false;

  portDISABLE_INTERRUPTS();

  if (channel < g_ppm_state.sample_len) {
    *result = g_ppm_state.sample_buf[channel];
    ret = true;
  }

  portENABLE_INTERRUPTS();

  return ret;
}

size_t timer_get_ppm(uint16_t *buf, size_t len, timer_tick_t *time)
{
  size_t result = 0;
  size_t i;

  /*
   * We disable interrupts here so we do not capture the sample
   * buffer in an intermediate state during an update.
   *
   * XXX This might not actually disable the capture/compare interrupt
   * since the priority is above the FreeRTOS syscall priority.
   */
  portDISABLE_INTERRUPTS();

  for (i = 0; i < len && i < g_ppm_state.sample_len; ++i) {
    buf[i] = g_ppm_state.sample_buf[i];
    ++result;
  }

  if (time != NULL)
    *time = g_ppm_state.sample_time;

  portENABLE_INTERRUPTS();

  return result;
}

void timer_clear_ppm(void)
{
  portDISABLE_INTERRUPTS();
  g_ppm_state.sample_len = 0;
  portENABLE_INTERRUPTS();
}

/** Handle a timer update interrupt. */
static void timer_update_isr(void)
{
  g_ticks += 0x10000ULL;
  TIMER_DEV->SR &= ~TIM_SR_UIF;
}

/** PPM decoder state machine. */
static void timer_ppm_decode(uint16_t sr)
{
  /* Note that there is an assumption here that the timer will not
   * updated more than once in between captures or the math will be
   * incorrect (but should still work unless the timer frequency is
   * changed). */
  uint16_t count = TIMER_DEV->CCR_PPM;
  uint16_t width = count - g_ppm_state.last_edge;
  uint16_t value;

  /* Start over if we overflowed the capture. */
  if (sr & SR_CCOF_PPM)
    goto error;

  g_ppm_state.last_edge = count;

  /* If the pulse is large enough to start a new frame, push any
   * values captured the last time around to the shadow buffer and
   * reset the state machine. */
  if (width >= PPM_START_MIN) {
    if (g_ppm_state.current_len > 4) {
      for (size_t i = 0; i < g_ppm_state.current_len; ++i)
        g_ppm_state.sample_buf[i] = g_ppm_state.current_buf[i];
    }

    g_ppm_state.sample_time = timer_get_ticks();
    g_ppm_state.sample_len = g_ppm_state.current_len;
    g_ppm_state.current_len = 0;
    g_ppm_state.state = ARMING;

    return;
  }

  switch (g_ppm_state.state) {
    case SYNC:
      /* Continue waiting for a start pulse. */
      return;

    case ARMING:
      if (width > PPM_LOW_MAX)
        goto error;             /* pulse was too long */

      g_ppm_state.last_mark = count;
      g_ppm_state.state = INACTIVE;
      return;

    case INACTIVE:
      g_ppm_state.state = ACTIVE;
      return;

    case ACTIVE:
      value = count - g_ppm_state.last_mark;
      g_ppm_state.last_mark = count;

      /* If the value is invalid, abandon the frame. */
      if ((value < PPM_HIGH_MIN) || (value > PPM_HIGH_MAX))
        goto error;

      /* Store the value in the current frame buffer. */
      if (g_ppm_state.current_len < PPM_MAX_CHANNELS)
        g_ppm_state.current_buf[g_ppm_state.current_len++] = value;

      g_ppm_state.state = INACTIVE;
      return;
  }

  /* fall through if state is invalid */
error:
  g_ppm_state.state = SYNC;
  g_ppm_state.current_len = 0;
}

/** Handle a timer capture/compare interrupt for PPM decoding. */
static void timer_cc_ppm_isr(uint16_t sr)
{
  timer_ppm_decode(sr);
  TIMER_DEV->SR &= ~(SR_CCIF_PPM | SR_CCOF_PPM);
}

/** Handle an output compare for "timer_usleep". */
static void timer_cc_delay_isr(uint16_t sr)
{
  g_delay_complete = true;

  /* ACK the interrupt and disable it. */
  TIMER_DEV->SR    &= ~TIM_SR_CC1IF;
  TIMER_DEV->DIER  &= ~TIM_DIER_CC1IE;
  TIMER_DEV->CCMR1 &= ~TIM_CCMR1_OC1M_0;
}

#if defined(TIMER_ISR)
void TIMER_ISR(void)
{
  uint16_t sr = TIMER_DEV->SR;

  if (sr & TIM_SR_UIF)
    timer_update_isr();
  if (sr & TIM_SR_CC1IF)
    timer_cc_delay_isr(sr);
  if (sr & (SR_CCIF_PPM | SR_CCOF_PPM))
    timer_cc_ppm_isr(sr);
}
#else
void TIMER_UP_ISR(void)
{
  timer_update_isr();
}

void TIMER_CC_ISR(void)
{
  uint16_t sr = TIMER_DEV->SR;

  if (sr & TIM_SR_CC1IF)
    timer_cc_delay_isr(sr);
  if (sr & (SR_CCIF_PPM | SR_CCOF_PPM))
    timer_cc_ppm_isr(sr);
}
#endif
