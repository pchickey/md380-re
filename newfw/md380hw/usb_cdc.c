/* -*- Mode: C; indent-tabs-mode: nil; c-basic-offset: 2 -*- */
/*
 * usb_cdc.c --- USB serial driver.
 *
 * Copyright (C) 2012, Galois, Inc.
 * All Rights Reserved.
 *
 * This software is released under the "BSD3" license.  Read the file
 * "LICENSE" for more information.
 *
 * Written by James Bielman <jamesjb@galois.com>, 13 December 2012
 */

#include <stdarg.h>
#include <stdbool.h>
#include <stdint.h>

#include <FreeRTOS.h>

#include <hwf4/usb_cdc.h>

#include <usb/usb_conf.h>
#include <usb/usb_dcd_int.h>
#include <usb/usbd_cdc_core.h>
#include <usb/usbd_usr.h>
#include <usb/usbd_desc.h>
#include <usb/usbd_cdc_vcp.h>

/** USB device handle initialized in "usb_cdc_init". */
__ALIGN_BEGIN USB_OTG_CORE_HANDLE g_usb_dev __ALIGN_END;

/** Interrupt handler for the USB stack. */
void OTG_FS_IRQHandler(void)
{
  USBD_OTG_ISR_Handler(&g_usb_dev);
}

/*
 * {Public Interface}
 */

void usb_cdc_init(void)
{
  VCP_Init();
  USBD_Init(&g_usb_dev, USB_OTG_FS_CORE_ID, &USR_desc,
            &USBD_CDC_cb, &USR_cb);
}

ssize_t usb_cdc_write(const void *buf, size_t len)
{
  return VCP_Write((uint8_t *)buf, len);
}

ssize_t usb_cdc_read_timeout(void *buf, size_t len, uint32_t timeout)
{
  return VCP_Read((uint8_t *)buf, len, timeout);
}
