/**
  ******************************************************************************
  * @file    usb_bsp.c
  * @author  MCD Application Team
  * @version V2.1.0
  * @date    19-March-2012
  * @brief   This file is responsible to offer board support package and is
  *          configurable by user.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT 2012 STMicroelectronics</center></h2>
  *
  * Licensed under MCD-ST Liberty SW License Agreement V2, (the "License");
  * You may not use this file except in compliance with the License.
  * You may obtain a copy of the License at:
  *
  *        http://www.st.com/software_license_agreement_liberty_v2
  *
  * Unless required by applicable law or agreed to in writing, software 
  * distributed under the License is distributed on an "AS IS" BASIS, 
  * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  * See the License for the specific language governing permissions and
  * limitations under the License.
  *
  ******************************************************************************
  */ 

/* Includes ------------------------------------------------------------------*/
#include "usb_bsp.h"

#include "rcc.h"
#include "gpio.h"
#include "interrupt.h"
#include "FreeRTOS.h"
#include "task.h"

/** USB pin definitions for the STM32F4. */
#define USB_VBUS_PIN pin_a9
#define USB_DM_PIN   pin_a11
#define USB_DP_PIN   pin_a12

/**
 * @brief  USB_OTG_BSP_Init
 *         Initilizes BSP configurations
 * @param  None
 * @retval None
 */
void USB_OTG_BSP_Init(USB_OTG_CORE_HANDLE *usb_dev)
{
  pin_enable(USB_VBUS_PIN);
  pin_set_mode(USB_VBUS_PIN, PIN_MODE_INPUT);
  pin_set_pupd(USB_VBUS_PIN, PIN_PUPD_NONE);

  pin_enable(USB_DM_PIN);
  pin_set_af(USB_DM_PIN, PIN_AF_OTG_FS);
  pin_set_ospeed(USB_DM_PIN, PIN_SPEED_100MHZ);
  pin_set_otype(USB_DM_PIN, PIN_TYPE_PUSHPULL);
  pin_set_pupd(USB_DM_PIN, PIN_PUPD_NONE);
  pin_set_mode(USB_DM_PIN, PIN_MODE_AF);

  pin_enable(USB_DP_PIN);
  pin_set_af(USB_DP_PIN, PIN_AF_OTG_FS);
  pin_set_ospeed(USB_DP_PIN, PIN_SPEED_100MHZ);
  pin_set_otype(USB_DP_PIN, PIN_TYPE_PUSHPULL);
  pin_set_pupd(USB_DP_PIN, PIN_PUPD_NONE);
  pin_set_mode(USB_DP_PIN, PIN_MODE_AF);

  rcc_enable(RCCDEV_SYSCFG);
  rcc_enable(RCCDEV_OTGFS);

  interrupt_set_priority(OTG_FS_IRQn, INTERRUPT_PRIORITY_FREERTOS_SAFE);
}

/**
 * @brief  USB_OTG_BSP_EnableInterrupt
 *         Enabele USB Global interrupt
 * @param  None
 * @retval None
 */
void USB_OTG_BSP_EnableInterrupt(USB_OTG_CORE_HANDLE *pdev)
{
  interrupt_enable(OTG_FS_IRQn);
}

/**
 * @brief  USB_OTG_BSP_uDelay
 *         This function provides delay time in micro sec
 * @param  usec : Value of delay required in micro sec
 * @retval None
 */
void USB_OTG_BSP_uDelay(uint32_t usec)
{

	while (usec > 1000) {
		USB_OTG_BSP_mDelay(1);
		usec -= 1000;
	}

	const uint32_t bound = usec * 168;
	for (uint32_t j = 0; j < bound; j+= 4) {
		asm volatile ("mov r0,r0");
	}

}

/**
 * @brief  USB_OTG_BSP_mDelay
 *          This function provides delay time in milli sec
 * @param  msec : Value of delay required in milli sec
 * @retval None
 */
void USB_OTG_BSP_mDelay(const uint32_t msec)
{
	vTaskDelay(msec);
}

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
