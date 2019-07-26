/* -----------------------------------------------------------------------------
 * Copyright (c) 2013-2014 ARM Ltd.
 *
 * This software is provided 'as-is', without any express or implied warranty.
 * In no event will the authors be held liable for any damages arising from
 * the use of this software. Permission is granted to anyone to use this
 * software for any purpose, including commercial applications, and to alter
 * it and redistribute it freely, subject to the following restrictions:
 *
 * 1. The origin of this software must not be misrepresented; you must not
 *    claim that you wrote the original software. If you use this software in
 *    a product, an acknowledgment in the product documentation would be
 *    appreciated but is not required.
 *
 * 2. Altered source versions must be plainly marked as such, and must not be
 *    misrepresented as being the original software.
 *
 * 3. This notice may not be removed or altered from any source distribution.
 *
 *
 * $Date:        11. December 2014
 * $Revision:    V1.0
 *
 * Project:      OTG Full/Low-Speed Common Driver for Freescale MK64
 * Configured:   via Processor Expert
 * -------------------------------------------------------------------------- */

/* History:
 *  Version 1.0
 *    Initial release
 */

/* Processor Expert configuration:
 *
 * For VBUS Power driving Pin:
 *   - define output GPIO pin in Processor Expert and name it kGpioUsbhVbus
 *   - define GpioUsbhVbusPin definition for this module
 *   - define GpioUsbhVbusPinActive for this module with value 1 if pin is active High
 *
 * For Overcurrent state Pin:
 *   - define input GPIO pin in Processor Expert and name it kGpioUsbhOc
 *   - define GpioUsbhOcPin definition for this module
 *   - define GpioUsbhOcPinActive for this module with value 1 if pin is active High
 */

 
#include <stdint.h>

#if   (defined(GpioUsbhVbusPin) || defined(GpioUsbhOcPin))
#include "fsl_gpio_driver.h"
#endif

#include "Driver_USBH.h"
#include "Driver_USBD.h"

#include "RTE_Components.h"

/*******************************************************************************
 * Adaption for custom USBD driver.
 ******************************************************************************/
#define RTE_Drivers_USBD0

/* Define USB Host VBUS Power Pin active state */
#ifndef GpioUsbhVbusPinActive
#define GpioUsbhVbusPinActive           0
#endif

/* Define USB Host Overcurrent Pin active state */
#ifndef GpioUsbhOcPinActive
#define GpioUsbhOcPinActive             0
#endif


#ifdef RTE_Drivers_USBH0
extern void USBH_FS_IRQ (void);
#endif
#ifdef RTE_Drivers_USBD0
extern void USBD_FS_IRQ (void);
#endif

uint8_t otg_fs_role   = ARM_USB_ROLE_NONE;
uint8_t otg_fs_state  = 0U;


/* Common IRQ Routine *********************************************************/

/**
  \fn          void USB0_IRQHandler (void)
  \brief       USB Interrupt Routine (IRQ).
*/
void USB0_IRQHandler (void) {

#if (defined(RTE_Drivers_USBH0) && defined(RTE_Drivers_USBD0))
  switch (otg_fs_role) {
#ifdef RTE_Drivers_USBH0
    case ARM_USB_ROLE_HOST:
      USBH_FS_IRQ ();
      break;
#endif
#ifdef RTE_Drivers_USBD0
    case ARM_USB_ROLE_DEVICE:
      USBD_FS_IRQ ();
      break;
#endif
  }
#else
#ifdef RTE_Drivers_USBH0
  USBH_FS_IRQ ();
#else
  USBD_FS_IRQ ();
#endif
#endif
}


/* Public Functions ***********************************************************/

/**
  \fn          void OTG_FS_PinVbusOnOff (bool state)
  \brief       Drive VBUS Pin On/Off.
  \param[in]   state    State On/Off (true = On, false = Off)
*/
void OTG_FS_PinVbusOnOff (bool state) {

  // VBUS power drive
  // On FRDM-K64F board VBUS is always powered and not under software control
  // To enable software control of VBUS, GPIO pin has to be used in output mode
#ifdef GpioUsbhVbusPin
  GPIO_DRV_WritePinOutput(kGpioUsbhVbus, (GpioUsbhVbusPinActive ^ state) == 0U);
#endif
}

/**
  \fn          bool OTG_FS_PinGetOC (void)
  \brief       Get state of Overcurrent Pin.
  \return      overcurrent state (true = Overcurrent active, false = No overcurrent)
*/
bool OTG_FS_PinGetOC (void) {

  // Overcurrent state read
  // This chip's USB Controller does not offer any overcurrent functionality
  // To use overcurrent functionality, GPIO pin has to be used in input mode
  // and to provide overcurrent event GPIO pin with interrupt capability has
  // to be used and interrupt routine implemented
#ifdef GpioUsbhOcPin
  return (((GPIO_DRV_ReadPinInput(kGpioUsbhOc) == 0U) ^ GpioUsbhOcPinActive));
#else
  return false;
#endif
}
