/*
 * Copyright (c) 2013 - 2015, Freescale Semiconductor, Inc.
 * Copyright 2016-2017 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */
#include <stdio.h>
#include "board.h"
#include "pin_mux.h"
#include "clock_config.h"
#include "fsl_device_registers.h"       // Device header
#include "fsl_debug_console.h"          // NXP::Device:SDK Utilities:debug_console
#include "fsl_sysmpu.h"
#include "cmsis_os.h"                   // ARM::CMSIS:RTOS:Keil RTX
#include "rl_usb.h"                     // Keil.MDK-Plus::USB:CORE
#include "EventRecorder.h"              // Keil.ARM Compiler::Compiler:Event Recorder


/*******************************************************************************
 * Definitions
 ******************************************************************************/


/*******************************************************************************
 * Prototypes
 ******************************************************************************/
 

/*******************************************************************************
 * Code
 ******************************************************************************/
// Thread IDs
osThreadId TID_Message;

// Thread definitions
static void Message  (void const *arg);
osThreadDef(Message,  osPriorityNormal, 1, 0);


static void Message (void const *arg) {

  static uint16_t counter = 0;
  static char    buf[24];
	while (1)
	{
		LED_GREEN_TOGGLE();
    counter++;
    sprintf (buf, "Message number: %d\t\r\n", counter);
    USBD_CDC_ACM_WriteData(0, (uint8_t *)buf, sizeof(buf));
		osDelay(1000);
	}
}

int main(void)
{
    BOARD_InitPins();
    BOARD_InitLEDsPins();
    BOARD_BootClockRUN();
    BOARD_InitDebugConsole();
    SYSMPU_Enable(SYSMPU, 0);
    EventRecorderInitialize (EventRecordAll, 1); // initialize and start Event Recorder

	
    PRINTF("\r\nStart\r\n");
    bool ret = USBD_Initialize(0);
    assert(ret == usbOK);
    ret = USBD_Connect(0);
    assert(ret == usbOK);
    //assert(USBD_Initialize(0) == usbOK);
    //assert(USBD_Connect(0) == usbOK);
    osKernelInitialize();

    TID_Message = osThreadCreate (osThread(Message),  NULL);

    osKernelStart();
	
    while (1)
    {
      osSignalWait(0U, osWaitForever);
    }
}
