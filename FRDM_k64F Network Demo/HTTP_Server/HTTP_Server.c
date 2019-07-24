/*------------------------------------------------------------------------------
 * MDK Middleware - Component ::Network
 * Copyright (c) 2004-2014 ARM Germany GmbH. All rights reserved.
 *------------------------------------------------------------------------------
 * Name:    HTTP_Server.c
 * Purpose: HTTP Server example
 *----------------------------------------------------------------------------*/

/*-----------------------------------------------------------------------------
      Modified by David Khosravi (davidkhosravi@yahoo.se)
      Date:     2019-07-22
  
 -----------------------------------------------------------------------------*/
 
#include <stdio.h>
#include "cmsis_os.h"                   /* CMSIS RTOS definitions             */
#include "rl_net.h"                     /* Network definitions                */
#include "fsl_sysmpu.h"
#include "Board_LED.h"
#include "Board_Buttons.h"
#include "EventRecorder.h"              /* Keil::Compiler:Event Messaging     */
#include "board.h"                      /* BSP definitions                    */
#include "pin_mux.h"

bool LEDrun;
bool MSGupdate;

char lcd_text[2][20+1] = { "LCD line 1",
                           "LCD line 2" };
// Thread IDs
osThreadId TID_Message;
osThreadId TID_Led;                           
                           
// Thread definitions
static void BlinkLed (void const *arg);
static void Message  (void const *arg);

osThreadDef(BlinkLed, osPriorityNormal, 1, 0);
osThreadDef(Message,  osPriorityNormal, 1, 0);


uint32_t ENET0_GetFreq(void)
{
	printf("\nGetFreq ETHERNET IEEE 1588 clock 50 MHZ\n");
  return CLOCK_GetFreq(kCLOCK_Osc0ErClk);
}

void ENET0_InitPins(void)
{
	printf("\nInitENETPins\n");
  BOARD_InitENETPins();
}

void ENET0_DeinitPins(void)
{
	printf("\ndeinitpins\n");
}

/// Read analog inputs
uint16_t AD_in (uint32_t ch) {
  /* User analog input is not available */
  static uint32_t counter =0;
  counter+=10;
  if(counter > 4095)
    counter = 0;
  return (counter);
}

/// Read digital inputs
uint8_t get_button (void) {
  /* User button is not available */
 
  return (Buttons_GetState ());
}


// IP address change notification
void netDHCP_Notify (uint32_t if_num, uint8_t option, const uint8_t *val, uint32_t len) {

  if (option == NET_DHCP_OPTION_IP_ADDRESS) {
    /* IP address change, trigger LCD update */
    osSignalSet (TID_Message, 0x01);
  }
}




/*----------------------------------------------------------------------------
  Thread 'Message': printf message handler
 *---------------------------------------------------------------------------*/
static void Message (void const *arg) {
  static uint8_t ip_addr[NET_ADDR_IP6_LEN];
  static char    ip_ascii[40];
  static char    buf[24];
  printf ("       MDK-MW       \n");
  printf (" HTTP Server example\n\n");

  printf ("Waiting for DHCP ...\n");
  fflush (stdout);
  MSGupdate = false;

  // Print Link-local IPv6 address
  netIF_GetOption (NET_IF_CLASS_ETH,
                   netIF_OptionIP6_LinkLocalAddress, ip_addr, sizeof(ip_addr));

  netIP_ntoa(NET_ADDR_IP6, ip_addr, ip_ascii, sizeof(ip_ascii));

  printf ("%s\n",buf);
  sprintf (buf, "%s", ip_ascii+16);
  printf ("%s\n",buf);
  
  
  
  while(1) {
   /* Wait for signal from DHCP */
    osSignalWait (0x01, osWaitForever);
     /* Retrieve and print IPv4 address */
  netIF_GetOption (NET_IF_CLASS_ETH,netIF_OptionIP4_Address, ip_addr, sizeof(ip_addr));
  netIP_ntoa(NET_ADDR_IP6, ip_addr, ip_ascii, sizeof(ip_ascii));
  sprintf (buf, "IP6:%.16s", ip_ascii);

  printf ("%s\n",buf);
  sprintf (buf, "%s", ip_ascii+16);
  printf ("%s\n",buf);
    
    if (MSGupdate == true) {
      printf ("%s\n", lcd_text[0]);
      MSGupdate = false;
    }
  //  osDelay (250);
  }
}

/*----------------------------------------------------------------------------
  Thread 'BlinkLed': Blink the LEDs on an eval board
 *---------------------------------------------------------------------------*/
static void BlinkLed (void const *arg) {
  
  const uint8_t led_val[] = { 1, 2, 4 };
  int cnt = 0;

  LEDrun = true;
  while(1) {

    // Every 500 ms
    if (LEDrun == true) {
      LED_SetOut (led_val[cnt]);
      if (++cnt >= sizeof(led_val)) {
        cnt = 0;
      }
    }
    osDelay (500);
  }
}

/*----------------------------------------------------------------------------
  Main Thread 'main': Run Network
 *---------------------------------------------------------------------------*/
 int main (void) {

   BOARD_InitPins();
   BOARD_BootClockRUN();
   BOARD_InitENETPins();
   BOARD_InitLEDsPins();
   BOARD_InitButtonsPins();
   //BOARD_InitDebugConsole();
   SYSMPU_Enable(SYSMPU, false);
   EventRecorderInitialize (EventRecordAll, 1); // initialize and start Event Recorder
   netInitialize();
   
   TID_Led = osThreadCreate (osThread(BlinkLed), NULL);
   TID_Message = osThreadCreate (osThread(Message),  NULL);

  while(1) {
    osSignalWait (0, osWaitForever);
  }
}
