/*-----------------------------------------------------------------------------
 * Name:    LED_FRDM_K64F.c
 * Purpose: LED interface for Freescale FRDM-K64F development board
 * Rev.:    1.2
 * Modified:David Khosravi
 * Mod Rev: 1.0
 *----------------------------------------------------------------------------*/
/*-----------------------------------------------------------------------------
      Modified by David Khosravi (davidkhosravi@yahoo.se)
      Date:     2019-07-22
  
 -----------------------------------------------------------------------------*/
 
/* Copyright (c) 2013 - 2014 ARM LIMITED

   All rights reserved.
   Redistribution and use in source and binary forms, with or without
   modification, are permitted provided that the following conditions are met:
   - Redistributions of source code must retain the above copyright
     notice, this list of conditions and the following disclaimer.
   - Redistributions in binary form must reproduce the above copyright
     notice, this list of conditions and the following disclaimer in the
     documentation and/or other materials provided with the distribution.
   - Neither the name of ARM nor the names of its contributors may be used
     to endorse or promote products derived from this software without
     specific prior written permission.
   *
   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
   AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
   IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
   ARE DISCLAIMED. IN NO EVENT SHALL COPYRIGHT HOLDERS AND CONTRIBUTORS BE
   LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
   CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
   SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
   INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
   CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
   ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
   POSSIBILITY OF SUCH DAMAGE.
   ---------------------------------------------------------------------------*/

#include "board.h"
#include "pin_mux.h"
#include "Board_LED.h"

/// GPIO Pin identifier
typedef struct _GPIO_PIN_ID {
  GPIO_Type *port;
  uint8_t       num;
} GPIO_PIN_ID;
   
    
/* Board has a 3-color LED that maps as follows:
     LED(0): Green
     LED(1): Red
     LED(2): Blue
*/
const GPIO_PIN_ID Pin_LED[] = {
  { GPIOE,  26 },
  { GPIOB,  22 },
  { GPIOB,  21 },
  
};

#define LED_COUNT (sizeof(Pin_LED)/sizeof(GPIO_PIN_ID))


/**
  \fn          int32_t LED_Initialize (void)
  \brief       Initialize LEDs
  \returns
   - \b  0: function succeeded
   - \b -1: function failed
*/
int32_t LED_Initialize (void) {

  BOARD_InitLEDsPins();
  return 0;
}

/**
  \fn          int32_t LED_Uninitialize (void)
  \brief       De-initialize LEDs
  \returns
   - \b  0: function succeeded
   - \b -1: function failed
*/
int32_t LED_Uninitialize (void) {
  return 0;
}

/**
  \fn          int32_t LED_On (uint32_t num)
  \brief       Turn on requested LED
  \param[in]   num  LED number
  \returns
   - \b  0: function succeeded
   - \b -1: function failed
*/
int32_t LED_On (uint32_t num) {

   int32_t retCode = 0;
  assert(num < LED_COUNT);
  if (num < LED_COUNT) {
    GPIO_PinWrite(Pin_LED[num].port, Pin_LED[num].num, 0);
  }
  else {
    retCode = -1;
  }

  return retCode;
}

/**
  \fn          int32_t LED_Off (uint32_t num)
  \brief       Turn off requested LED
  \param[in]   num  LED number
  \returns
   - \b  0: function succeeded
   - \b -1: function failed
*/
int32_t LED_Off (uint32_t num) {

  int32_t retCode = 0;
  assert(num < LED_COUNT);
  if (num < LED_COUNT) {
    GPIO_PinWrite(Pin_LED[num].port, Pin_LED[num].num, 1);
  }
  else {
    retCode = -1;
  }

  return retCode;
}

/**
  \fn          int32_t LED_SetOut (uint32_t val)
  \brief       Write value to LEDs
  \param[in]   val  value to be displayed on LEDs
  \returns
   - \b  0: function succeeded
   - \b -1: function failed
*/
int32_t LED_SetOut (uint32_t val) {
  uint32_t n;

  for (n = 0; n < LED_COUNT; n++) {
    if (val & (1 << n)) LED_On (n);
    else                LED_Off(n);
  }
  return 0;
}

/**
  \fn          uint32_t LED_GetCount (void)
  \brief       Get number of LEDs
  \return      Number of available LEDs
*/
uint32_t LED_GetCount (void) {
  return LED_COUNT;
}
