/*-----------------------------------------------------------------------------
 * Name:    Buttons_FRDM_K64F.c
 * Purpose: Buttons interface for Freescale FRDM-K64F development board
 * Rev.:    1.00
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
#include "Board_Buttons.h"

#define BUTTONS_COUNT                  (2U)

/// GPIO Pin identifier
typedef struct _GPIO_PIN_ID {
  GPIO_Type *port;
  uint8_t       num;
} GPIO_PIN_ID;

/* Boards Two Switch maped as follows:
     Pin_SW[0]: SW2
     Pin_SW[1]: SW3
*/
const GPIO_PIN_ID Pin_SW[] = {
  { GPIOC,  6 }, 
  { GPIOA,  4 },
  
};

/**
  \fn          int32_t Buttons_Initialize (void)
  \brief       Initialize buttons
  \returns
   - \b  0: function succeeded
   - \b -1: function failed
*/
int32_t Buttons_Initialize (void) {
  //GPIO_DRV_Init(switchPins, NULL);
  BOARD_InitButtonsPins();
  return 0;
}

/**
  \fn          int32_t Buttons_Uninitialize (void)
  \brief       De-initialize buttons
  \returns
   - \b  0: function succeeded
   - \b -1: function failed
*/
int32_t Buttons_Uninitialize (void) {
  return 0;
}

/**
  \fn          uint32_t Buttons_GetState (void)
  \brief       Get buttons state
  \returns     Buttons state
*/
uint32_t Buttons_GetState (void) {
  uint32_t val = 0;
  uint32_t n;

  for (n = 0; n < BUTTONS_COUNT; n++) {
//    if (GPIO_DRV_ReadPinInput(switchPins[n].pinName) == 0) {
//      val |= 1U << n;
//    }
    if (GPIO_PinRead(Pin_SW[n].port, Pin_SW[n].num) == 0) {
      val |= 1U << n;
    }
  }

  return val;
  
}

/**
  \fn          uint32_t Buttons_GetCount (void)
  \brief       Get number of available buttons
  \return      Number of available buttons
*/
uint32_t Buttons_GetCount (void) {
  return BUTTONS_COUNT;
}
