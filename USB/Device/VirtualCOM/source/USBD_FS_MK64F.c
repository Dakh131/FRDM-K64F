/* -----------------------------------------------------------------------------
 * Copyright (c) 2013-2017 ARM Ltd.
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
 * $Date:        19. September 2017
 * $Revision:    V1.2
 *
 * Driver:       Driver_USBD0
 * Project:      USB Full/Low-Speed Device Driver for Freescale MK64F
 * --------------------------------------------------------------------------
 * Use the following configuration settings in the middleware component
 * to connect to this driver.
 *
 *   Configuration Setting                  Value
 *   ---------------------                  -----
 *   Connect to hardware via Driver_USBD# = 0
 * --------------------------------------------------------------------------
 * Defines used for driver configuration (at compile time):
 *
 *   USBD_MAX_ENDPOINT_NUM:  defines maximum number of IN/OUT Endpoint pairs 
 *                           that driver will support with Control Endpoint 0
 *                           not included, this value impacts driver memory
 *                           requirements
 *     - default value: 15
 *     - maximum value: 15
 *
 *  USBD_EP_MAX_PACKET_SIZE: defines maximum packet size (in bytes) for any
 *                           Endpoint (reducing this value can save RAM used
 *                           for Endpoint data buffers, but it must be at least
 *                           as big as largest maximum packet size of Endpoint)
 *     - default value: 64
 * -------------------------------------------------------------------------- */

/* History:
 *  Version 1.2
 *    Added support for CMSIS-RTOS2
 *  Version 1.1
 *    Updated conditionality of functions
 *  Version 1.0
 *    Initial release for USB Device CMSIS Driver API v2.01, with double buffer
 *    support
 */


#include <stdint.h>
#include <string.h>

#include "RTE_Components.h"

#if       defined(RTE_CMSIS_RTOS2)
#include "cmsis_os2.h"
#elif     defined(RTE_CMSIS_RTOS)
#include "cmsis_os.h"
#endif

#include "Driver_USBD.h"

#include "fsl_device_registers.h"

#ifndef USBD_MAX_ENDPOINT_NUM
#define USBD_MAX_ENDPOINT_NUM           15U
#endif
#if    (USBD_MAX_ENDPOINT_NUM > 15)
#error  Too many Endpoints, maximum IN/OUT Endpoint pairs that this driver supports is 15 !!!
#endif

#ifndef USBD_EP_MAX_PACKET_SIZE
#define USBD_EP_MAX_PACKET_SIZE         64U
#endif

extern uint8_t otg_fs_role;
extern uint8_t otg_fs_state;


// USBD Driver *****************************************************************

#define ARM_USBD_DRV_VERSION ARM_DRIVER_VERSION_MAJOR_MINOR(1,2)

// Driver Version
static const ARM_DRIVER_VERSION usbd_driver_version = { ARM_USBD_API_VERSION, ARM_USBD_DRV_VERSION };

// Driver Capabilities
static const ARM_USBD_CAPABILITIES usbd_driver_capabilities = {
  0U,   // VBUS Detection
  0U,   // Event VBUS On
  0U    // Event VBUS Off
};

#define OTG_FS_USBD_DRIVER_INITIALIZED  (1U     )
#define OTG_FS_USBD_DRIVER_POWERED      (1U << 1)

#define EP_NUM(ep_addr)                 (ep_addr & ARM_USB_ENDPOINT_NUMBER_MASK)
#define EP_ID(ep_addr)                  ((EP_NUM(ep_addr) * 2U) + ((ep_addr >> 7U) & 1U))

#define BD_OWN_MASK                     (1U << 7)
#define BD_DATA01_MASK                  (1U << 6)
#define BD_KEEP_MASK                    (1U << 5)
#define BD_NINC_MASK                    (1U << 4)
#define BD_DTS_MASK                     (1U << 3)
#define BD_BDT_STALL_MASK               (1U << 2)
#define BD_TOK_PID_MASK                 (0x0FU << 2)
#define BD_TOK_PID(idx)                 ((BD[idx].stat & BD_TOK_PID_MASK) >> 2)
#define BD_IDX(endp, tx, odd)           (((endp & 0x0FU)  * 4U) + (tx * 2U) + odd)
#define BD_ID(ep_addr)                  (((EP_NUM(ep_addr) * 4U) + (((ep_addr >> 7) & 1U) * 2U)))
#define EP_ID_FROM_NUM(endp,tx)         ((endp * 2U) +  tx)
#define BD_ID_FROM_NUM(endp,tx,odd)     ((endp * 4U) + (tx * 2U) + odd)

#define TOK_PID_SETUP                   (0x0DU)
#define TOK_PID_IN                      (0x09U)
#define TOK_PID_OUT                     (0x01U)

typedef struct {                        // Buffer Descriptor table entry structure definition
  uint32_t  cmd_stat;
  uint32_t  buf_addr;
} BD_t;

typedef struct {                        // Endpoint structure definition
  uint8_t  *data;
  uint32_t  num;
  uint32_t  num_transferred_total;
  uint16_t  num_transferring[2];
  uint8_t   next_data_toggle;
  uint8_t   last_bd_odd;
  uint8_t   next_bd_odd;
  uint8_t   active;
  uint16_t  ep_max_packet_size;
} ENDPOINT_t;

static ARM_USBD_SignalDeviceEvent_t   SignalDeviceEvent;
static ARM_USBD_SignalEndpointEvent_t SignalEndpointEvent;

static ARM_USBD_STATE      usbd_state;

static uint32_t            setup_packet[2];     // Setup packet data
static volatile uint8_t    setup_received;      // Setup packet received

// Buffer Descriptor Table
//static BD_t __align(512)   bd          [(USBD_MAX_ENDPOINT_NUM + 1U) * 2U * 2U];
static BD_t __attribute__((aligned(512)))    bd          [(USBD_MAX_ENDPOINT_NUM + 1U) * 2U * 2U];

// Endpoints IN data buffers
static uint32_t            ep_in_data  [(USBD_MAX_ENDPOINT_NUM + 1U)][2U][USBD_EP_MAX_PACKET_SIZE / 4U];

// Endpoints runtime information
static volatile ENDPOINT_t ep          [(USBD_MAX_ENDPOINT_NUM + 1U) * 2U];


// Auxiliary functions

/**
  \fn          void USBD_Reset (void)
  \brief       Reset USB Endpoint settings and variables.
*/
static void USBD_Reset (void) {
  volatile ENDPOINT_t *ptr_ep;
  volatile BD_t       *ptr_bd;
  uint8_t              i;

  // Reset global variables
  setup_packet[0] = 0U;
  setup_packet[1] = 0U;
  setup_received  = 0U;
  memset((void *)&usbd_state, 0, sizeof(usbd_state));
  memset((void *)bd,          0, sizeof(bd));
  memset((void *)ep_in_data,  0, sizeof(ep_in_data));
  memset((void *)ep,          0, sizeof(ep));

  // Disable all Endpoints and initialize all starting odd bits
  ptr_ep = ep;
  for (i = 0U; i < USBD_MAX_ENDPOINT_NUM; i++) {
    USB0->ENDPOINT[i].ENDPT = 0U;
    ptr_ep->last_bd_odd = 1U; ptr_ep++;
    ptr_ep->last_bd_odd = 1U; ptr_ep++;
  }

  // Set BDT address
  USB0->BDTPAGE1      = ((uint32_t)(bd) >>  8) & 0xFEU;
  USB0->BDTPAGE2      = ((uint32_t)(bd) >> 16);
  USB0->BDTPAGE3      = ((uint32_t)(bd) >> 24);

  // Initialize Endpoints IN buffer and maximum packet sizes
  ptr_ep  = ep;
  ptr_ep->next_bd_odd = 1U;                             // Set next_bd_odd to odd for next transfer
  ptr_bd  = bd;                                         // Setup Endpoint 0 OUT EVEN Buffer Descriptor Table (BDT) entry
  ptr_bd->buf_addr    = (uint32_t)setup_packet;
  ptr_bd->cmd_stat    = (8U << 16) | BD_OWN_MASK | BD_DTS_MASK | BD_DATA01_MASK;
  ptr_bd += 2U;                                         // Skip Endpoint 0 OUT ODD BDT entry
                                                        // Set all initial Endpoints IN data sizes and buffer addresses
  for (i = 0U; i < USBD_MAX_ENDPOINT_NUM; i++) {
                                                        // Endpoint IN EVEN BDT entry
    ptr_bd->buf_addr  = (uint32_t)(&ep_in_data[i][0][0]);
    ptr_bd->cmd_stat |= (USBD_EP_MAX_PACKET_SIZE << 16);
    ptr_bd++;
                                                        // Endpoint IN ODD  BDT entry
    ptr_bd->buf_addr  = (uint32_t)(&ep_in_data[i][1][0]);
    ptr_bd->cmd_stat |= (USBD_EP_MAX_PACKET_SIZE << 16);

    ptr_bd           += 3U;                             // Skip Endpoint OUT BDT entries
    // OUT Endpoints directly use provided RAM buffers except initial OUT for setup packet
  }

  USB0->ENDPOINT[0].ENDPT = USB_ENDPT_EPHSHK_MASK |     // Enable Endpoint handshaking
                            USB_ENDPT_EPRXEN_MASK ;     // Enable RX (OUT or SETUP) transfer

  USB0->CTL |=  USB_CTL_ODDRST_MASK;    // Reset buffer descriptor table entry to EVEN
  USB0->CTL &= ~USB_CTL_ODDRST_MASK;    // Clear buffer descriptor table entry reset

  USB0->ADDR =  0U;                     // Clear ADDR
}

/**
  \fn          void USBD_HW_EndpointTransfer (uint8_t endp,
                                              uint8_t tx)
  \brief       Start transfer on Endpoint.
  \param[in]   endp:    Endpoint Number
  \param[in]   tx       Transmit or receive direction
                - value 0: Receive
                - value 1: Transmit
*/
static void USBD_HW_EndpointTransfer (uint8_t endp, uint8_t tx) {
  volatile ENDPOINT_t *ptr_ep;
  volatile BD_t       *ptr_bd;
  uint8_t             *data_addr;
  uint32_t             data_num;
  uint16_t             num_to_transfer;
  uint8_t              next_bd_odd, next_data_toggle;

  ptr_ep           = &ep[EP_ID_FROM_NUM(endp,tx)];
  next_bd_odd      =  ptr_ep->next_bd_odd;
  ptr_bd           = &bd[BD_ID_FROM_NUM(endp,tx,next_bd_odd)]; if ((ptr_bd->cmd_stat & BD_OWN_MASK) != 0U) { return; }
  data_addr        = (uint8_t *)(ptr_ep->data + ptr_ep->num_transferred_total + ptr_ep->num_transferring[next_bd_odd == 0U]);
  data_num         =  ptr_ep->num - ptr_ep->num_transferred_total - ptr_ep->num_transferring[next_bd_odd == 0U];
  next_data_toggle =  ptr_ep->next_data_toggle;

  if (data_num > ptr_ep->ep_max_packet_size) {
    num_to_transfer = ptr_ep->ep_max_packet_size;
  } else {
    num_to_transfer = data_num;
  }
  ptr_ep->num_transferring[next_bd_odd] = num_to_transfer;
  ptr_ep->next_data_toggle ^= 1U;
  ptr_ep->next_bd_odd      ^= 1U;

  if (tx != 0U) {                       // Endpoint IN
                                        // Copy data to RAM buffer to be sent
    memcpy(&ep_in_data[endp][next_bd_odd][0], data_addr, num_to_transfer);
  } else {                              // Endpoint OUT
                                        // Set address where data should be received
    ptr_bd->buf_addr = (uint32_t)data_addr;
  }
  ptr_bd->cmd_stat   = (num_to_transfer << 16) | BD_OWN_MASK | BD_DTS_MASK | (BD_DATA01_MASK * next_data_toggle);
}


// USBD Driver functions

/**
  \fn          ARM_DRIVER_VERSION USBD_GetVersion (void)
  \brief       Get driver version.
  \return      \ref ARM_DRIVER_VERSION
*/
static ARM_DRIVER_VERSION USBD_GetVersion (void) { return usbd_driver_version; }

/**
  \fn          ARM_USBD_CAPABILITIES USBD_GetCapabilities (void)
  \brief       Get driver capabilities.
  \return      \ref ARM_USBD_CAPABILITIES
*/
static ARM_USBD_CAPABILITIES USBD_GetCapabilities (void) { return usbd_driver_capabilities; }

/**
  \fn          int32_t USBD_Initialize (ARM_USBD_SignalDeviceEvent_t   cb_device_event,
                                        ARM_USBD_SignalEndpointEvent_t cb_endpoint_event)
  \brief       Initialize USB Device Interface.
  \param[in]   cb_device_event    Pointer to \ref ARM_USBD_SignalDeviceEvent
  \param[in]   cb_endpoint_event  Pointer to \ref ARM_USBD_SignalEndpointEvent
  \return      \ref execution_status
*/
static int32_t USBD_Initialize (ARM_USBD_SignalDeviceEvent_t   cb_device_event,
                                ARM_USBD_SignalEndpointEvent_t cb_endpoint_event) {

  if ((otg_fs_state & OTG_FS_USBD_DRIVER_INITIALIZED) != 0U) { return ARM_DRIVER_OK; }

  SignalDeviceEvent   = cb_device_event;
  SignalEndpointEvent = cb_endpoint_event;

  otg_fs_role   =  ARM_USB_ROLE_DEVICE;
  otg_fs_state  =  OTG_FS_USBD_DRIVER_INITIALIZED;

  return ARM_DRIVER_OK;
}

/**
  \fn          int32_t USBD_Uninitialize (void)
  \brief       De-initialize USB Device Interface.
  \return      \ref execution_status
*/
static int32_t USBD_Uninitialize (void) {

  otg_fs_role   =  ARM_USB_ROLE_NONE;
  otg_fs_state &= ~OTG_FS_USBD_DRIVER_INITIALIZED;

  return ARM_DRIVER_OK;
}

/**
  \fn          int32_t USBD_PowerControl (ARM_POWER_STATE state)
  \brief       Control USB Device Interface Power.
  \param[in]   state  Power state
  \return      \ref execution_status
*/
static int32_t USBD_PowerControl (ARM_POWER_STATE state) {

  switch (state) {
    case ARM_POWER_OFF:
      NVIC_DisableIRQ      (USB0_IRQn);                 // Disable interrupt
      NVIC_ClearPendingIRQ (USB0_IRQn);                 // Clear pending interrupt
      otg_fs_state  &= ~OTG_FS_USBD_DRIVER_POWERED;     // Clear powered flag
      USB0->INTEN    =  0U;                             // Disable USB interrupts
      USB0->USBTRC0 |=  USB_USBTRC0_USBRESET_MASK;      // Reset OTG FS module
      __nop();__nop();__nop();__nop();__nop();__nop();  // Wait for reset to finish
                                                        // Reset variables
      setup_received =  0U;
      memset((void *)&usbd_state, 0, sizeof(usbd_state));
      memset((void *)ep,          0, sizeof(ep));

      USB0->USBCTRL |=  USB_USBCTRL_SUSP_MASK;          // Suspend USB transceiver
      USB0->CTL     &= ~USB_CTL_USBENSOFEN_MASK;        // Disable USB
      SIM->SCGC4    &= ~SIM_SCGC4_USBOTG_MASK;          // Disable OTG FS clock
      break;

    case ARM_POWER_FULL:
      if ((otg_fs_state & OTG_FS_USBD_DRIVER_POWERED) != 0U) { return ARM_DRIVER_OK; }

      SIM->SOPT2    |=  SIM_SOPT2_USBSRC_MASK;          // MCGFLLCLK, MCGPLLCLK, or IRC48M used as src
      SIM->SCGC4    |=  SIM_SCGC4_USBOTG_MASK;          // Enable OTG FS Clock
      USB0->USBTRC0 |=  USB_USBTRC0_USBRESET_MASK;      // Reset OTG FS module
      __nop();__nop();__nop();__nop();__nop();__nop();  // Wait for reset to finish
      USB0->CTL     &= ~USB_CTL_USBENSOFEN_MASK;        // Disable USB
      USB0->CTL     |=  USB_CTL_USBENSOFEN_MASK;        // Enable USB
      USB0->USBCTRL  =  0U;                             // Reset USB CTRL register
      USBD_Reset ();                                    // Reset variables and endpoint settings
      USB0->CTL     &= ~USB_CTL_HOSTMODEEN_MASK;        // Force Device mode
      USB0->CTL     |=  USB_CTL_ODDRST_MASK;            // Reset buffer descriptor table entry to EVEN
      USB0->CTL     &= ~USB_CTL_ODDRST_MASK;            // Clear buffer descriptor table entry reset
      USB0->USBCTRL &= ~USB_USBCTRL_SUSP_MASK;          // USB transceiver resume
      USB0->ERREN   |=  USB_ERREN_BTSERREN_MASK |       // Enable all error interrupts
                        USB_ERREN_DMAERREN_MASK |
                        USB_ERREN_BTOERREN_MASK |
                        USB_ERREN_DFN8EN_MASK   |
                        USB_ERREN_CRC16EN_MASK  |
                        USB_ERREN_CRC5EOFEN_MASK|
                        USB_ERREN_PIDERREN_MASK ;
      USB0->INTEN    =  USB_INTEN_USBRSTEN_MASK ;       // Enable USBRST interrupt

      otg_fs_state  |=  OTG_FS_USBD_DRIVER_POWERED;     // Set powered flag
      NVIC_EnableIRQ   (USB0_IRQn);                     // Enable interrupt
      break;

    default:
      return ARM_DRIVER_ERROR_UNSUPPORTED;
  }

  return ARM_DRIVER_OK;
}

/**
  \fn          int32_t USBD_DeviceConnect (void)
  \brief       Connect USB Device.
  \return      \ref execution_status
*/
static int32_t USBD_DeviceConnect (void) {

  if ((otg_fs_state & OTG_FS_USBD_DRIVER_POWERED) == 0U) { return ARM_DRIVER_ERROR; }

  USB0->CONTROL =  USB_CONTROL_DPPULLUPNONOTG_MASK;     // Activate pull-up on D+

  return ARM_DRIVER_OK;
}

/**
  \fn          int32_t USBD_DeviceDisconnect (void)
  \brief       Disconnect USB Device.
  \return      \ref execution_status
*/
static int32_t USBD_DeviceDisconnect (void) {

  if ((otg_fs_state & OTG_FS_USBD_DRIVER_POWERED) == 0U) { return ARM_DRIVER_ERROR; }

  USB0->CONTROL =  0U;                                  // De-activate pull-up on D+

  return ARM_DRIVER_OK;
}

/**
  \fn          ARM_USBD_STATE USBD_DeviceGetState (void)
  \brief       Get current USB Device State.
  \return      Device State \ref ARM_USBD_STATE
*/
static ARM_USBD_STATE USBD_DeviceGetState (void) {
  return usbd_state;
}

/**
  \fn          int32_t USBD_DeviceRemoteWakeup (void)
  \brief       Trigger USB Remote Wakeup.
  \return      \ref execution_status
*/
static int32_t USBD_DeviceRemoteWakeup (void) {

  if ((otg_fs_state & OTG_FS_USBD_DRIVER_POWERED) == 0U) { return ARM_DRIVER_ERROR; }

  USB0->CTL |=  USB_CTL_RESUME_MASK;
  osDelay(5U);
  USB0->CTL &= ~USB_CTL_RESUME_MASK;

  return ARM_DRIVER_OK;
}

/**
  \fn          int32_t USBD_DeviceSetAddress (uint8_t dev_addr)
  \brief       Set USB Device Address.
  \param[in]   dev_addr  Device Address
  \return      \ref execution_status
*/
static int32_t USBD_DeviceSetAddress (uint8_t dev_addr) {

  if ((otg_fs_state & OTG_FS_USBD_DRIVER_POWERED) == 0U) { return ARM_DRIVER_ERROR; }

  USB0->ADDR = dev_addr & 0x7FU;

  return ARM_DRIVER_OK;
}

/**
  \fn          int32_t USBD_ReadSetupPacket (uint8_t *setup)
  \brief       Read setup packet received over Control Endpoint.
  \param[out]  setup  Pointer to buffer for setup packet
  \return      \ref execution_status
*/
static int32_t USBD_ReadSetupPacket (uint8_t *setup) {

  if ((otg_fs_state & OTG_FS_USBD_DRIVER_POWERED) == 0U) { return ARM_DRIVER_ERROR; }
  if (setup_received                              == 0U) { return ARM_DRIVER_ERROR; }

  setup_received = 0U;
  memcpy(setup, setup_packet, 8);

  if (setup_received != 0U) {           // IF new setup packet was received while this was being read
    return ARM_DRIVER_ERROR;
  }

  USB0->CTL &= ~USB_CTL_TXSUSPENDTOKENBUSY_MASK;

  return ARM_DRIVER_OK;
}

/**
  \fn          int32_t USBD_EndpointConfigure (uint8_t  ep_addr,
                                               uint8_t  ep_type,
                                               uint16_t ep_max_packet_size)
  \brief       Configure USB Endpoint.
  \param[in]   ep_addr  Endpoint Address
                - ep_addr.0..3: Address
                - ep_addr.7:    Direction
  \param[in]   ep_type  Endpoint Type (ARM_USB_ENDPOINT_xxx)
  \param[in]   ep_max_packet_size Endpoint Maximum Packet Size
  \return      \ref execution_status
*/
static int32_t USBD_EndpointConfigure (uint8_t  ep_addr,
                                       uint8_t  ep_type,
                                       uint16_t ep_max_packet_size) {
  volatile ENDPOINT_t *ptr_ep;
  uint8_t              ep_num;
  bool                 ep_dir;

  ep_num = EP_NUM(ep_addr);
  if (ep_num > USBD_MAX_ENDPOINT_NUM)                    { return ARM_DRIVER_ERROR; }
  if ((otg_fs_state & OTG_FS_USBD_DRIVER_POWERED) == 0U) { return ARM_DRIVER_ERROR; }

  ptr_ep = &ep[EP_ID(ep_addr)];
  if (ptr_ep->active != 0U)                              { return ARM_DRIVER_ERROR_BUSY; }

  ep_dir = (ep_addr & ARM_USB_ENDPOINT_DIRECTION_MASK) == ARM_USB_ENDPOINT_DIRECTION_MASK;

  // Clear Endpoint transfer information (leave last_bd_odd and next_bd_odd)
  ptr_ep->data                  = NULL;
  ptr_ep->num                   = 0U;
  ptr_ep->num_transferred_total = 0U;
  ptr_ep->num_transferring[0]   = 0U;
  ptr_ep->num_transferring[1]   = 0U;
  ptr_ep->next_data_toggle      = 0U;

  // Set maximum packet size to requested
  ptr_ep->ep_max_packet_size    = ep_max_packet_size & ARM_USB_ENDPOINT_MAX_PACKET_SIZE_MASK;

  USB0->ENDPOINT[ep_num].ENDPT |= USB_ENDPT_EPHSHK_MASK                   |     // Enable Endpoint handshaking
                                 (USB_ENDPT_EPRXEN_MASK * (ep_dir == 0U)) |     // Enable RX if OUT Endpoint
                                 (USB_ENDPT_EPTXEN_MASK * (ep_dir != 0U)) ;     // Enable TX if IN  Endpoint

  return ARM_DRIVER_OK;
}

/**
  \fn          int32_t USBD_EndpointUnconfigure (uint8_t ep_addr)
  \brief       Unconfigure USB Endpoint.
  \param[in]   ep_addr  Endpoint Address
                - ep_addr.0..3: Address
                - ep_addr.7:    Direction
  \return      \ref execution_status
*/
static int32_t USBD_EndpointUnconfigure (uint8_t ep_addr) {
  volatile ENDPOINT_t *ptr_ep;
  volatile BD_t       *ptr_bd;
  uint8_t              ep_num;
  bool                 ep_dir;

  ep_num = EP_NUM(ep_addr);
  if (ep_num > USBD_MAX_ENDPOINT_NUM)                    { return ARM_DRIVER_ERROR; }
  if ((otg_fs_state & OTG_FS_USBD_DRIVER_POWERED) == 0U) { return ARM_DRIVER_ERROR; }

  ptr_ep = &ep[EP_ID(ep_addr)];
  if (ptr_ep->active != 0U)                              { return ARM_DRIVER_ERROR_BUSY; }

  ep_dir = (ep_addr & ARM_USB_ENDPOINT_DIRECTION_MASK) == ARM_USB_ENDPOINT_DIRECTION_MASK;

  USB0->ENDPOINT[ep_num].ENDPT &= ~((USB_ENDPT_EPRXEN_MASK * (ep_dir == 0U))  | // Disable RX if OUT Endpoint
                                    (USB_ENDPT_EPTXEN_MASK * (ep_dir != 0U))) ; // Disable TX if IN  Endpoint

  ptr_bd           = &bd[BD_ID(ep_addr)];
  ptr_bd->cmd_stat = 0U; ptr_bd++;      // Clear corresponding EVEN BDT entry
  ptr_bd->cmd_stat = 0U;                // Clear corresponding ODD  BDT entry

  // Clear Endpoint transfer and configuration information (leave last_bd_odd and next_bd_odd)
  ptr_ep->data                  = NULL;
  ptr_ep->num                   = 0U;
  ptr_ep->num_transferred_total = 0U;
  ptr_ep->num_transferring[0]   = 0U;
  ptr_ep->num_transferring[1]   = 0U;
  ptr_ep->next_data_toggle      = 0U;
  ptr_ep->ep_max_packet_size    = 0U;

  return ARM_DRIVER_OK;
}

/**
  \fn          int32_t USBD_EndpointStall (uint8_t ep_addr, bool stall)
  \brief       Set/Clear Stall for USB Endpoint.
  \param[in]   ep_addr  Endpoint Address
                - ep_addr.0..3: Address
                - ep_addr.7:    Direction
  \param[in]   stall  Operation
                - \b false Clear
                - \b true Set
  \return      \ref execution_status
*/
static int32_t USBD_EndpointStall (uint8_t ep_addr, bool stall) {
  volatile ENDPOINT_t *ptr_ep;
  volatile BD_t       *ptr_bd;
  uint8_t              ep_num;

  ep_num = EP_NUM(ep_addr);
  if (ep_num > USBD_MAX_ENDPOINT_NUM)                    { return ARM_DRIVER_ERROR; }
  if ((otg_fs_state & OTG_FS_USBD_DRIVER_POWERED) == 0U) { return ARM_DRIVER_ERROR; }

  ptr_ep = &ep[EP_ID(ep_addr)];
  if (ptr_ep->active != 0U)                              { return ARM_DRIVER_ERROR_BUSY; }

  if (stall) {                          // Activate STALL
    USB0->ENDPOINT[ep_num].ENDPT |=  USB_ENDPT_EPSTALL_MASK;
  } else {                              // Clear STALL
    ptr_ep->next_data_toggle = 0U;      // Clear data toggle bit

    ptr_bd           = &bd[BD_ID(ep_addr)];
    ptr_bd->cmd_stat = 0U; ptr_bd++;    // Clear corresponding EVEN BDT entry
    ptr_bd->cmd_stat = 0U;              // Clear corresponding ODD  BDT entry

    USB0->ENDPOINT[ep_num].ENDPT &= ~USB_ENDPT_EPSTALL_MASK;
  }

  return ARM_DRIVER_OK;
}

/**
  \fn          int32_t USBD_EndpointTransfer (uint8_t ep_addr, uint8_t *data, uint32_t num)
  \brief       Read data from or Write data to USB Endpoint.
  \param[in]   ep_addr  Endpoint Address
                - ep_addr.0..3: Address
                - ep_addr.7:    Direction
  \param[out]  data Pointer to buffer for data to read or with data to write
  \param[in]   num  Number of data bytes to transfer
  \return      \ref execution_status
*/
static int32_t USBD_EndpointTransfer (uint8_t ep_addr, uint8_t *data, uint32_t num) {
  volatile ENDPOINT_t *ptr_ep;
  uint8_t              ep_num;
  bool                 ep_dir;

  ep_num = EP_NUM(ep_addr);
  if (ep_num > USBD_MAX_ENDPOINT_NUM)                    { return ARM_DRIVER_ERROR; }
  if ((otg_fs_state & OTG_FS_USBD_DRIVER_POWERED) == 0U) { return ARM_DRIVER_ERROR; }

  ptr_ep = &ep[EP_ID(ep_addr)];
  if (ptr_ep->active != 0U)                              { return ARM_DRIVER_ERROR_BUSY; }

  ep_dir = (ep_addr & ARM_USB_ENDPOINT_DIRECTION_MASK) == ARM_USB_ENDPOINT_DIRECTION_MASK;

  ptr_ep->active = 1U;

  ptr_ep->data                  = data;
  ptr_ep->num                   = num;
  ptr_ep->num_transferred_total = 0U;
  ptr_ep->num_transferring[0]   = 0U;
  ptr_ep->num_transferring[1]   = 0U;
  ptr_ep->next_bd_odd           = ptr_ep->last_bd_odd ^ 1U;     // Set next odd bit

  USBD_HW_EndpointTransfer(ep_num, ep_dir);             // Start transaction

  return ARM_DRIVER_OK;
}

/**
  \fn          uint32_t USBD_EndpointTransferGetResult (uint8_t ep_addr)
  \brief       Get result of USB Endpoint transfer.
  \param[in]   ep_addr  Endpoint Address
                - ep_addr.0..3: Address
                - ep_addr.7:    Direction
  \return      number of successfully transferred data bytes
*/
static uint32_t USBD_EndpointTransferGetResult (uint8_t ep_addr) {

  if (EP_NUM(ep_addr) > USBD_MAX_ENDPOINT_NUM) { return 0U; }

  return (ep[EP_ID(ep_addr)].num_transferred_total);
}

/**
  \fn          int32_t USBD_EndpointTransferAbort (uint8_t ep_addr)
  \brief       Abort current USB Endpoint transfer.
  \param[in]   ep_addr  Endpoint Address
                - ep_addr.0..3: Address
                - ep_addr.7:    Direction
  \return      \ref execution_status
*/
static int32_t USBD_EndpointTransferAbort (uint8_t ep_addr) {
  volatile ENDPOINT_t *ptr_ep;
  volatile BD_t       *ptr_bd;

  if (EP_NUM(ep_addr) > USBD_MAX_ENDPOINT_NUM)           { return ARM_DRIVER_ERROR; }
  if ((otg_fs_state & OTG_FS_USBD_DRIVER_POWERED) == 0U) { return ARM_DRIVER_ERROR; }

  ptr_ep = &ep[EP_ID(ep_addr)];

  ptr_ep->num              = 0U;
  ptr_ep->next_data_toggle = 0U;

  ptr_bd           = &bd[BD_ID(ep_addr)];
  ptr_bd->cmd_stat =  0U; ptr_bd++;     // Clear corresponding EVEN BDT entry
  ptr_bd->cmd_stat =  0U;               // Clear corresponding ODD  BDT entry

  ptr_ep->active = 0U;

  return ARM_DRIVER_OK;
}

/**
  \fn          uint16_t USBD_GetFrameNumber (void)
  \brief       Get current USB Frame Number.
  \return      Frame Number
*/
static uint16_t USBD_GetFrameNumber (void) {

  if ((otg_fs_state & OTG_FS_USBD_DRIVER_POWERED) == 0U) { return 0U; }

  return ((((uint32_t)(USB0->FRMNUML)) | (((uint32_t)(USB0->FRMNUMH)) << 8)) & 0x07FFU);
}

/**
  \fn          void USBD_FS_IRQ (void)
  \brief       USB Device Interrupt Routine (IRQ).
*/
void USBD_FS_IRQ (void) {
  volatile ENDPOINT_t *ptr_ep;
  volatile BD_t       *ptr_bd;
  uint32_t             istat, stat, errstat;
  uint8_t              endp, tx, odd, token, en_setup;

  istat       = USB0->ISTAT & USB0->INTEN;              // Read active interrupts
  stat        = USB0->STAT;
  errstat     = USB0->ERRSTAT;
  USB0->ISTAT = istat;                                  // Clear interrupts

  en_setup = 0U;

  if ((istat & USB_ISTAT_USBRST_MASK) != 0U) {          // Reset interrupt
    USBD_Reset();                                       // USB Bus Reset
    USB0->INTEN = USB_ISTAT_STALL_MASK    |             // Enable STALL interrupt
                  USB_INTEN_SLEEPEN_MASK  |             // Enable SLEEP interrupt
                  USB_INTEN_TOKDNEEN_MASK |             // Enable TOKDNE interrupt
                  USB_INTEN_ERROREN_MASK  |             // Enable ERROR interrupt
                  USB_INTEN_USBRSTEN_MASK ;             // Enable USBRST interrupt
    SignalDeviceEvent(ARM_USBD_EVENT_RESET);
  }

  if ((istat & USB_ISTAT_SLEEP_MASK) != 0U) {           // Suspend interrupt
    usbd_state.active = false;
    SignalDeviceEvent(ARM_USBD_EVENT_SUSPEND);
  }

  if ((istat & USB_ISTAT_RESUME_MASK) != 0U) {          // Resume interrupt
    usbd_state.vbus   = true;
    usbd_state.speed  = ARM_USB_SPEED_FULL;
    usbd_state.active = true;
    SignalDeviceEvent(ARM_USBD_EVENT_RESUME);
  }

  if ((istat & USB_ISTAT_ERROR_MASK) != 0U) {           // Error interrupt
    USB0->ERRSTAT = errstat;                            // Clear all errors
  }

  if ((istat & USB_ISTAT_STALL_MASK) != 0U) {           // Stall interrupt
    if ((USB0->ENDPOINT[0].ENDPT & USB_ENDPT_EPSTALL_MASK) != 0U) {
      USB0->ENDPOINT[0].ENDPT &= ~USB_ENDPT_EPSTALL_MASK;
      en_setup = 1U;                                    // Enable reception of new SETUP Packet
    }
  }

  if ((istat & USB_ISTAT_TOKDNE_MASK) != 0U) {          // Token interrupt
    endp   = (stat >> 4) & 0x0FU;
    tx     = (stat >> 3) & 0x01U;
    odd    = (stat >> 2) & 0x01U;
    ptr_ep = &ep[EP_ID_FROM_NUM(endp,tx)];
    ptr_bd = &bd[BD_ID_FROM_NUM(endp,tx,odd)];
    ptr_ep->last_bd_odd = odd;
    token  = ((ptr_bd->cmd_stat >> 2) & 0x0FU);
    switch (token) {
      case TOK_PID_SETUP:                               // SETUP packet
        ptr_bd           = bd;                          // Disable all BDT entries for Endpoint 0
        ptr_bd->cmd_stat = 0U; ptr_bd++;
        ptr_bd->cmd_stat = 0U; ptr_bd++;
        ptr_bd->cmd_stat = 0U; ptr_bd++;
        ptr_bd->cmd_stat = 0U;

        ep[0].next_data_toggle = 1U;                    // Next OUT packet should be DATA1
        ep[1].next_data_toggle = 1U;                    // Next IN  packet should be DATA1

        setup_received = 1U;                            // Set SETUP packet pending
        SignalEndpointEvent(0, ARM_USBD_EVENT_SETUP);
        break;

      case TOK_PID_OUT:                                 // OUT packet
        ptr_ep->num_transferred_total += (ptr_bd->cmd_stat >> 16) & 0x3FFU;
        ptr_ep->num_transferring[odd]  =  0U;
        if ((endp == 0U) && (ptr_ep->num == 0U) && ((setup_packet[0] & 0x80U) != 0U)) {
           en_setup = 1U;                               // If OUT ZLP (Device-to-Host request) enable reception of new SETUP Packet
        }
        if (((ptr_ep->num_transferred_total % ptr_ep->ep_max_packet_size) != 0U) || (ptr_ep->num == ptr_ep->num_transferred_total)){
          // If all OUT data was received (required size or data terminated with ZLP or short packet), set ARM_USBD_EP_EVENT_OUT
          ptr_ep->active = 0U;          // Clear Endpoint busy flag
          SignalEndpointEvent((tx << 7) | endp, ARM_USBD_EVENT_OUT);    // Send OUT event
        } else {
          // If this is not last transfer, enable next
          if ((ptr_ep->num > (ptr_ep->num_transferred_total + ptr_ep->num_transferring[odd == 0U])) && (ptr_ep->num_transferring[odd == 0U] == 0U)) {
            // If alternate BDT entry is empty and there is data to transfer, activate it
            USBD_HW_EndpointTransfer (endp, tx);
          }
          if (ptr_ep->num > (ptr_ep->num_transferred_total + ptr_ep->num_transferring[odd == 0U])) {
            // If there is data to transfer activate it on current BDT entry
            USBD_HW_EndpointTransfer (endp, tx);
          }
        }
        break;

      case TOK_PID_IN:                                  // IN Packet
        ptr_ep->num_transferred_total += ptr_ep->num_transferring[odd];
        ptr_ep->num_transferring[odd]  = 0U;
        if ((endp == 0U) && (ptr_ep->num == 0U) && ((setup_packet[0] & 0x80U) == 0U)) {
          en_setup = 1U;                                // If IN ZLP (Host-to-Device request) enable reception of new SETUP Packet
        }
        if (ptr_ep->num == ptr_ep->num_transferred_total) {
          // If all required IN data was sent, set ARM_USBD_EP_EVENT_IN
          ptr_ep->active = 0U;          // Clear Endpoint busy flag
          SignalEndpointEvent((tx << 7) | endp, ARM_USBD_EVENT_IN);     // Send IN event
        } else {
          // If this is not last transfer, enable next
          if ((ptr_ep->num > (ptr_ep->num_transferred_total + ptr_ep->num_transferring[odd == 0U])) && (ptr_ep->num_transferring[odd == 0U] == 0U)) {
            // If alternate BDT entry is empty and there is data to transfer, activate it
            USBD_HW_EndpointTransfer (endp, tx);
          }
          if (ptr_ep->num > (ptr_ep->num_transferred_total + ptr_ep->num_transferring[odd == 0U])) {
            // If there is data to transfer activate it on current BDT entry
            USBD_HW_EndpointTransfer (endp, tx);
          }
        }
        break;
    }
  }
  if (en_setup != 0U) {                                 // If SETUP is expected, enable it's reception
    ptr_ep  = &ep[0];
    odd     =  ptr_ep->last_bd_odd ^ 1U;                // Next is alternate of last
    ptr_bd  = &bd[odd];                                 // Endpoint 0 OUT Buffer Descriptor Table (BDT) entry
    // Enable reception of SETUP Packet or OUT ZLP (DATA1)
    ptr_bd->buf_addr = (uint32_t)setup_packet;
    ptr_bd->cmd_stat = (8U << 16) | BD_OWN_MASK | BD_DTS_MASK;
  }
}

ARM_DRIVER_USBD Driver_USBD0 = {
  USBD_GetVersion,
  USBD_GetCapabilities,
  USBD_Initialize,
  USBD_Uninitialize,
  USBD_PowerControl,
  USBD_DeviceConnect,
  USBD_DeviceDisconnect,
  USBD_DeviceGetState,
  USBD_DeviceRemoteWakeup,
  USBD_DeviceSetAddress,
  USBD_ReadSetupPacket,
  USBD_EndpointConfigure,
  USBD_EndpointUnconfigure,
  USBD_EndpointStall,
  USBD_EndpointTransfer,
  USBD_EndpointTransferGetResult,
  USBD_EndpointTransferAbort,
  USBD_GetFrameNumber
};
