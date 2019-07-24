
/*
 * Auto generated Run-Time-Environment Configuration File
 *      *** Do not modify ! ***
 *
 * Project: 'HTTP_Server' 
 * Target:  'DEBUG_MK64FN1M Flash' 
 */

#ifndef RTE_COMPONENTS_H
#define RTE_COMPONENTS_H


/*
 * Define the Device Header File: 
 */
#define CMSIS_device_header "fsl_device_registers.h"

/*  ARM::CMSIS:RTOS:Keil RTX:4.82.0 */
#define RTE_CMSIS_RTOS                  /* CMSIS-RTOS */
        #define RTE_CMSIS_RTOS_RTX              /* CMSIS-RTOS Keil RTX */
/*  Keil.ARM Compiler::Compiler:Event Recorder:DAP:1.4.0 */
#define RTE_Compiler_EventRecorder
          #define RTE_Compiler_EventRecorder_DAP
/*  Keil.ARM Compiler::Compiler:I/O:STDOUT:ITM:1.2.0 */
#define RTE_Compiler_IO_STDOUT          /* Compiler I/O: STDOUT */
          #define RTE_Compiler_IO_STDOUT_ITM      /* Compiler I/O: STDOUT ITM */
/*  Keil.MDK-Pro::Network:CORE:IPv4/IPv6 Debug:7.11.0 */
#define RTE_Network_Core                /* Network Core */
          #define RTE_Network_IPv4                /* Network IPv4 Stack */
          #define RTE_Network_IPv6                /* Network IPv6 Stack */
          #define RTE_Network_Debug               /* Network Debug Version */
/*  Keil.MDK-Pro::Network:Interface:ETH:7.11.0 */
#define RTE_Network_Interface_ETH_0     /* Network Interface ETH 0 */

/*  Keil.MDK-Pro::Network:Service:Web Server Compact:HTTP:7.11.0 */
#define RTE_Network_Web_Server_RO       /* Network Web Server with Read-only Web Resources */
/*  Keil.MDK-Pro::Network:Socket:TCP:7.11.0 */
#define RTE_Network_Socket_TCP          /* Network Socket TCP */
/*  Keil.MDK-Pro::Network:Socket:UDP:7.11.0 */
#define RTE_Network_Socket_UDP          /* Network Socket UDP */


#endif /* RTE_COMPONENTS_H */
