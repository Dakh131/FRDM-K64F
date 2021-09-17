
/*
 * Auto generated Run-Time-Environment Configuration File
 *      *** Do not modify ! ***
 *
 * Project: 'VirtualCOM' 
 * Target:  'Target 1' 
 */

#ifndef RTE_COMPONENTS_H
#define RTE_COMPONENTS_H


/*
 * Define the Device Header File: 
 */
#define CMSIS_device_header "fsl_device_registers.h"

/* ARM::CMSIS:RTOS2:Keil RTX5:Library:5.5.3 */
#define RTE_CMSIS_RTOS2                 /* CMSIS-RTOS2 */
        #define RTE_CMSIS_RTOS2_RTX5            /* CMSIS-RTOS2 Keil RTX5 */
/* ARM::CMSIS:RTOS:Keil RTX5:5.5.3 */
#define RTE_CMSIS_RTOS                  /* CMSIS-RTOS */
        #define RTE_CMSIS_RTOS_RTX5             /* CMSIS-RTOS Keil RTX5 */
/* Keil.ARM Compiler::Compiler:Event Recorder:DAP:1.4.0 */
#define RTE_Compiler_EventRecorder
          #define RTE_Compiler_EventRecorder_DAP
/* Keil.ARM Compiler::Compiler:I/O:STDERR:Breakpoint:1.2.0 */
#define RTE_Compiler_IO_STDERR          /* Compiler I/O: STDERR */
          #define RTE_Compiler_IO_STDERR_BKPT     /* Compiler I/O: STDERR Breakpoint */
/* Keil.ARM Compiler::Compiler:I/O:STDIN:Breakpoint:1.2.0 */
#define RTE_Compiler_IO_STDIN           /* Compiler I/O: STDIN */
          #define RTE_Compiler_IO_STDIN_BKPT      /* Compiler I/O: STDIN Breakpoint */
/* Keil.MDK-Plus::USB:CORE:Debug:6.15.0 */
#define RTE_USB_Core                    /* USB Core */
          #define RTE_USB_Core_Debug              /* USB Core Debug Version */
/* Keil.MDK-Plus::USB:Device:6.15.0 */
#define RTE_USB_Device_0                /* USB Device 0 */

/* Keil.MDK-Plus::USB:Device:CDC:6.15.0 */
#define RTE_USB_Device_CDC_0            /* USB Device CDC instance 0 */

/* NXP::Device:SDK Utilities:serial_manager_virtual:1.0.0 */
#ifndef SERIAL_PORT_TYPE_VIRTUAL
#define SERIAL_PORT_TYPE_VIRTUAL 1
#endif
#ifndef DEBUG_CONSOLE_TRANSFER_NON_BLOCKING
#define DEBUG_CONSOLE_TRANSFER_NON_BLOCKING 
#endif


#endif /* RTE_COMPONENTS_H */
