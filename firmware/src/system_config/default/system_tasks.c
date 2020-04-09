/*******************************************************************************
 System Tasks File

  File Name:
    system_tasks.c

  Summary:
    This file contains source code necessary to maintain system's polled state
    machines.

  Description:
    This file contains source code necessary to maintain system's polled state
    machines.  It implements the "SYS_Tasks" function that calls the individual
    "Tasks" functions for all the MPLAB Harmony modules in the system.

  Remarks:
    This file requires access to the systemObjects global data structure that
    contains the object handles to all MPLAB Harmony module objects executing
    polled in the system.  These handles are passed into the individual module
    "Tasks" functions to identify the instance of the module to maintain.
 *******************************************************************************/

// DOM-IGNORE-BEGIN
/*******************************************************************************
Copyright (c) 2013-2015 released Microchip Technology Inc.  All rights reserved.

Microchip licenses to you the right to use, modify, copy and distribute
Software only when embedded on a Microchip microcontroller or digital signal
controller that is integrated into your product or third party product
(pursuant to the sublicense terms in the accompanying license agreement).

You should refer to the license agreement accompanying this Software for
additional information regarding your rights and obligations.

SOFTWARE AND DOCUMENTATION ARE PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
EITHER EXPRESS OR IMPLIED, INCLUDING WITHOUT LIMITATION, ANY WARRANTY OF
MERCHANTABILITY, TITLE, NON-INFRINGEMENT AND FITNESS FOR A PARTICULAR PURPOSE.
IN NO EVENT SHALL MICROCHIP OR ITS LICENSORS BE LIABLE OR OBLIGATED UNDER
CONTRACT, NEGLIGENCE, STRICT LIABILITY, CONTRIBUTION, BREACH OF WARRANTY, OR
OTHER LEGAL EQUITABLE THEORY ANY DIRECT OR INDIRECT DAMAGES OR EXPENSES
INCLUDING BUT NOT LIMITED TO ANY INCIDENTAL, SPECIAL, INDIRECT, PUNITIVE OR
CONSEQUENTIAL DAMAGES, LOST PROFITS OR LOST DATA, COST OF PROCUREMENT OF
SUBSTITUTE GOODS, TECHNOLOGY, SERVICES, OR ANY CLAIMS BY THIRD PARTIES
(INCLUDING BUT NOT LIMITED TO ANY DEFENSE THEREOF), OR OTHER SIMILAR COSTS.
 *******************************************************************************/
// DOM-IGNORE-END


// *****************************************************************************
// *****************************************************************************
// Section: Included Files
// *****************************************************************************
// *****************************************************************************

#include "system_config.h"
#include "system_definitions.h"
#include "app_uart_term.h"
#include "app_netpie.h"
#include "app_pubsub.h"
#include "app_logger.h"


// *****************************************************************************
// *****************************************************************************
// Section: Local Prototypes
// *****************************************************************************
// *****************************************************************************


 
static void _SYS_Tasks ( void );
static void _APP_UART_TERM_Tasks(void);
static void _APP_NETPIE_Tasks(void);
static void _APP_PUBSUB_Tasks(void);
static void _APP_LOGGER_Tasks(void);


// *****************************************************************************
// *****************************************************************************
// Section: System "Tasks" Routine
// *****************************************************************************
// *****************************************************************************

/*******************************************************************************
  Function:
    void SYS_Tasks ( void )

  Remarks:
    See prototype in system/common/sys_module.h.
*/

void SYS_Tasks ( void )
{
    /* Create OS Thread for Sys Tasks. */
    xTaskCreate((TaskFunction_t) _SYS_Tasks,
                "Sys Tasks",
                4096, NULL, 2, NULL);

    /* Create OS Thread for APP_UART_TERM Tasks. */
    xTaskCreate((TaskFunction_t) _APP_UART_TERM_Tasks,
                "APP_UART_TERM Tasks",
                1024, NULL, 1, NULL);

    /* Create OS Thread for APP_NETPIE Tasks. */
    xTaskCreate((TaskFunction_t) _APP_NETPIE_Tasks,
                "APP_NETPIE Tasks",
                1024, NULL, 1, NULL);

    /* Create OS Thread for APP_PUBSUB Tasks. */
    xTaskCreate((TaskFunction_t) _APP_PUBSUB_Tasks,
                "APP_PUBSUB Tasks",
                1024, NULL, 1, NULL);

    /* Create OS Thread for APP_LOGGER Tasks. */
    xTaskCreate((TaskFunction_t) _APP_LOGGER_Tasks,
                "APP_LOGGER Tasks",
                1024, NULL, 1, NULL);

    /**************
     * Start RTOS * 
     **************/
    vTaskStartScheduler(); /* This function never returns. */
}


/*******************************************************************************
  Function:
    void _SYS_Tasks ( void )

  Summary:
    Maintains state machines of system modules.
*/
static void _SYS_Tasks ( void)
{
    while(1)
    {
        /* Maintain system services */
        SYS_DEVCON_Tasks(sysObj.sysDevcon);
    /* SYS_TMR Device layer tasks routine */ 
    SYS_TMR_Tasks(sysObj.sysTmr);

        /* Maintain Device Drivers */

        /* Maintain Middleware */
    /* Maintain the TCP/IP Stack*/
    TCPIP_STACK_Task(sysObj.tcpip);
    NET_PRES_Tasks(sysObj.netPres);

        /* Task Delay */
        vTaskDelay(1 / portTICK_PERIOD_MS);
    }
}


/*******************************************************************************
  Function:
    void _APP_UART_TERM_Tasks ( void )

  Summary:
    Maintains state machine of APP_UART_TERM.
*/

static void _APP_UART_TERM_Tasks(void)
{
    while(1)
    {
        APP_UART_TERM_Tasks();
    }
}


/*******************************************************************************
  Function:
    void _APP_NETPIE_Tasks ( void )

  Summary:
    Maintains state machine of APP_NETPIE.
*/

static void _APP_NETPIE_Tasks(void)
{
    while(1)
    {
        APP_NETPIE_Tasks();
    }
}


/*******************************************************************************
  Function:
    void _APP_PUBSUB_Tasks ( void )

  Summary:
    Maintains state machine of APP_PUBSUB.
*/

static void _APP_PUBSUB_Tasks(void)
{
    while(1)
    {
        APP_PUBSUB_Tasks();
    }
}


/*******************************************************************************
  Function:
    void _APP_LOGGER_Tasks ( void )

  Summary:
    Maintains state machine of APP_LOGGER.
*/

static void _APP_LOGGER_Tasks(void)
{
    while(1)
    {
        APP_LOGGER_Tasks();
    }
}


/*******************************************************************************
 End of File
 */

