/*******************************************************************************
  MPLAB Harmony Application Source File
  
  Company:
    Microchip Technology Inc.
  
  File Name:
    app_logger.c

  Summary:
    This file contains the source code for the MPLAB Harmony application.

  Description:
    This file contains the source code for the MPLAB Harmony application.  It 
    implements the logic of the application's state machine and it may call 
    API routines of other MPLAB Harmony modules in the system, such as drivers,
    system services, and middleware.  However, it does not call any of the
    system interfaces (such as the "Initialize" and "Tasks" functions) of any of
    the modules in the system or make any assumptions about when those functions
    are called.  That is the responsibility of the configuration-specific system
    files.
 *******************************************************************************/

// DOM-IGNORE-BEGIN
/*******************************************************************************
Copyright (c) 2013-2014 released Microchip Technology Inc.  All rights reserved.

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

#include "app_logger.h"

// *****************************************************************************
// *****************************************************************************
// Section: Global Data Definitions
// *****************************************************************************
// *****************************************************************************

// *****************************************************************************
/* Application Data

  Summary:
    Holds application data

  Description:
    This structure holds the application's data.

  Remarks:
    This structure should be initialized by the APP_Initialize function.
    
    Application strings and buffers are be defined outside this structure.
*/

APP_LOGGER_DATA app_loggerData;
static uint8_t app_logger_tx_buf[] = "Hello World\r\n";
static uint8_t app_logger_rx_buf[10];
static enum 
{
    USART_BM_INIT,
    USART_BM_WORKING,
    USART_BM_DONE,
} usartBMState;


// *****************************************************************************
// *****************************************************************************
// Section: Application Callback Functions
// *****************************************************************************
// *****************************************************************************

/* TODO:  Add any necessary callback functions.
*/

// *****************************************************************************
// *****************************************************************************
// Section: Application Local Functions
// *****************************************************************************
// *****************************************************************************

/******************************************************************************
  Function:
    static void USART_Task (void)
    
   Remarks:
    Feeds the USART transmitter by reading characters from a specified pipe.  The pipeRead function is a 
    standard interface that allows data to be exchanged between different automatically 
    generated application modules.  Typically, the pipe is connected to the application's
    USART receive function, but could be any other Harmony module which supports the pipe interface. 
*/
static void USART_Task (void)
{
    switch (usartBMState)
    {
        default:
        case USART_BM_INIT:
        {
            app_loggerData.tx_count = 0;
            app_loggerData.rx_count = 0;
            usartBMState = USART_BM_WORKING;
            break;
        }

        case USART_BM_WORKING:
        {
            if (app_loggerData.tx_count < sizeof(app_logger_tx_buf)) 
            {
                if(!DRV_USART_TransmitBufferIsFull(app_loggerData.handleUSART1))
                {
                    DRV_USART_WriteByte(app_loggerData.handleUSART1, app_logger_tx_buf[app_loggerData.tx_count]);
                    app_loggerData.tx_count++;
                }
            }

            if (app_loggerData.rx_count < sizeof(app_logger_rx_buf)) 
            {
                if(!DRV_USART_ReceiverBufferIsEmpty(app_loggerData.handleUSART1))
                {
                    app_logger_rx_buf[app_loggerData.rx_count] = DRV_USART_ReadByte(app_loggerData.handleUSART1);
                    app_loggerData.rx_count++;
                }
            }

            /* Have we finished? */
            if (app_loggerData.tx_count == sizeof(app_logger_tx_buf) 
                    //&& app_loggerData.rx_count == sizeof(app_logger_rx_buf)
                    )
            {
                usartBMState = USART_BM_DONE;
            }
            break;
        }

        case USART_BM_DONE:
        {
            usartBMState = USART_BM_INIT;
            vTaskDelay(1000 / portTICK_PERIOD_MS);
            break;
        }
    }
}

/* TODO:  Add any necessary local functions.
*/


// *****************************************************************************
// *****************************************************************************
// Section: Application Initialization and State Machine Functions
// *****************************************************************************
// *****************************************************************************

/*******************************************************************************
  Function:
    void APP_LOGGER_Initialize ( void )

  Remarks:
    See prototype in app_logger.h.
 */

void APP_LOGGER_Initialize ( void )
{
    /* Place the App state machine in its initial state. */
    app_loggerData.state = APP_LOGGER_STATE_INIT;

    app_loggerData.handleUSART1 = DRV_HANDLE_INVALID;
    
    /* TODO: Initialize your application's state machine and other
     * parameters.
     */
}


/******************************************************************************
  Function:
    void APP_LOGGER_Tasks ( void )

  Remarks:
    See prototype in app_logger.h.
 */

void APP_LOGGER_Tasks ( void )
{

    /* Check the application's current state. */
    switch ( app_loggerData.state )
    {
        /* Application's initial state. */
        case APP_LOGGER_STATE_INIT:
        {
            bool appInitialized = true;
       
            if (app_loggerData.handleUSART1 == DRV_HANDLE_INVALID)
            {
                app_loggerData.handleUSART1 = DRV_USART_Open(APP_LOGGER_DRV_USART, DRV_IO_INTENT_READWRITE|DRV_IO_INTENT_NONBLOCKING);
                appInitialized &= ( DRV_HANDLE_INVALID != app_loggerData.handleUSART1 );
            }
        
            if (appInitialized)
            {
                /* initialize the USART state machine */
                usartBMState = USART_BM_INIT;
            
                app_loggerData.state = APP_LOGGER_STATE_SERVICE_TASKS;
            }
            break;
        }

        case APP_LOGGER_STATE_SERVICE_TASKS:
        {
			USART_Task();
        
            break;
        }

        /* TODO: implement your application state machine.*/
        

        /* The default state should never be executed. */
        default:
        {
            /* TODO: Handle error in application's state machine. */
            break;
        }
    }
}

 

/*******************************************************************************
 End of File
 */
