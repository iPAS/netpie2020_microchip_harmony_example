/*******************************************************************************
  MPLAB Harmony Application Source File
  
  Company:
    Microchip Technology Inc.
  
  File Name:
    app0_uart.c

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

#include <stdarg.h>

#include "app0_uart.h"

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

APP0_UART_DATA app0_uartData;
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

/**
 * Enqueue a message to UART0 Tx
 */
BaseType_t uart0_send_tx_queue(const char *fmt, ... )
{
    va_list args;
    uart0_queue_item_t q_item;
    uint16_t len;

    va_start(args, fmt);
    len = vsnprintf(q_item.buffer, UART0_QUEUE_ITEM_SIZE, fmt, args);
    va_end(args);

    q_item.length = strlen(q_item.buffer);
    return xQueueSendToBack(app0_uartData.q_tx, &q_item, 0);
}


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
            app0_uartData.q_tx = xQueueCreate(UART0_QUEUE_SIZE, sizeof(uart0_queue_item_t));
            app0_uartData.q_rx = xQueueCreate(UART0_QUEUE_SIZE, sizeof(uart0_queue_item_t));
            if (app0_uartData.q_tx == NULL || app0_uartData.q_rx == NULL)
            {
                // Some error
            }
            xQueueReset(app0_uartData.q_tx);
            xQueueReset(app0_uartData.q_rx);
            
            usartBMState = USART_BM_WORKING;
            break;
        }

        case USART_BM_WORKING:
        {
            // ******
            // * TX *
            if (!DRV_USART_TransmitBufferIsFull(app0_uartData.handleUSART0))
            {
                static uint8_t index = 0;
                static uart0_queue_item_t q_item;
                bool do_send = true;

                if (index == 0)
                {
                    //uxQueueMessagesWaiting();
                    do_send = xQueueReceive(app0_uartData.q_tx, &q_item, 0);
                }

                if (do_send)
                {
                    DRV_USART_WriteByte(app0_uartData.handleUSART0, q_item.buffer[index]);
                    index = (index == q_item.length-1)? 0 : index+1;
                }
            }

            // ******
            // * RX *
            if (!DRV_USART_ReceiverBufferIsEmpty(app0_uartData.handleUSART0))
            {
                if (uxQueueSpacesAvailable(app0_uartData.q_rx) > 0)
                {
                    //= DRV_USART_ReadByte(app0_uartData.handleUSART0);
                }
            }

            //usartBMState = USART_BM_DONE;
            break;
        }

        case USART_BM_DONE:
        {
            vTaskDelay(100 / portTICK_PERIOD_MS);
            usartBMState = USART_BM_WORKING;
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
    void APP0_UART_Initialize ( void )

  Remarks:
    See prototype in app0_uart.h.
 */

void APP0_UART_Initialize ( void )
{
    /* Place the App state machine in its initial state. */
    app0_uartData.state = APP0_UART_STATE_INIT;

    app0_uartData.handleUSART0 = DRV_HANDLE_INVALID;
    
    /* TODO: Initialize your application's state machine and other
     * parameters.
     */
}


/******************************************************************************
  Function:
    void APP0_UART_Tasks ( void )

  Remarks:
    See prototype in app0_uart.h.
 */

void APP0_UART_Tasks ( void )
{

    /* Check the application's current state. */
    switch ( app0_uartData.state )
    {
        /* Application's initial state. */
        case APP0_UART_STATE_INIT:
        {
            bool appInitialized = true;
       
            if (app0_uartData.handleUSART0 == DRV_HANDLE_INVALID)
            {
                app0_uartData.handleUSART0 = DRV_USART_Open(APP0_UART_DRV_USART, DRV_IO_INTENT_READWRITE|DRV_IO_INTENT_NONBLOCKING);
                appInitialized &= ( DRV_HANDLE_INVALID != app0_uartData.handleUSART0 );
            }
        
            if (appInitialized)
            {
                /* initialize the USART state machine */
                usartBMState = USART_BM_INIT;
            
                app0_uartData.state = APP0_UART_STATE_SERVICE_TASKS;
            }
            break;
        }

        case APP0_UART_STATE_SERVICE_TASKS:
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
