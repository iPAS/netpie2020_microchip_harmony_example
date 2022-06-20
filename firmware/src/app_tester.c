/*******************************************************************************
  MPLAB Harmony Application Source File
  
  Company:
    Microchip Technology Inc.
  
  File Name:
    app_tester.c

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

#include "app_tester.h"

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

APP_TESTER_DATA app_testerData;

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


/* TODO:  Add any necessary local functions.
*/


// *****************************************************************************
// *****************************************************************************
// Section: Application Initialization and State Machine Functions
// *****************************************************************************
// *****************************************************************************

/*******************************************************************************
  Function:
    void APP_TESTER_Initialize ( void )

  Remarks:
    See prototype in app_tester.h.
 */

void APP_TESTER_Initialize ( void )
{
    /* Place the App state machine in its initial state. */
    app_testerData.state = APP_TESTER_STATE_INIT;

    
    /* TODO: Initialize your application's state machine and other
     * parameters.
     */
}


/******************************************************************************
  Function:
    void APP_TESTER_Tasks ( void )

  Remarks:
    See prototype in app_tester.h.
 */

void APP_TESTER_Tasks ( void )
{

    /* Check the application's current state. */
    switch ( app_testerData.state )
    {
        /* Application's initial state. */
        case APP_TESTER_STATE_INIT:
        {
            bool appInitialized = true;

            if (appInitialized)
            {
                app_testerData.state = APP_TESTER_STATE_SERVICE_TASKS;

                #if defined(DO_TEST) && (DO_TEST == 0)
                // app_testerData.state = APP_TESTER_STATE_FINISH;
                vTaskSuspend(NULL);  // Suspend itself
                #endif
            }
            break;
        }

        case APP_TESTER_STATE_SERVICE_TASKS:
        {
            int i;
            for (i = 2; i > 0; i--)
            {
                logger_send_tx_queue(">>> %d min. to start testing\n\r", i);
                vTaskDelay(60000 / portTICK_PERIOD_MS);
            }

            app_testerData.state = APP_TESTER_STATE_TURN_NETPIE_OFF_ON;
            break;
        }

        /* TODO: implement your application state machine.*/
        case APP_TESTER_STATE_TURN_NETPIE_OFF_ON:
        {   
            int i;

            logger_send_tx_queue(">>> Disable the Netpie module in 3 seconds\n\r");
            vTaskDelay(3000 / portTICK_PERIOD_MS);
            netpie_set_running(false);
            for (i = 2; i > 0; i--)
            {
                logger_send_tx_queue(">>> %d min. before enable\n\r", i);
                vTaskDelay(60000 / portTICK_PERIOD_MS);
            }

            netpie_set_running(true);
            logger_send_tx_queue(">>> Enable the Netpie module\n\r");
            for (i = 2; i > 0; i--)
            {
                logger_send_tx_queue(">>> %d min. before next test\n\r", i);
                vTaskDelay(60000 / portTICK_PERIOD_MS);
            }

            app_testerData.state = APP_TESTER_STATE_TURN_LOGGER_OFF_ON;
            break;
        }        

        case APP_TESTER_STATE_TURN_LOGGER_OFF_ON:
        {
            logger_send_tx_queue(">>> Disable the Logger module\n\r");
            vTaskDelay(10000 / portTICK_PERIOD_MS);
            logger_set_running(false);
            
            vTaskDelay(10000 / portTICK_PERIOD_MS); 
            logger_send_tx_queue(">>> Never see me!!\n\r");

            vTaskDelay(10000 / portTICK_PERIOD_MS);
            logger_set_running(true);

            vTaskDelay(10000 / portTICK_PERIOD_MS);
            logger_send_tx_queue(">>> Hello I am back!!\n\r");

            app_testerData.state = APP_TESTER_STATE_FINISH;
            break;
        }

        case APP_TESTER_STATE_FINISH:
        {
            //app_testerData.state = APP_TESTER_STATE_INIT;
            //vTaskSuspend(NULL);  // Suspend itself
            
            app_testerData.state = APP_TESTER_STATE_WAIT_BEFORE_NEXT;
            break;
        }
        
        case APP_TESTER_STATE_WAIT_BEFORE_NEXT:
        {
            int i;
            for (i = 5; i > 0; i--)
            {
                logger_send_tx_queue(">>> %d min. to restart all testing again\n\r", i);
                vTaskDelay(60000 / portTICK_PERIOD_MS);
            }

            app_testerData.state = APP_TESTER_STATE_INIT;
            break;
        }

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
