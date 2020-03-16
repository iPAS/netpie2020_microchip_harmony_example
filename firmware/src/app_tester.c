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
#include "app_mqtt_client.h"

#ifdef DO_TRACE
#include "app_uart_term.h"
#define TRACE_LOG(...) uart_send_tx_queue(__VA_ARGS__)
#else
#define TRACE_LOG(...)
#endif


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

#define LEN_OF_ARRAY(arr) (sizeof(arr)/sizeof(arr[0]))


static void mqttclient_callback(uint32_t address, const char *message)
{
    TRACE_LOG("[Tester] calling back for updating %d with '%s'\n\r", address, message);  // DEBUG: iPAS
}


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

    mqttclient_set_callback(mqttclient_callback);
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
            }
            break;
        }

        case APP_TESTER_STATE_SERVICE_TASKS:
        {
            vTaskDelay(10000 / portTICK_PERIOD_MS);
            if (mqttclient_ready())
            {
                const uint32_t addresses[] = {30000, 30100, 40000};
                static uint8_t i = 0;
                static uint16_t cnt = 0;
                char message[10];
                snprintf(message, sizeof(message), "%.4X", cnt);
                
                mqttclient_publish_register(addresses[i], message);
                
                i++;
                if (i == LEN_OF_ARRAY(addresses))
                {
                   i = 0;
                   cnt++;
                }
            }             
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
