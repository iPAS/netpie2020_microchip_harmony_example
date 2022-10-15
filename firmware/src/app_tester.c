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

#define TESTER_COUNT_MAX 2
#if defined(ONLY_SERVICE) && (ONLY_SERVICE != 0)
#define TESTER_WAIT_MS 5000
#else
#define TESTER_WAIT_MS 60000
#endif


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
                app_testerData.state = APP_TESTER_STATE_WAIT;

                #if defined(RUN_TESTER) && (RUN_TESTER == 0)
                // app_testerData.state = APP_TESTER_STATE_FINISH;
                vTaskSuspend(NULL);  // Suspend itself
                #endif
            }
            break;
        }

        case APP_TESTER_STATE_WAIT:
        {
            int i;

            for (i = TESTER_COUNT_MAX; i > 0; i--)
            {
                logger_send_tx_queue(">>> %d:%d to start testing\n\r", (i * TESTER_WAIT_MS/1000)/60, (i * TESTER_WAIT_MS/1000)%60);
                vTaskDelay(TESTER_WAIT_MS / portTICK_PERIOD_MS);
            }

            #if defined(ONLY_SERVICE) && (ONLY_SERVICE != 0)
            app_testerData.state = APP_TESTER_STATE_TEST_ONLY_SERVICE;
            #else
            app_testerData.state = APP_TESTER_STATE_TURN_NETPIE_OFF_ON;
            #endif
            break;
        }

        /* TODO: implement your application state machine.*/
        case APP_TESTER_STATE_TURN_NETPIE_OFF_ON:
        {
            int i;

            logger_send_tx_queue(">>> Disable the Netpie module in 3 seconds\n\r");
            vTaskDelay(3000 / portTICK_PERIOD_MS);

            netpie_set_running(false);  // Disable app_netpie

            for (i = 2; i > 0; i--)
            {
                logger_send_tx_queue(">>> %d:%d before enable the Netpie module\n\r", (i * TESTER_WAIT_MS/1000)/60, (i * TESTER_WAIT_MS/1000)%60);
                vTaskDelay(TESTER_WAIT_MS / portTICK_PERIOD_MS);
            }

            netpie_set_running(true);  // Enable app_netpie

            logger_send_tx_queue(">>> Enable the Netpie module\n\r");
            for (i = 2; i > 0; i--)
            {
                logger_send_tx_queue(">>> %d:%d before next test, off-on the logger\n\r", (i * TESTER_WAIT_MS/1000)/60, (i * TESTER_WAIT_MS/1000)%60);
                vTaskDelay(TESTER_WAIT_MS / portTICK_PERIOD_MS);
            }

            app_testerData.state = APP_TESTER_STATE_TURN_LOGGER_OFF_ON;
            break;
        }

        case APP_TESTER_STATE_TURN_LOGGER_OFF_ON:
        {
            logger_send_tx_queue(">>> Disable the Logger module\n\r");
            vTaskDelay(10000 / portTICK_PERIOD_MS);

            logger_set_running(false);  // Disable app_logger

            vTaskDelay(10000 / portTICK_PERIOD_MS);  // Out-of-service period
            logger_send_tx_queue(">>> You are supposed to never see me (logger)!!\n\r");
            vTaskDelay(10000 / portTICK_PERIOD_MS);  // Out-of-service

            logger_set_running(true);  // Enable app_logger

            vTaskDelay(10000 / portTICK_PERIOD_MS);
            logger_send_tx_queue(">>> Hello I am 'logger', back!!\n\r");

            app_testerData.state = APP_TESTER_STATE_FINISH;
            break;
        }


        case APP_TESTER_STATE_TEST_ONLY_SERVICE:
        {
            #if defined(ONLY_SERVICE) && (ONLY_SERVICE != 0)

            const char mqtt_topic[] = "only_service";
            const char mqtt_msg_fmt[] = "count me %d";
            char mqtt_msg[20];
            uint8_t cnt = 0;

            logger_send_tx_queue(">>> Test pubsub_send() cnt=%d\n\r", cnt);

            snprintf(mqtt_msg, sizeof(mqtt_msg), mqtt_msg_fmt, cnt++);
            pubsub_send(mqtt_topic, mqtt_msg);
            snprintf(mqtt_msg, sizeof(mqtt_msg), mqtt_msg_fmt, cnt++);
            pubsub_send(mqtt_topic, mqtt_msg);
            snprintf(mqtt_msg, sizeof(mqtt_msg), mqtt_msg_fmt, cnt++);
            pubsub_send(mqtt_topic, mqtt_msg);
            snprintf(mqtt_msg, sizeof(mqtt_msg), mqtt_msg_fmt, cnt++);
            pubsub_send(mqtt_topic, mqtt_msg);
            snprintf(mqtt_msg, sizeof(mqtt_msg), mqtt_msg_fmt, cnt++);
            pubsub_send(mqtt_topic, mqtt_msg);

            vTaskDelay(10000 / portTICK_PERIOD_MS);

            #endif

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
            for (i = TESTER_COUNT_MAX*2; i > 0; i--)
            {
                logger_send_tx_queue(">>> %d:%d to restart all over again...\n\r", (i * TESTER_WAIT_MS/1000)/60, (i * TESTER_WAIT_MS/1000)%60);
                vTaskDelay(TESTER_WAIT_MS / portTICK_PERIOD_MS);
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
