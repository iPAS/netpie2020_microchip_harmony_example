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
#include "register_mapping.h"

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

#define REGISTER_UPDATE_INTERVAL_MS 30000 


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


static void mqttclient_callback(const char *sub_topic, const char *message)
{
    TRACE_LOG("[Tester] --- calling back for updating reg:'%s' with '%s'\n\r", sub_topic, message);  // DEBUG: iPAS

    st_register_t *p_reg = st_registers;
    char *endptr = NULL;
    float value;
    
    while (p_reg->sub_topic != NULL)  // Find the matched reference
    {
        if (strcmp(sub_topic, p_reg->sub_topic) == 0)
        {
            if (strlen(message) == 0)  // Just refresh
            {
                
            }
            else  // Update if valid
            {
                value = strtof(message, &endptr);
            }
            break;
        }
        p_reg++;
    }
    
    
    char msg[50];
    
    if (p_reg->sub_topic == NULL)  // Reference error
    {
        snprintf(msg, sizeof(msg), "Unknown sub_topic:'%s'", sub_topic);
        mqttclient_publish_log(msg);
    } else
    if (endptr == message)  // Value conversion error
    {
        snprintf(msg, sizeof(msg), "Conversion error on message:'%s'", message);
        mqttclient_publish_log(msg);
    } 
    else
    {
        if (endptr != NULL)  // Valid
            *p_reg->p_value = value;            
        snprintf(msg, sizeof(msg), "%f", *p_reg->p_value);
        mqttclient_publish_register(sub_topic, msg);
    }
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
            static st_register_t *p_reg = st_registers;
            
            if (mqttclient_ready())
            {
                char message[20];
                const char *sub_topic = p_reg->sub_topic;
                
                snprintf(message, sizeof(message), "%f", *p_reg->p_value);
                
                TRACE_LOG("[Tester] publish every %.2fs sub_topic:%s > '%s'\n\r", 
                        REGISTER_UPDATE_INTERVAL_MS/1000.0, sub_topic, message);  // DEBUG: iPAS
                mqttclient_publish_register(sub_topic, message);
                
                p_reg++;
                if (p_reg->sub_topic == NULL)
                {
                    p_reg = st_registers;
                    vTaskDelay(REGISTER_UPDATE_INTERVAL_MS / portTICK_PERIOD_MS);
                }
                else
                {
                    vTaskDelay(100 / portTICK_PERIOD_MS);
                }
            }
            else
            {
                TRACE_LOG("[Tester] Wait MQTT ready ...\n\r");  // DEBUG: iPAS
                vTaskDelay(1000 / portTICK_PERIOD_MS);
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
