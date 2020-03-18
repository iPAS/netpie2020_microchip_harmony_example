/*******************************************************************************
  MPLAB Harmony Application Header File

  Company:
    Microchip Technology Inc.

  File Name:
    app_mqtt_client.h

  Summary:
    This header file provides prototypes and definitions for the application.

  Description:
    This header file provides function prototypes and data type definitions for
    the application.  Some of these are required by the system (such as the
    "APP_Initialize" and "APP_Tasks" prototypes) and some of them are only used
    internally by the application (such as the "APP_STATES" definition).  Both
    are defined here for convenience.
*******************************************************************************/

//DOM-IGNORE-BEGIN
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
//DOM-IGNORE-END


#ifndef _APP_MQTT_CLIENT_H
#define _APP_MQTT_CLIENT_H

// *****************************************************************************
// *****************************************************************************
// Section: Included Files
// *****************************************************************************
// *****************************************************************************

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdlib.h>
#include "system_config.h"
#include "system_definitions.h"

#include <wolfmqtt/mqtt_client.h>

// DOM-IGNORE-BEGIN
#ifdef __cplusplus  // Provide C++ Compatibility

extern "C" {

#endif
// DOM-IGNORE-END

    
// *****************************************************************************
// *****************************************************************************
// Section: Type Definitions
// *****************************************************************************
// *****************************************************************************

// *****************************************************************************
/* Application Codes */
enum AppCodes {
    APP_CODE_ERROR_BAD_ARG = -255,
    APP_CODE_ERROR_OUT_OF_BUFFER,
    APP_CODE_ERROR_SSL_FATAL,
    APP_CODE_ERROR_INVALID_SOCKET,
    APP_CODE_ERROR_FAILED_TO_BEGIN_DNS_RESOLUTION,
    APP_CODE_ERROR_DNS_FAILED,
    APP_CODE_ERROR_FAILED_SSL_NEGOTIATION,
    APP_CODE_ERROR_TIMEOUT,
    APP_CODE_ERROR_CMD_TIMEOUT,
    APP_CODE_SUCCESS = 0,
};


// *****************************************************************************
/* Application states

  Summary:
    Application states enumeration

  Description:
    This enumeration defines the valid application states.  These states
    determine the behavior of the application at various times.
*/
typedef enum
{
	/* Application's state machine's initial state. */
    APP_STATE_INIT = 0,

    APP_STATE_TCPIP_WAIT_INIT,
    APP_STATE_TCPIP_WAIT_FOR_IP,

    APP_STATE_MQTT_INIT,
    APP_STATE_MQTT_NET_CONNECT,
    APP_STATE_MQTT_PROTOCOL_CONNECT,
    APP_STATE_MQTT_SUBSCRIBE,
    APP_STATE_MQTT_LOOP,

    APP_TCPIP_ERROR,
    APP_FATAL_ERROR,

} APP_STATES;


// *****************************************************************************
/* Application Data

  Summary:
    Holds application data

  Description:
    This structure holds the application's data.

  Remarks:
    Application strings and buffers are be defined outside this structure.
 */
typedef struct
{
    /* The application's current state */
    APP_STATES state;

    // Timers
    uint32_t genericUseTimer;
    uint32_t mqttKeepAlive;
    uint32_t mqttUpdateStatus;

    /* TCPIP & MQTT */
    char macAddress[12 + 1];
    //__attribute__ ((aligned(4))) 
    char host[30];  // The endpoint to access the broker.
    IP_MULTI_ADDRESS host_ipv4;  // The endpoint IP address location.
    TCP_PORT         port;

    NET_PRES_SKT_HANDLE_T socket;
    NET_PRES_SKT_ERROR_T  error;

    MqttClient mqttClient;
    MqttNet    mqttNet;

    /* Debug Variables */
    bool socket_connected;
    bool mqtt_connected;
    IP_MULTI_ADDRESS board_ipAddr;

} APP_DATA;


// *****************************************************************************
/* Callback functions
 */
typedef void (*mqttclient_callback_t)(uint32_t address, const char *message);


// *****************************************************************************
// *****************************************************************************
// Section: Application Callback Routines
// *****************************************************************************
// *****************************************************************************

/* These routines are called by drivers when certain events occur.
*/


// *****************************************************************************
// *****************************************************************************
// Section: Application Initialization and State Machine Functions
// *****************************************************************************
// *****************************************************************************

/*******************************************************************************
  Function:
    void APP_MQTT_CLIENT_Initialize ( void )

  Summary:
     MPLAB Harmony application initialization routine.

  Description:
    This function initializes the Harmony application.  It places the 
    application in its initial state and prepares it to run so that its 
    APP_Tasks function can be called.

  Precondition:
    All other system initialization routines should be called before calling
    this routine (in "SYS_Initialize").

  Parameters:
    None.

  Returns:
    None.

  Example:
    <code>
    APP_MQTT_CLIENT_Initialize();
    </code>

  Remarks:
    This routine must be called from the SYS_Initialize function.
*/
void APP_MQTT_CLIENT_Initialize ( void );


/*******************************************************************************
  Function:
    void APP_MQTT_CLIENT_Tasks ( void )

  Summary:
    MPLAB Harmony Demo application tasks function

  Description:
    This routine is the Harmony Demo application's tasks function.  It
    defines the application's state machine and core logic.

  Precondition:
    The system and application initialization ("SYS_Initialize") should be
    called before calling this.

  Parameters:
    None.

  Returns:
    None.

  Example:
    <code>
    APP_MQTT_CLIENT_Tasks();
    </code>

  Remarks:
    This routine must be called from SYS_Tasks() routine.
 */
void APP_MQTT_CLIENT_Tasks( void );


// *****************************************************************************
// *****************************************************************************
// Section: Global Functions for other module
// *****************************************************************************
// *****************************************************************************

extern bool mqttclient_ready(void);
extern int  mqttclient_publish_register(uint32_t address, const char *message);  // Publish the update of register at address.
extern void mqttclient_set_callback(mqttclient_callback_t cb);  // Set the callback function for updating register as request.


#endif /* _APP_MQTT_CLIENT_H */

//DOM-IGNORE-BEGIN
#ifdef __cplusplus
}
#endif
//DOM-IGNORE-END

/*******************************************************************************
 End of File
 */

