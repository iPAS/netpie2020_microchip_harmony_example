/*******************************************************************************
  MPLAB Harmony Application Source File
  
  Company:
    Microchip Technology Inc.
  
  File Name:
    app2_tcp_client.c

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

#include "app2_tcp_client.h"

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

APP2_TCP_CLIENT_DATA app2_tcp_clientData;
static char app2_tcp_clientServerIP[] = "192.168.1.1";
static char app2_tcp_clientMsg[] = "Hello Server! .. from TCP client";
static uint8_t app2_tcp_clientResponseBuffer[80];

static SYS_STATUS          	app2_tcp_client_tcpipStat;
static TCPIP_NET_HANDLE    	app2_tcp_client_netH;
static int                 	app2_tcp_client_nNets;

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
    static void TCP_Client_TXRX_Task (void)
    
   Remarks:
    Feeds the USB write function. 
*/
static void TCP_Client_TXRX_Task (void)
{
	static IPV4_ADDR ipAddr;
	int i;

	switch (app2_tcp_clientData.txrxTaskState)
	{
        case APP2_TCP_CLIENT_TCPIP_WAIT_FOR_IP:
        {
            app2_tcp_client_nNets = TCPIP_STACK_NumberOfNetworksGet();

            for (i = 0; i < app2_tcp_client_nNets; i++)
            {
                app2_tcp_client_netH = TCPIP_STACK_IndexToNet(i);
                ipAddr.Val = TCPIP_STACK_NetAddress(app2_tcp_client_netH);
                if(TCPIP_STACK_NetIsReady(app2_tcp_client_netH))
                {
                    app2_tcp_clientData.txrxTaskState = APP2_TCP_CLIENT_TCPIP_WAITING_FOR_COMMAND;
                }
            }
            break;
        }
        case APP2_TCP_CLIENT_TCPIP_WAITING_FOR_COMMAND:
        {
            IPV4_ADDR addr;
            
            TCPIP_Helper_StringToIPAddress(app2_tcp_clientServerIP, &addr);
            app2_tcp_clientData.socket = TCPIP_TCP_ClientOpen(IP_ADDRESS_TYPE_IPV4,app2_tcp_clientData.port,
                                                    (IP_MULTI_ADDRESS*) &addr);
            
            if (app2_tcp_clientData.socket == INVALID_SOCKET)
            {
                app2_tcp_clientData.txrxTaskState = APP2_TCP_CLIENT_TCPIP_WAITING_FOR_COMMAND;
            }

            app2_tcp_clientData.txrxTaskState = APP2_TCP_CLIENT_TCPIP_WAIT_FOR_CONNECTION;
            break;
        }
        break;

        case APP2_TCP_CLIENT_TCPIP_WAIT_FOR_CONNECTION:
        {
            char buffer[256];
            if (!TCPIP_TCP_IsConnected(app2_tcp_clientData.socket))
            {
                break;
            }
            if(TCPIP_TCP_PutIsReady(app2_tcp_clientData.socket) == 0)
            {
                break;
            }

            sprintf(buffer, "Message: %s\r\n", app2_tcp_clientMsg);
            TCPIP_TCP_ArrayPut(app2_tcp_clientData.socket, (uint8_t*)buffer, strlen(buffer));
            app2_tcp_clientData.txrxTaskState = APP2_TCP_CLIENT_TCPIP_WAIT_FOR_RESPONSE;
        }
        break;

        case APP2_TCP_CLIENT_TCPIP_WAIT_FOR_RESPONSE:
        {
            if (!TCPIP_TCP_IsConnected(app2_tcp_clientData.socket))
            {
                app2_tcp_clientData.txrxTaskState = APP2_TCP_CLIENT_TCPIP_WAITING_FOR_COMMAND;
                break;
            }
            if (TCPIP_TCP_GetIsReady(app2_tcp_clientData.socket))
            {
                TCPIP_TCP_ArrayGet(app2_tcp_clientData.socket, app2_tcp_clientResponseBuffer, sizeof(app2_tcp_clientResponseBuffer) - 1);
            }
        }
        break;

        /* The default state should never be executed. */
        default:
        {
            /* TODO: Handle error in application's state machine. */
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
    void APP2_TCP_CLIENT_Initialize ( void )

  Remarks:
    See prototype in app2_tcp_client.h.
 */

void APP2_TCP_CLIENT_Initialize ( void )
{
    /* Place the App state machine in its initial state. */
    app2_tcp_clientData.state = APP2_TCP_CLIENT_STATE_INIT;

	app2_tcp_clientData.port = 1883;
    
    /* TODO: Initialize your application's state machine and other
     * parameters.
     */
}


/******************************************************************************
  Function:
    void APP2_TCP_CLIENT_Tasks ( void )

  Remarks:
    See prototype in app2_tcp_client.h.
 */

void APP2_TCP_CLIENT_Tasks ( void )
{
    int i;

    /* Check the application's current state. */
    switch ( app2_tcp_clientData.state )
    {
        /* Application's initial state. */
        case APP2_TCP_CLIENT_STATE_INIT:
        {
            bool appInitialized = true;
       
            app2_tcp_client_tcpipStat = TCPIP_STACK_Status(sysObj.tcpip);
            if(app2_tcp_client_tcpipStat < 0)
            {   // some error occurred
                app2_tcp_clientData.state = APP2_TCP_CLIENT_STATE_ERROR;
				appInitialized = false;
            }
            else if(app2_tcp_client_tcpipStat == SYS_STATUS_READY)
            {
                // now that the stack is ready we can check the
                // available interfaces
                app2_tcp_client_nNets = TCPIP_STACK_NumberOfNetworksGet();
                for(i = 0; i < app2_tcp_client_nNets; i++)
                {
                    app2_tcp_client_netH = TCPIP_STACK_IndexToNet(i);
                }
                app2_tcp_clientData.txrxTaskState = APP2_TCP_CLIENT_TCPIP_WAIT_FOR_IP;
            }

        
            if (appInitialized)
            {
            
                app2_tcp_clientData.state = APP2_TCP_CLIENT_STATE_SERVICE_TASKS;
            }
            break;
        }

        case APP2_TCP_CLIENT_STATE_SERVICE_TASKS:
        {
            TCP_Client_TXRX_Task();
        
            break;
        }

        /* TODO: implement your application state machine.*/
        
        case APP2_TCP_CLIENT_STATE_ERROR:
        {
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
