/*******************************************************************************
  MPLAB Harmony Application Source File
  
  Company:
    Microchip Technology Inc.
  
  File Name:
    app_netpie.c

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

#include "app_netpie.h"

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

APP_NETPIE_DATA app_netpieData;
static char app_netpieMsgToClient[] = "Hello Client! .. from your TCP server\n\r";
static uint8_t app_netpieMsgFromClient[80];

static TCPIP_NET_HANDLE    	app_netpie_netH;
static SYS_STATUS          	app_netpie_tcpipStat;
static int                 	app_netpie_nNets;

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
    static void TCP_Server_TXRX_Task (void)
    
   Remarks:
    Feeds the USB write function. 
*/
static void TCP_Server_TXRX_Task (void)
{
	static IPV4_ADDR    		dwLastIP[2] = { {-1}, {-1} };
	static IPV4_ADDR           	ipAddr;
	int                 		i;

	switch (app_netpieData.txrxTaskState)
	{
        case APP_NETPIE_TCPIP_WAIT_FOR_IP:
        {
            app_netpie_nNets = TCPIP_STACK_NumberOfNetworksGet();

            for (i = 0; i < app_netpie_nNets; i++)
            {
                app_netpie_netH = TCPIP_STACK_IndexToNet(i);
                ipAddr.Val = TCPIP_STACK_NetAddress(app_netpie_netH);
                if (TCPIP_STACK_NetIsReady(app_netpie_netH))
                {
                    app_netpieData.txrxTaskState = APP_NETPIE_TCPIP_OPENING_SERVER;
                }
            }
            break;
        }
        case APP_NETPIE_TCPIP_OPENING_SERVER:
        {
            app_netpieData.socket = TCPIP_TCP_ServerOpen(IP_ADDRESS_TYPE_IPV4, app_netpieData.port, 0);
            if (app_netpieData.socket == INVALID_SOCKET)
            {
                break;
            }
            app_netpieData.txrxTaskState = APP_NETPIE_TCPIP_WAIT_FOR_CONNECTION;;
        }
        break;

        case APP_NETPIE_TCPIP_WAIT_FOR_CONNECTION:
        {
            if (!TCPIP_TCP_IsConnected(app_netpieData.socket))
            {
                break;
            }
            else
            {
                // We got a connection
				TCPIP_TCP_ArrayPut(app_netpieData.socket, app_netpieMsgToClient, sizeof(app_netpieMsgToClient));
                app_netpieData.txrxTaskState = APP_NETPIE_TCPIP_WAIT_FOR_RESPONSE;
            }
        }
        break;

        case APP_NETPIE_TCPIP_WAIT_FOR_RESPONSE:
        {
            if (!TCPIP_TCP_IsConnected(app_netpieData.socket))
            {
                app_netpieData.txrxTaskState = APP_NETPIE_TCPIP_WAIT_FOR_IP;
                break;
            }
            if (TCPIP_TCP_GetIsReady(app_netpieData.socket))
            {
                TCPIP_TCP_ArrayGet(app_netpieData.socket, app_netpieMsgFromClient, sizeof(app_netpieMsgFromClient) - 1);
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
    void APP_NETPIE_Initialize ( void )

  Remarks:
    See prototype in app_netpie.h.
 */

void APP_NETPIE_Initialize ( void )
{
    /* Place the App state machine in its initial state. */
    app_netpieData.state = APP_NETPIE_STATE_INIT;

	app_netpieData.port = 8000;
    
    /* TODO: Initialize your application's state machine and other
     * parameters.
     */
}


/******************************************************************************
  Function:
    void APP_NETPIE_Tasks ( void )

  Remarks:
    See prototype in app_netpie.h.
 */

void APP_NETPIE_Tasks ( void )
{
    int i;

    /* Check the application's current state. */
    switch ( app_netpieData.state )
    {
        /* Application's initial state. */
        case APP_NETPIE_STATE_INIT:
        {
            bool appInitialized = true;
       
            app_netpie_tcpipStat = TCPIP_STACK_Status(sysObj.tcpip);
            if(app_netpie_tcpipStat < 0)
            {   // some error occurred
                app_netpieData.state = APP_NETPIE_STATE_ERROR;
				appInitialized = false;
            }
            else if(app_netpie_tcpipStat == SYS_STATUS_READY)
            {
                // now that the stack is ready we can check the
                // available interfaces
                app_netpie_nNets = TCPIP_STACK_NumberOfNetworksGet();
                for(i = 0; i < app_netpie_nNets; i++)
                {
                    app_netpie_netH = TCPIP_STACK_IndexToNet(i);
                }
                app_netpieData.txrxTaskState = APP_NETPIE_TCPIP_WAIT_FOR_IP;
            }
        
            if (appInitialized)
            {
            
                app_netpieData.state = APP_NETPIE_STATE_SERVICE_TASKS;
            }
            break;
        }

        case APP_NETPIE_STATE_SERVICE_TASKS:
        {
            TCP_Server_TXRX_Task();
        
            break;
        }

        /* TODO: implement your application state machine.*/
        
        case APP_NETPIE_STATE_ERROR:
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
