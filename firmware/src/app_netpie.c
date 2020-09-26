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
#include "parson/parson.h"

#if defined(DO_TRACE)
#include "app_uart_term.h"
#define TRACE_LOG(...) uart_send_tx_queue(__VA_ARGS__)
#elif defined(DO_LOG)
#include "app_logger.h"
#define TRACE_LOG(...) logger_send_tx_queue(__VA_ARGS__)
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

APP_NETPIE_DATA appNetpieData;

enum AppCodes {  /* Application Codes */
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
/* NETPIE2020 Configuration
*/
#include "netpie2020_config.h"

uint16_t packet_id = 0;

#define MAX_BUFFER_SIZE 1024
uint8_t txBuffer[MAX_BUFFER_SIZE];
uint8_t rxBuffer[MAX_BUFFER_SIZE];

#define MQTT_DEFAULT_CMD_TIMEOUT_MS 30000
#define MQTT_KEEP_ALIVE_TIMEOUT 900
#define MQTT_UPDATE_STATUS_TIMEOUT 15

// -- NEXPIE2020 --
// server: broker.netpie.io 1883 (mqtt)
const char mqtt_broker[] = "broker.netpie.io";
const TCP_PORT mqtt_port = MQTT_DEFAULT_PORT;

const char mqtt_client_id[] = NETPIE_CLIENT_ID;
const char mqtt_user[]      = NETPIE_TOKEN;
const char mqtt_password[]  = NETPIE_SECRET;

// -- MQTT topics for updating register --
#define MQTT_TOPIC_FILTER "@msg/" NETPIE_DEVICE_NAME "/#"
//const char mqtt_topic_status[]   = "@msg/" NETPIE_DEVICE_NAME "/status";  // Publish the device status
const char mqtt_topic_status[]   = "@shadow/data/update";                 // Publish the device status	
const char mqtt_topic_update[]   = "@msg/" NETPIE_DEVICE_NAME "/update";  // Get request for updating the register/%d
const char mqtt_topic_register[] = "@msg/" NETPIE_DEVICE_NAME "/register";  // Publish an update for register/%d
const char mqtt_topic_log[]      = "@msg/" NETPIE_DEVICE_NAME "/log";     // log

static netpie_callback_t subscription_callback = NULL;


// *****************************************************************************
// *****************************************************************************
// Section: Application Local Functions
// *****************************************************************************
// *****************************************************************************

static bool APP_timerExpired(uint32_t * timer, uint32_t seconds)
{
    return ((SYS_TMR_TickCountGet() - *timer) > (seconds * 1000))? true : false;
}


static bool APP_timerExpired_ms(uint32_t * timer, uint32_t mseconds)
{
    return ((SYS_TMR_TickCountGet() - *timer) > (mseconds))? true : false;
}


static bool APP_timerSet(uint32_t * timer)
{
    *timer = SYS_TMR_TickCountGet();
    return true;
}


int netpie_publish(const char *topic, const char *buf, uint16_t pkg_id)
{
    MqttPublish publish;
    XMEMSET(&publish, 0, sizeof(MqttPublish));
    publish.retain      = 0;
    publish.qos         = 0;
    publish.duplicate   = 0;
    publish.topic_name  = topic;
    publish.packet_id   = pkg_id;
    publish.buffer      = (byte *)buf;
    publish.total_len   = strlen(buf);

    TRACE_LOG("[%d] #%d publish topic:'%s' msg:'%s'\n\r", __LINE__, pkg_id, topic, buf);
    
    return MqttClient_Publish(&appNetpieData.mqttClient, &publish);
}


int netpie_publish_status(void)
{
    // Example: https://github.com/kgabis/parson/blob/master/tests.c#L348

    char buf[MAX_BUFFER_SIZE];

    /* Make JSON object */
    JSON_Status sts;
    JSON_Value  *root_value  = json_value_init_object();
    JSON_Object *root_object = json_value_get_object(root_value);
    
    JSON_Value  *data_value  = json_value_init_object();
    JSON_Object *data_object = json_value_get_object(data_value);
    json_object_set_value(root_object, "data", data_value);
    
    json_object_set_string(data_object, "name", NETPIE_DEVICE_NAME);
    json_object_set_number(data_object, "temperature", rand() % 250);
    json_object_set_number(data_object, "humidity", rand() % 100);
    char ip[15];
    IPV4_ADDR ipAddr; ipAddr.Val = appNetpieData.board_ipAddr.v4Add.Val;
    sprintf(ip, "%d.%d.%d.%d", ipAddr.v[0], ipAddr.v[1], ipAddr.v[2], ipAddr.v[3]);
    json_object_set_string(data_object, "ip_address", ip);
    
    
    /* Transform the object to string */
    char *serialized_string = NULL;
    serialized_string = json_serialize_to_string(root_value);
    strncpy(buf, serialized_string, sizeof(buf));

    json_free_serialized_string(serialized_string);
    json_value_free(root_value);

    return netpie_publish(mqtt_topic_status, buf, packet_id++);
}


// *****************************************************************************
// *****************************************************************************
// Section: Application Callback Functions
// *****************************************************************************
// *****************************************************************************

/* WolfMQTT Callbacks for network connectivity 
 */

int APP_tcpipConnect_cb(void *context, const char* host, word16 port, int timeout_ms)
{
    TRACE_LOG("[%d] APP_tcpipConnect_cb() get started\n\r", __LINE__);  // DEBUG: iPAS

    uint32_t timeout;
    timeout = SYS_TMR_TickCountGet();

    appNetpieData.socket_connected = false;

    TCPIP_DNS_RESULT dnsResult = TCPIP_DNS_Resolve((const char *)appNetpieData.host, TCPIP_DNS_TYPE_A);
    if(dnsResult < 0)
    {
        TRACE_LOG("[%d] TCPIP_DNS_Resolve() fail\n\r", __LINE__);  // DEBUG: iPAS
        return APP_CODE_ERROR_FAILED_TO_BEGIN_DNS_RESOLUTION;  // DNS resolving problem
    }

    while((dnsResult = TCPIP_DNS_IsResolved((const char *)appNetpieData.host, &appNetpieData.host_ipv4, IP_ADDRESS_TYPE_IPV4)
          ) == TCPIP_DNS_RES_PENDING)
    {
        if(APP_timerExpired_ms(&timeout, timeout_ms))
        {
            TRACE_LOG("[%d] TCPIP_DNS_IsResolve() timeout\n\r", __LINE__);  // DEBUG: iPAS
            return APP_CODE_ERROR_CMD_TIMEOUT;
        }
    }
    if(dnsResult != (TCPIP_DNS_RES_OK))
    {
        TRACE_LOG("[%d] TCPIP_DNS_IsResolve() fail\n\r", __LINE__);  // DEBUG: iPAS
        return APP_CODE_ERROR_DNS_FAILED;
    }

    appNetpieData.socket = NET_PRES_SocketOpen(0, NET_PRES_SKT_UNENCRYPTED_STREAM_CLIENT, IP_ADDRESS_TYPE_IPV4,
                                         (NET_PRES_SKT_PORT_T)port,
                                         (NET_PRES_ADDRESS *)&appNetpieData.host_ipv4,
                                         (NET_PRES_SKT_ERROR_T*)&appNetpieData.error);
    NET_PRES_SocketWasReset(appNetpieData.socket);

    if (appNetpieData.socket == INVALID_SOCKET)
    {
        TRACE_LOG("[%d] NET_PRES_SocketOpen() socket invalid\n\r", __LINE__);  // DEBUG: iPAS

        NET_PRES_SocketClose(appNetpieData.socket);
        return APP_CODE_ERROR_INVALID_SOCKET;
    }

    while (!NET_PRES_SKT_IsConnected(appNetpieData.socket))
    {
        if (APP_timerExpired_ms(&timeout, timeout_ms))
        {
            TRACE_LOG("[%d] NET_PRES_SKT_IsConnected() timeout\n\r", __LINE__);  // DEBUG: iPAS
            return APP_CODE_ERROR_CMD_TIMEOUT;
        }
    }

    while (NET_PRES_SKT_IsNegotiatingEncryption(appNetpieData.socket))
    {
        if (APP_timerExpired_ms(&timeout, timeout_ms))
        {
            TRACE_LOG("[%d] NET_PRES_SKT_IsNegotiatingEncryption() timeout\n\r", __LINE__);  // DEBUG: iPAS
            return APP_CODE_ERROR_CMD_TIMEOUT;
        }
    }

    // if (!NET_PRES_SKT_IsSecure(appData.socket))
    // {
    //     TRACE_LOG("[%d] NET_PRES_SKT_IsSecure() fail\n\r", __LINE__);  // DEBUG: iPAS
    //     NET_PRES_SocketClose(appData.socket);
    //     return APP_CODE_ERROR_FAILED_SSL_NEGOTIATION;
    // }

    appNetpieData.socket_connected = true;
    return 0;  //Success
}


/* TCP Callbacks for network connectivity 
 */

int APP_tcpipDisconnect_cb(void *context)
{
    int ret = 0;
    appNetpieData.socket_connected = false;
    NET_PRES_SKT_Close(appNetpieData.socket);
    appNetpieData.state = APP_NETPIE_STATE_MQTT_NET_CONNECT;
    return ret;
}


int APP_tcpipRead_cb(void *context, byte* buf, int buf_len, int timeout_ms)
{
    int ret = 0;
    uint32_t timeout;

    APP_timerSet(&timeout);
    // Wait for data to be read, or error, or timeout
    while (NET_PRES_SocketReadIsReady(appNetpieData.socket) == 0)
    {
        if (NET_PRES_SocketWasReset(appNetpieData.socket))
        {
            ret = APP_CODE_ERROR_SSL_FATAL;
            return ret;
        }
        if (APP_timerExpired_ms(&timeout, (uint32_t)timeout_ms))
        {
            ret = APP_CODE_ERROR_CMD_TIMEOUT;
            return ret;
        }
    }

    ret = NET_PRES_SocketRead(appNetpieData.socket, (uint8_t*)buf, buf_len);
    return ret;
}


int APP_tcpipWrite_cb(void *context, const byte* buf, int buf_len, int timeout_ms)
{
    int ret = 0;
    uint32_t timeout;

    APP_timerSet(&timeout);
    // Wait for data to be read, or error, or timeout
    while (NET_PRES_SocketWriteIsReady(appNetpieData.socket, buf_len, 0) == 0)
    {
        if (NET_PRES_SocketWasReset(appNetpieData.socket))
        {
            ret = APP_CODE_ERROR_SSL_FATAL;
            return ret;
        }
        if (APP_timerExpired_ms(&timeout, (uint32_t)timeout_ms))
        {
            ret = APP_CODE_ERROR_CMD_TIMEOUT;
            return ret;
        }
    }

    ret = NET_PRES_SocketWrite(appNetpieData.socket, (uint8_t*)buf, buf_len);
    return ret;
}


int APP_mqttMessage_cb(MqttClient *client, MqttMessage *msg, byte msg_new, byte msg_done)
{
    char *buf = (char *)malloc(msg->total_len + msg->topic_name_len + 2);
    
    char *message, *topic;
    message = buf;
    topic   = (char *)&buf[msg->total_len + 1];

    memcpy(message, msg->buffer, msg->total_len);
    message[msg->total_len] = '\0';  // Requite for using as string

    memcpy(topic, msg->topic_name, msg->topic_name_len);
    topic[msg->topic_name_len] = '\0';  // Requite for using as string


    TRACE_LOG("--- received #%d from topic:%s msg:'%s'\n\r", msg->packet_id, topic, message);  // DEBUG: iPAS
    

    /**
     * Process the request for updating 
     */
    if (strncmp(mqtt_topic_update, topic, sizeof(mqtt_topic_update)-1) == 0)
    {
        if (subscription_callback != NULL)  // If the callback was set, then
        {
            char *sub_topic = &topic[ sizeof(mqtt_topic_update) ];  // .../update/<sub_topic> with 'message'
            subscription_callback(sub_topic, message);
        }
        else
        {
            TRACE_LOG("[%d] A subscribed message is received ,but , subscription_callback is NULL!\n\r", __LINE__);
        }
    }
    
    free(buf);
    return 0;
}


// *****************************************************************************
// *****************************************************************************
// Section: Global Functions
// *****************************************************************************
// *****************************************************************************

/**
 * Check the status of the connections: TCP & MQTT
 * @return 
 */
bool netpie_ready(void)
{
    return appNetpieData.socket_connected && appNetpieData.mqtt_connected;
}


/**
 * Publish log message
 * @param message
 * @return 
 */
int netpie_publish_log(const char *message)
{
    return netpie_publish(mqtt_topic_log, message, packet_id++);
}


/**
 * Publish the update of register at address.
 * @param address
 * @param message
 * @return 
 */
int netpie_publish_register(const char *sub_topic, const char *message)
{
    int rc;
    
    char *topic = (char *)malloc(sizeof(mqtt_topic_register) + 1 + strlen(sub_topic) + 1);
    sprintf(topic, "%s/%s", mqtt_topic_register, sub_topic);

    rc = netpie_publish(topic, message, packet_id++);
    if (rc != MQTT_CODE_SUCCESS)
    {
        appNetpieData.state = APP_NETPIE_STATE_TCPIP_ERROR;
    }
    free(topic);
    return rc;
}


/**
 * Set the callback function for updating register as request.
 * @param cb
 */
void netpie_set_callback(netpie_callback_t cb)
{
    subscription_callback = cb;
}


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
    appNetpieData.state = APP_NETPIE_STATE_INIT;

    /* Initialize MQTT */
    sprintf(appNetpieData.macAddress, "Null");
    strncpy(appNetpieData.host, mqtt_broker, sizeof(appNetpieData.host));
    appNetpieData.port = mqtt_port;

    appNetpieData.mqttNet.connect    = APP_tcpipConnect_cb;
    appNetpieData.mqttNet.disconnect = APP_tcpipDisconnect_cb;
    appNetpieData.mqttNet.read       = APP_tcpipRead_cb;
    appNetpieData.mqttNet.write      = APP_tcpipWrite_cb;

    /* ETC. */
    appNetpieData.socket_connected   = false;
    appNetpieData.mqtt_connected     = false;
}


/******************************************************************************
  Function:
    void APP_NETPIE_Tasks ( void )

  Remarks:
    See prototype in app_netpie.h.
 */
void APP_NETPIE_Tasks ( void )
{
    /* Check the application's current state. */
    switch ( appNetpieData.state )
    {
        /* Application's initial state. */
        case APP_NETPIE_STATE_INIT:
        {
            vTaskDelay(3000 / portTICK_PERIOD_MS);
            appNetpieData.state = APP_NETPIE_STATE_TCPIP_WAIT_INIT;
            
            TRACE_LOG("\n\r--- APP MQTT Client Init ---\n\r");  // DEBUG: iPAS
            break;
        }

        case APP_NETPIE_STATE_TCPIP_WAIT_INIT:
        {
            TCPIP_NET_HANDLE    netH;
            int                 i, nNets;

            SYS_STATUS tcpipStatus = TCPIP_STACK_Status(sysObj.tcpip);
            if (tcpipStatus < 0) 
            {   // some error occurred
                break;
            }
            else
            if (tcpipStatus == SYS_STATUS_READY)
            {
                // Now the stack is ready, we can check the available interfaces
                nNets = TCPIP_STACK_NumberOfNetworksGet();
                for (i = 0; i < nNets; i++) 
                {
                    netH = TCPIP_STACK_IndexToNet(i);
                    TCPIP_STACK_NetNameGet(netH);
                    TCPIP_STACK_NetBIOSName(netH);

                    // Retrieve MAC Address to store and convert to string
                    TCPIP_MAC_ADDR* pAddr = (TCPIP_MAC_ADDR *)TCPIP_STACK_NetAddressMac(netH);
                    sprintf(appNetpieData.macAddress, "%02x%02x%02x%02x%02x%02x",
                            pAddr->v[0], pAddr->v[1], pAddr->v[2], pAddr->v[3], pAddr->v[4], pAddr->v[5]);
                    
                    // Show interface's MAC
                    char ifname[10];
                    ifname[ TCPIP_STACK_NetAliasNameGet(netH, ifname, sizeof(ifname)) ] = '\0';
                    TRACE_LOG("[%d] '%s' MAC: %s\n\r", __LINE__, ifname, appNetpieData.macAddress);  // DEBUG: iPAS
                }

                APP_timerSet(&appNetpieData.genericUseTimer);
                appNetpieData.state = APP_NETPIE_STATE_TCPIP_WAIT_FOR_IP;
            }
            break;
        }

        case APP_NETPIE_STATE_TCPIP_WAIT_FOR_IP:
        {
            TCPIP_NET_HANDLE    netH;
            int                 i, nNets;

            if (APP_timerExpired(&appNetpieData.genericUseTimer, 10))
            {
                APP_timerSet(&appNetpieData.genericUseTimer);
            }

            nNets = TCPIP_STACK_NumberOfNetworksGet();
            for (i = 0; i < nNets; i++)
            {
                netH = TCPIP_STACK_IndexToNet(i);

                IPV4_ADDR ipAddr;
                ipAddr.Val = TCPIP_STACK_NetAddress(netH);
                if (ipAddr.Val != 0)
                {
                    if (ipAddr.v[0] != 0 && ipAddr.v[0] != 169) // Wait for a Valid IP
                    {
                        char ifname[10];
                        ifname[ TCPIP_STACK_NetAliasNameGet(netH, ifname, sizeof(ifname)) ] = '\0';
                        TRACE_LOG("[%d] '%s' IP: %d.%d.%d.%d\n\r", __LINE__, ifname,
                            ipAddr.v[0], ipAddr.v[1], ipAddr.v[2], ipAddr.v[3]);  // DEBUG: iPAS

                        //const char *ifnames[] = {"PIC32INT", "MRF24W"};
                        //const char *ifnames[] = {"eth0", "wlan0"};
                        if (strcmp(ifname, "wlan0") == 0)  // Waiting for the WiFi interface to get DHCP done
                        {
                            appNetpieData.board_ipAddr.v4Add.Val = ipAddr.Val;  // saved for debugging
                            appNetpieData.state = APP_NETPIE_STATE_MQTT_INIT;
                        }
                        else
                        {
                            vTaskDelay(1000 / portTICK_PERIOD_MS);
                        }
                    }
                }
            }
            break;
        }

        case APP_NETPIE_STATE_MQTT_INIT:
        {
            int rc = MqttClient_Init(&appNetpieData.mqttClient,
                                     &appNetpieData.mqttNet,
                                     APP_mqttMessage_cb,
                                     txBuffer, MAX_BUFFER_SIZE,
                                     rxBuffer, MAX_BUFFER_SIZE,
                                     MQTT_DEFAULT_CMD_TIMEOUT_MS);
            if (rc != MQTT_CODE_SUCCESS)
            {
                appNetpieData.state = APP_NETPIE_STATE_FATAL_ERROR;
                break;
            }
            APP_timerSet(&appNetpieData.genericUseTimer);
            appNetpieData.state = APP_NETPIE_STATE_MQTT_NET_CONNECT;

            TRACE_LOG("[%d] MQTT init ready\n\r", __LINE__);  // DEBUG: iPAS
            break;
        }

        case APP_NETPIE_STATE_MQTT_NET_CONNECT:
        {
            int rc = MqttClient_NetConnect(&appNetpieData.mqttClient,
                                           (const char *)appNetpieData.host, appNetpieData.port,
                                           MQTT_DEFAULT_CMD_TIMEOUT_MS,
                                           NULL,    // TLS disable
                                           NULL);   // TLS callback

            //
            // DEBUG: 'rc' is still 'MQTT_CODE_SUCCESS', even if the MQTT broker has been shutdown.
            //

            if (rc != MQTT_CODE_SUCCESS)
            {
                appNetpieData.socket_connected = appNetpieData.mqtt_connected = false;
                NET_PRES_SocketClose(appNetpieData.socket);

                while (!APP_timerExpired(&appNetpieData.genericUseTimer, 5));  // Delay 5 seconds before trying next
                APP_timerSet(&appNetpieData.genericUseTimer);

                TRACE_LOG("[%d] MQTT connect network %s fail (rc=%d)\n\r", __LINE__, appNetpieData.host, rc);  // DEBUG: iPAS
                break;
            }
            appNetpieData.state = APP_NETPIE_STATE_MQTT_PROTOCOL_CONNECT;

            TRACE_LOG("[%d] MQTT network ready\n\r", __LINE__);  // DEBUG: iPAS
            break;
        }

        case APP_NETPIE_STATE_MQTT_PROTOCOL_CONNECT:
        {
            MqttConnect connect;
            XMEMSET(&connect, 0, sizeof(connect));
            connect.keep_alive_sec = MQTT_KEEP_ALIVE_TIMEOUT;
            connect.clean_session = 1;

            // connect.client_id   = appData.macAddress;
            connect.client_id   = mqtt_client_id;
            connect.username    = mqtt_user;
            connect.password    = mqtt_password;

            MqttMessage lwt_msg;  // Last-will and testament message
            XMEMSET(&lwt_msg, 0, sizeof(lwt_msg));
            connect.lwt_msg     = &lwt_msg;
            connect.enable_lwt  = 0;

            /* Send Connect and wait for Connect Ack */
            int rc = MqttClient_Connect(&appNetpieData.mqttClient, &connect);
            if(rc != MQTT_CODE_SUCCESS)
            {
                APP_timerSet(&appNetpieData.genericUseTimer);
                while (!APP_timerExpired(&appNetpieData.genericUseTimer, 5));

                appNetpieData.state = APP_NETPIE_STATE_TCPIP_ERROR;

                TRACE_LOG("[%d] MQTT protocol negotiation fail (rc=%d)\n\r", __LINE__, rc);  // DEBUG: iPAS
                break;
            }
            appNetpieData.mqtt_connected = true;
            APP_timerSet(&appNetpieData.mqttKeepAlive);
            APP_timerSet(&appNetpieData.mqttUpdateStatus);
            appNetpieData.state = APP_NETPIE_STATE_MQTT_SUBSCRIBE;

            TRACE_LOG("[%d] MQTT protocol negotiation success\n\r", __LINE__);  // DEBUG: iPAS
            break;
        }

        case APP_NETPIE_STATE_MQTT_SUBSCRIBE:
        {
            MqttSubscribe subscribe;
            MqttTopic topics[1];
            MqttPublish publish;
            int rc;

            /* Build list of topics */
            topics[0].topic_filter = MQTT_TOPIC_FILTER;
            topics[0].qos = 0;

            /* Subscribe Topic */
            XMEMSET(&subscribe, 0, sizeof(MqttSubscribe));
            subscribe.packet_id = packet_id++;
            subscribe.topic_count = sizeof(topics)/sizeof(MqttTopic);
            subscribe.topics = topics;

            rc = MqttClient_Subscribe(&appNetpieData.mqttClient, &subscribe);
            if (rc != MQTT_CODE_SUCCESS)
            {
                APP_timerSet(&appNetpieData.genericUseTimer);
                while (!APP_timerExpired(&appNetpieData.genericUseTimer, 5));

                appNetpieData.state = APP_NETPIE_STATE_TCPIP_ERROR;

                TRACE_LOG("[%d] MQTT subscription fail (rc=%d)\n\r", __LINE__, rc);  // DEBUG: iPAS
                break;
            }

            appNetpieData.state = APP_NETPIE_STATE_MQTT_LOOP;

            TRACE_LOG("[%d] MQTT subscribe '%s'\n\r", __LINE__, topics[0].topic_filter);  // DEBUG: iPAS
            break;
        }

        case APP_NETPIE_STATE_MQTT_LOOP:
        {
            int rc = 0;

            /* Keep alive */
            if (APP_timerExpired(&appNetpieData.mqttKeepAlive, MQTT_KEEP_ALIVE_TIMEOUT))
            {
                rc = MqttClient_Ping(&appNetpieData.mqttClient);
                if (rc != MQTT_CODE_SUCCESS)
                {
                    appNetpieData.state = APP_NETPIE_STATE_TCPIP_ERROR;
                }

                APP_timerSet(&appNetpieData.mqttKeepAlive);  // Reset keep alive timer
                break;
            }
            
            /* Update status */
            if (APP_timerExpired(&appNetpieData.mqttUpdateStatus, MQTT_UPDATE_STATUS_TIMEOUT))
            {
                TRACE_LOG("[%d] Update status every %d s\n\r", __LINE__, MQTT_UPDATE_STATUS_TIMEOUT);  // DEBUG: iPAS

                rc = netpie_publish_status();
                if (rc != MQTT_CODE_SUCCESS)
                {
                    appNetpieData.state = APP_NETPIE_STATE_TCPIP_ERROR;
                }
                
                APP_timerSet(&appNetpieData.mqttUpdateStatus);  // Reset status update timer
                
                APP_timerSet(&appNetpieData.mqttKeepAlive);  // Reset keep alive timer since we sent a publish
                break;
            }

            /* Check for incoming messages */
            rc = MqttClient_WaitMessage(&appNetpieData.mqttClient, MQTT_DEFAULT_CMD_TIMEOUT_MS);
            if (rc == MQTT_CODE_ERROR_TIMEOUT)
            {
                /* Keep Alive */
                rc = MqttClient_Ping(&appNetpieData.mqttClient);
                if (rc != MQTT_CODE_SUCCESS)
                {
                    appNetpieData.state = APP_NETPIE_STATE_TCPIP_ERROR;
                    break;
                }
                
                APP_timerSet(&appNetpieData.mqttKeepAlive);  // Reset keep alive timer since we sent a publish
            }
            else
            if (rc == MQTT_CODE_ERROR_NETWORK)
            {
                appNetpieData.state = APP_NETPIE_STATE_TCPIP_ERROR;
                break;
            }
            else
            if (rc == APP_CODE_ERROR_CMD_TIMEOUT)  // No any message within DEFAULT_CMD_TIMEOUT_MS, then speak!
            {                                      // XXX: maybe returned from WolfMQTT callback functions
                TRACE_LOG("[%d] No any message within %d ms (APP_CODE_ERROR_CMD_TIMEOUT)\n\r", __LINE__, MQTT_DEFAULT_CMD_TIMEOUT_MS);  // DEBUG: iPAS

                rc = netpie_publish_log("Waiting too long without message in");
                if (rc != MQTT_CODE_SUCCESS)
                {
                    appNetpieData.state = APP_NETPIE_STATE_TCPIP_ERROR;
                    break;
                }

                APP_timerSet(&appNetpieData.mqttKeepAlive);  // Reset keep alive timer since we sent a publish
            }
            else
            if (rc != MQTT_CODE_SUCCESS)
            {
                appNetpieData.state = APP_NETPIE_STATE_TCPIP_ERROR;
                break;
            }

            break;
        }

        /* Connection lost */
        case APP_NETPIE_STATE_TCPIP_ERROR:
        {
            appNetpieData.socket_connected = appNetpieData.mqtt_connected = false;
            NET_PRES_SocketClose(appNetpieData.socket);
            appNetpieData.state = APP_NETPIE_STATE_MQTT_NET_CONNECT;

            TRACE_LOG("[%d] APP_TCPIP_ERROR, going to APP_STATE_MQTT_NET_CONNECT\n\r", __LINE__);  // DEBUG: iPAS
            break;
        }

        /* This error requires system reset or hardware debugging */
        case APP_NETPIE_STATE_FATAL_ERROR:
        {
            TRACE_LOG("[%d] APP_FATAL_ERROR\n\r", __LINE__);  // DEBUG: iPAS
            break;
        }

        /* The default state should never be executed. */
        default:
        {
            TRACE_LOG("--- APP MQTT Client lost its state machine ---\n\r");  // DEBUG: iPAS
            break;
        }
    }
}


/*******************************************************************************
 End of File
 */
