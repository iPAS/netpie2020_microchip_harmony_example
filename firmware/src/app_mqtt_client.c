/*******************************************************************************
  MPLAB Harmony Application Source File
  
  Company:
    Microchip Technology Inc.
  
  File Name:
    app_mqtt_client.c

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

#include "app_mqtt_client.h"
#include "app_uart_term.h"

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

APP_DATA appData;

static bool APP_TIMER_Expired(uint32_t * timer, uint32_t seconds);
static bool APP_TIMER_Expired_ms(uint32_t * timer, uint32_t mseconds);
static bool APP_TIMER_Set(uint32_t * timer);
static const char *basename(const char *name);
static int mqttclient_publish(const char *topic, const char *buf, uint16_t pkg_id);
static int mqttclient_publish_number(const char *topic);
static int mqttclient_publish_json(const char *topic, uint16_t pkg_id);
static int mqttclient_publish_feed(const char *topic, uint16_t pkg_id);
static int mqttclient_publish_shadow(uint16_t pkg_id);


static uint16_t packet_id = 0;

#define MAX_BUFFER_SIZE 1024
static byte txBuffer[MAX_BUFFER_SIZE];
static byte rxBuffer[MAX_BUFFER_SIZE];

#define MQTT_DEFAULT_CMD_TIMEOUT_MS 30000
#define MQTT_KEEP_ALIVE 900

#define BROKER_SELECTION 2

#if BROKER_SELECTION == 0
// -- Mosquitto --
// local MQTT broker for testing
const char mqtt_broker[] = "192.168.1.1";
const TCP_PORT mqtt_port = MQTT_DEFAULT_PORT;

const char mqtt_client_id[] = "iPAS";
const char mqtt_user[]      = "ipas";
const char mqtt_password[]  = "***REMOVED***";

const char mqtt_topic_subscribed[]  = "experiment/testSubscribe";
const char mqtt_topic_published[]   = "experiment/testPublish";
const char mqtt_topic_number[]      = "experiment/random_number";
const char mqtt_topic_json[]        = "experiment/json";

#define MQTT_TOPIC_FILTER "#"

#elif BROKER_SELECTION == 1
// -- NETPIE beta --
// server:  gb.netpie.io 1883
// app:     experiment
// device:  uRTU
// key:     ***REMOVED***
// secret:  ***REMOVED***
const char mqtt_broker[] = "gb.netpie.io";
const TCP_PORT mqtt_port = MQTT_DEFAULT_PORT;

const char mqtt_client_id[] = "***REMOVED***";
const char mqtt_user[]      = "***REMOVED***";
const char mqtt_password[]  = "***REMOVED***";

const char mqtt_topic_subscribed[]  = "gearname/uRTU/testSubscribe";  // Cause of Microgear
                                                                      // microgear["ds_testSubscribe"].chat("uRTU/testSubscribe", "Hello")
const char mqtt_topic_published[]   = "uRTU/testPublish";  // on Freeboard: "experiment/uRTU/..."
                                                           // datasources["ds_testPublish"]["/experiment/uRTU/testPublish"]
const char mqtt_topic_number[]      = "uRTU/random_number";
const char mqtt_topic_json[]        = "uRTU/json";

#define FEED_ID "experiment"
const char feed_id[]         = FEED_ID;
const char mqtt_topic_feed[] = "@writefeed/" FEED_ID;

#define MQTT_TOPIC_FILTER "#"

#elif BROKER_SELECTION == 2
// -- NEXPIE (next NETPIE generation) --
// server:      broker.netpie.io 1883 (mqtt)
// device:      uRTU
const char mqtt_broker[] = "broker.netpie.io";
const TCP_PORT mqtt_port = MQTT_DEFAULT_PORT;

const char mqtt_client_id[] = "***REMOVED***";
const char mqtt_user[]      = "***REMOVED***";
const char mqtt_password[]  = "***REMOVED***";

const char mqtt_topic_subscribed[]  = "@msg/testSubscribe";
const char mqtt_topic_published[]   = "@msg/testPublish";
const char mqtt_topic_number[]      = "@msg/random_number";
const char mqtt_topic_json[]        = "@msg/json";

#define MQTT_TOPIC_FILTER "@msg/#"

#else
#error "Please select the broker to be connected!"
#endif


// *****************************************************************************
// *****************************************************************************
// Section: Application Callback Functions
// *****************************************************************************
// *****************************************************************************

/* WolfMQTT Callbacks for network connectivity */

int APP_tcpipConnect_cb(void *context, const char* host, word16 port, int timeout_ms)
{
    uart_send_tx_queue("[%s] APP_tcpipConnect_cb() get started\n\r", basename(__FILE__));  // DEBUG: iPAS

    uint32_t timeout;
    timeout = SYS_TMR_TickCountGet();

    appData.socket_connected = false;

    TCPIP_DNS_RESULT dnsResult = TCPIP_DNS_Resolve((const char *)appData.host, TCPIP_DNS_TYPE_A);
    if(dnsResult < 0)
    {
        uart_send_tx_queue("[%s] TCPIP_DNS_Resolve() fail\n\r", basename(__FILE__));  // DEBUG: iPAS
        return APP_CODE_ERROR_FAILED_TO_BEGIN_DNS_RESOLUTION;  // DNS resolving problem
    }

    while((dnsResult = TCPIP_DNS_IsResolved((const char *)appData.host, &appData.host_ipv4, IP_ADDRESS_TYPE_IPV4)
          ) == TCPIP_DNS_RES_PENDING)
    {
        if(APP_TIMER_Expired_ms(&timeout, timeout_ms))
        {
            uart_send_tx_queue("[%s] TCPIP_DNS_IsResolve() timeout\n\r", basename(__FILE__));  // DEBUG: iPAS
            return APP_CODE_ERROR_CMD_TIMEOUT;
        }
    }
    if(dnsResult != (TCPIP_DNS_RES_OK))
    {
        uart_send_tx_queue("[%s] TCPIP_DNS_IsResolve() fail\n\r", basename(__FILE__));  // DEBUG: iPAS
        return APP_CODE_ERROR_DNS_FAILED;
    }

    appData.socket = NET_PRES_SocketOpen(0, NET_PRES_SKT_UNENCRYPTED_STREAM_CLIENT, IP_ADDRESS_TYPE_IPV4,
                                         (NET_PRES_SKT_PORT_T)port,
                                         (NET_PRES_ADDRESS *)&appData.host_ipv4,
                                         (NET_PRES_SKT_ERROR_T*)&appData.error);
    NET_PRES_SocketWasReset(appData.socket);

    if (appData.socket == INVALID_SOCKET)
    {
        uart_send_tx_queue("[%s] NET_PRES_SocketOpen() socket invalid\n\r", basename(__FILE__));  // DEBUG: iPAS

        NET_PRES_SocketClose(appData.socket);
        return APP_CODE_ERROR_INVALID_SOCKET;
    }

    while (!NET_PRES_SKT_IsConnected(appData.socket))
    {
        if (APP_TIMER_Expired_ms(&timeout, timeout_ms))
        {
            uart_send_tx_queue("[%s] NET_PRES_SKT_IsConnected() timeout\n\r", basename(__FILE__));  // DEBUG: iPAS
            return APP_CODE_ERROR_CMD_TIMEOUT;
        }
    }

    while (NET_PRES_SKT_IsNegotiatingEncryption(appData.socket))
    {
        if (APP_TIMER_Expired_ms(&timeout, timeout_ms))
        {
            uart_send_tx_queue("[%s] NET_PRES_SKT_IsNegotiatingEncryption() timeout\n\r", basename(__FILE__));  // DEBUG: iPAS
            return APP_CODE_ERROR_CMD_TIMEOUT;
        }
    }

    // if (!NET_PRES_SKT_IsSecure(appData.socket))
    // {
    //     uart_send_tx_queue("[%s] NET_PRES_SKT_IsSecure() fail\n\r", basename(__FILE__));  // DEBUG: iPAS
    //     NET_PRES_SocketClose(appData.socket);
    //     return APP_CODE_ERROR_FAILED_SSL_NEGOTIATION;
    // }

    appData.socket_connected = true;
    return 0;  //Success
}

int APP_tcpipDisconnect_cb(void *context)
{
    int ret = 0;
    appData.socket_connected = false;
    NET_PRES_SKT_Close(appData.socket);
    appData.state = APP_STATE_MQTT_NET_CONNECT;
    return ret;
}

int APP_tcpipRead_cb(void *context, byte* buf, int buf_len, int timeout_ms)
{
    int ret = 0;
    uint32_t timeout;

    APP_TIMER_Set(&timeout);
    // Wait for data to be read, or error, or timeout
    while (NET_PRES_SocketReadIsReady(appData.socket) == 0)
    {
        if (NET_PRES_SocketWasReset(appData.socket))
        {
            ret = APP_CODE_ERROR_SSL_FATAL;
            return ret;
        }
        if (APP_TIMER_Expired_ms(&timeout, (uint32_t)timeout_ms))
        {
            ret = APP_CODE_ERROR_CMD_TIMEOUT;
            return ret;
        }
    }

    ret = NET_PRES_SocketRead(appData.socket, (uint8_t*)buf, buf_len);
    return ret;
}

int APP_tcpipWrite_cb(void *context, const byte* buf, int buf_len, int timeout_ms)
{
    int ret = 0;
    uint32_t timeout;

    APP_TIMER_Set(&timeout);
    // Wait for data to be read, or error, or timeout
    while (NET_PRES_SocketWriteIsReady(appData.socket, buf_len, 0) == 0)
    {
        if (NET_PRES_SocketWasReset(appData.socket))
        {
            ret = APP_CODE_ERROR_SSL_FATAL;
            return ret;
        }
        if (APP_TIMER_Expired_ms(&timeout, (uint32_t)timeout_ms))
        {
            ret = APP_CODE_ERROR_CMD_TIMEOUT;
            return ret;
        }
    }

    ret = NET_PRES_SocketWrite(appData.socket, (uint8_t*)buf, buf_len);
    return ret;
}

int mqttclient_message_cb(MqttClient *client, MqttMessage *msg, byte msg_new, byte msg_done)
{
    /* Show data of the subcribed topic */
    char *buf, *message, *topic;
    buf = (char *)malloc(msg->total_len + msg->topic_name_len + 2);
    message = buf;
    topic = (char *)&buf[msg->total_len + 1];

    memcpy(message, msg->buffer, msg->total_len);
    message[msg->total_len] = '\0';

    memcpy(topic, msg->topic_name, msg->topic_name_len);
    topic[msg->topic_name_len] = '\0';


    uart_send_tx_queue("--- received #%d from topic:%s msg:%s\n\r", msg->packet_id, topic, message);  // DEBUG: iPAS
    free(buf);


    if (strncmp(mqtt_topic_subscribed, msg->topic_name, strlen(mqtt_topic_subscribed)) == 0)
    {
        /* Publish on the subscribed topic */
        int rc = mqttclient_publish(mqtt_topic_published, "Thank you for your support!", packet_id++);
        if (rc != MQTT_CODE_SUCCESS)
        {
            appData.state = APP_FATAL_ERROR;
        }
    }

    return 0;
}


// *****************************************************************************
// *****************************************************************************
// Section: Application Local Functions
// *****************************************************************************
// *****************************************************************************

bool APP_TIMER_Expired(uint32_t * timer, uint32_t seconds)
{
    return ((SYS_TMR_TickCountGet() - *timer) > (seconds * 1000))? true : false;
}

bool APP_TIMER_Expired_ms(uint32_t * timer, uint32_t mseconds)
{
    return ((SYS_TMR_TickCountGet() - *timer) > (mseconds))? true : false;
}

bool APP_TIMER_Set(uint32_t * timer)
{
    *timer = SYS_TMR_TickCountGet();
    return true;
}

const char *basename(const char *name)
{   
    const char *r = name;
    const char *p = name;

    for (; *p != '\0'; p++)
    {
        if (*p == '/')
        {
            r = p+1;
        }
    }
    return r;
}

int mqttclient_publish(const char *topic, const char *buf, uint16_t pkg_id)
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

    char uart_buf[UART_QUEUE_ITEM_SIZE];
    snprintf(uart_buf, UART_QUEUE_ITEM_SIZE,
             "[#%d] publish topic:'%s' msg:'%s'\n\r", pkg_id, topic, buf);
    uart_send_tx_queue(uart_buf);  // DEBUG: iPAS

    return MqttClient_Publish(&appData.mqttClient, &publish);
}

int mqttclient_publish_random_number(const char *topic, uint16_t pkg_id)
{
    char buf[10];
    sprintf(buf, "%d", rand() % 40);
    
    return mqttclient_publish(topic, buf, pkg_id);
}

int mqttclient_publish_json(const char *topic, uint16_t pkg_id)
{
    char buf[MAX_BUFFER_SIZE];
    
    // -- JSON --
    JSON_Status sts;
    JSON_Value *root_value = json_value_init_object();
    JSON_Object *root_object = json_value_get_object(root_value);

    json_object_set_string(root_object, "device", "uRTU");
    json_object_dotset_number(root_object, "sensor.random", rand() % 40);

    // json_object_set_string(root_object, "name", "John Smith");
    // json_object_set_number(root_object, "age", 25);
    // json_object_dotset_string(root_object, "address.city", "Cupertino");
    // json_object_dotset_value(root_object, "contact.emails", json_parse_string("[\"email@example.com\",\"email2@example.com\"]"));

    // Example: https://github.com/kgabis/parson/blob/master/tests.c#L348
    JSON_Array *array_value;
    uint8_t i;

    json_object_set_value(root_object, "array", json_value_init_array());
    array_value = json_object_get_array(root_object, "array");
    for (i = 0; i < 20; i++)
    {
        json_array_append_number(array_value, rand() % 20);
    }

    char *serialized_string = NULL;
    // serialized_string = json_serialize_to_string_pretty(root_value);
    serialized_string = json_serialize_to_string(root_value);
    strncpy(buf, serialized_string, sizeof(buf));

    json_free_serialized_string(serialized_string);
    json_value_free(root_value);

    return mqttclient_publish(topic, buf, pkg_id);
}

int mqttclient_publish_feed(const char *topic, uint16_t pkg_id)
{
    char buf[MAX_BUFFER_SIZE];

    // -- JSON --
    JSON_Status sts;
    JSON_Value *root_value = json_value_init_object();
    JSON_Object *root_object = json_value_get_object(root_value);
    /*
    json_object_set_string(root_object, "feedid", feed_id);
    json_object_set_string(root_object, "description", "");
    json_object_set_null(root_object, "from");
    json_object_set_null(root_object, "to");

    JSON_Array *array_value;
    json_object_set_value(root_object, "granularity", json_value_init_array());
    array_value = json_object_get_array(root_object, "granularity");
    json_array_append_number(array_value, 1);
    json_array_append_string(array_value, "seconds");
    

    JSON_Value *data_value = json_value_init_object();
    JSON_Object *data_object = json_value_get_object(data_value);
    json_object_set_string(data_object, "attr", "temp");
    json_object_set_string(data_object, "unit", "C");
    json_object_set_value(data_object, "values", json_value_init_array());


    json_object_set_value(root_object, "lastest_data", json_value_init_array());
    array_value = json_object_get_array(root_object, "lastest_data");
    json_array_append_value(array_value, data_value);
    */
    
    json_object_set_number(root_object, "temp", rand() % 250);
    json_object_set_number(root_object, "humid", rand() % 100);
    

    char *serialized_string = NULL;
    serialized_string = json_serialize_to_string(root_value);
    strncpy(buf, serialized_string, sizeof(buf));

    json_free_serialized_string(serialized_string);
    json_value_free(root_value);

    return mqttclient_publish(topic, buf, pkg_id);
}

int mqttclient_publish_shadow(uint16_t pkg_id)
{
    char buf[MAX_BUFFER_SIZE];

    // -- JSON --
    JSON_Status sts;
    JSON_Value  *root_value  = json_value_init_object();
    JSON_Object *root_object = json_value_get_object(root_value);
    
    JSON_Value  *data_value  = json_value_init_object();
    JSON_Object *data_object = json_value_get_object(data_value);
    json_object_set_value(root_object, "data", data_value);
    
    json_object_set_number(data_object, "temp", rand() % 250);
    json_object_set_number(data_object, "humid", rand() % 100);

    
    char *serialized_string = NULL;
    serialized_string = json_serialize_to_string(root_value);
    strncpy(buf, serialized_string, sizeof(buf));

    json_free_serialized_string(serialized_string);
    json_value_free(root_value);

    return mqttclient_publish("@shadow/data/update", buf, pkg_id);
}


// *****************************************************************************
// *****************************************************************************
// Section: Application Initialization and State Machine Functions
// *****************************************************************************
// *****************************************************************************

/*******************************************************************************
  Function:
    void APP_MQTT_CLIENT_Initialize ( void )

  Remarks:
    See prototype in app_mqtt_client.h.
 */

void APP_MQTT_CLIENT_Initialize ( void )
{
    /* Place the App state machine in its initial state. */
    appData.state = APP_STATE_INIT;

    /* Initialize MQTT */
    sprintf(appData.macAddress, "Null");
    strncpy(appData.host, mqtt_broker, sizeof(appData.host));
    appData.port = mqtt_port;

    appData.mqttNet.connect    = APP_tcpipConnect_cb;
    appData.mqttNet.disconnect = APP_tcpipDisconnect_cb;
    appData.mqttNet.read       = APP_tcpipRead_cb;
    appData.mqttNet.write      = APP_tcpipWrite_cb;

    /* ETC. */
    appData.socket_connected   = false;
    appData.mqtt_connected     = false;
}


/******************************************************************************
  Function:
    void APP_MQTT_CLIENT_Tasks ( void )

  Remarks:
    See prototype in app_mqtt_client.h.
 */

void APP_MQTT_CLIENT_Tasks ( void )
{

    /* Check the application's current state. */
    switch ( appData.state )
    {
        /* Application's initial state. */
        case APP_STATE_INIT:
        {
            vTaskDelay(3000 / portTICK_PERIOD_MS);
            uart_send_tx_queue("--- APP MQTT Client Init ---\n\r");  // DEBUG: iPAS

            appData.state = APP_STATE_TCPIP_WAIT_INIT;
            break;
        }

        case APP_STATE_TCPIP_WAIT_INIT:
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
                    TCPIP_NET_HANDLE netH = TCPIP_STACK_NetHandleGet("PIC32INT");
                    TCPIP_MAC_ADDR* pAddr = (TCPIP_MAC_ADDR *)TCPIP_STACK_NetAddressMac(netH);
                    sprintf(appData.macAddress, "%02x%02x%02x%02x%02x%02x",
                            pAddr->v[0], pAddr->v[1], pAddr->v[2], pAddr->v[3], pAddr->v[4], pAddr->v[5]);
                }

                APP_TIMER_Set(&appData.genericUseTimer);
                appData.state = APP_STATE_TCPIP_WAIT_FOR_IP;

                uart_send_tx_queue("[%s] MAC: %s\n\r", basename(__FILE__), appData.macAddress);  // DEBUG: iPAS
            }
            break;
        }

        case APP_STATE_TCPIP_WAIT_FOR_IP:
        {
            TCPIP_NET_HANDLE    netH;
            int                 i, nNets;

            if (APP_TIMER_Expired(&appData.genericUseTimer, 10))
            {
                APP_TIMER_Set(&appData.genericUseTimer);
            }

            nNets = TCPIP_STACK_NumberOfNetworksGet();
            for (i = 0; i < nNets; i++)
            {
                netH = TCPIP_STACK_IndexToNet(i);

                IPV4_ADDR ipAddr;
                ipAddr.Val = TCPIP_STACK_NetAddress(netH);
                if (0 != ipAddr.Val)
                {
                    if (ipAddr.v[0] != 0 && ipAddr.v[0] != 169) // Wait for a Valid IP
                    {
                        appData.board_ipAddr.v4Add.Val = ipAddr.Val;
                        appData.state = APP_STATE_MQTT_INIT;

                        uart_send_tx_queue("[%s] IP: %d.%d.%d.%d\n\r",
                            basename(__FILE__),
                            appData.board_ipAddr.v4Add.v[0],
                            appData.board_ipAddr.v4Add.v[1],
                            appData.board_ipAddr.v4Add.v[2],
                            appData.board_ipAddr.v4Add.v[3]);  // DEBUG: iPAS
                    }
                }
            }
            break;
        }

        case APP_STATE_MQTT_INIT:
        {
            int rc = MqttClient_Init(&appData.mqttClient,
                                     &appData.mqttNet,
                                     mqttclient_message_cb,
                                     txBuffer, MAX_BUFFER_SIZE,
                                     rxBuffer, MAX_BUFFER_SIZE,
                                     MQTT_DEFAULT_CMD_TIMEOUT_MS);
            if (rc != MQTT_CODE_SUCCESS)
            {
                appData.state = APP_FATAL_ERROR;
                break;
            }
            APP_TIMER_Set(&appData.genericUseTimer);
            appData.state = APP_STATE_MQTT_NET_CONNECT;

            uart_send_tx_queue("[%s] MQTT init ready\n\r", basename(__FILE__));  // DEBUG: iPAS
            break;
        }

        case APP_STATE_MQTT_NET_CONNECT:
        {
            int rc = MqttClient_NetConnect(&appData.mqttClient,
                                           (const char *)appData.host, appData.port,
                                           MQTT_DEFAULT_CMD_TIMEOUT_MS,
                                           NULL,    // TLS disable
                                           NULL);   // TLS callback

            //
            // DEBUG: 'rc' is still 'MQTT_CODE_SUCCESS', even if the MQTT broker has been shutdown.
            //

            if (rc != MQTT_CODE_SUCCESS)
            {
                appData.socket_connected = appData.mqtt_connected = false;
                NET_PRES_SocketClose(appData.socket);

                while (!APP_TIMER_Expired(&appData.genericUseTimer, 5));  // Delay 5 seconds before trying next
                APP_TIMER_Set(&appData.genericUseTimer);

                uart_send_tx_queue("[%s] MQTT connect network %s fail (rc=%d)\n\r", basename(__FILE__), appData.host, rc);  // DEBUG: iPAS
                break;
            }
            appData.state = APP_STATE_MQTT_PROTOCOL_CONNECT;

            uart_send_tx_queue("[%s] MQTT network ready\n\r", basename(__FILE__));  // DEBUG: iPAS
            break;
        }

        case APP_STATE_MQTT_PROTOCOL_CONNECT:
        {
            MqttConnect connect;
            XMEMSET(&connect, 0, sizeof(connect));
            connect.keep_alive_sec = MQTT_KEEP_ALIVE;
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
            int rc = MqttClient_Connect(&appData.mqttClient, &connect);
            if(rc != MQTT_CODE_SUCCESS)
            {
                APP_TIMER_Set(&appData.genericUseTimer);
                while (!APP_TIMER_Expired(&appData.genericUseTimer, 5));

                appData.state = APP_TCPIP_ERROR;

                uart_send_tx_queue("[%s] MQTT protocol negotiation fail (rc=%d)\n\r", basename(__FILE__), rc);  // DEBUG: iPAS
                break;
            }
            appData.mqtt_connected = true;
            APP_TIMER_Set(&appData.mqttKeepAlive);
            appData.state = APP_STATE_MQTT_SUBSCRIBE;

            uart_send_tx_queue("[%s] MQTT protocol negotiation success\n\r", basename(__FILE__));  // DEBUG: iPAS
            break;
        }

        case APP_STATE_MQTT_SUBSCRIBE:
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

            rc = MqttClient_Subscribe(&appData.mqttClient, &subscribe);
            if (rc != MQTT_CODE_SUCCESS)
            {
                APP_TIMER_Set(&appData.genericUseTimer);
                while (!APP_TIMER_Expired(&appData.genericUseTimer, 5));

                appData.state = APP_TCPIP_ERROR;

                uart_send_tx_queue("[%s] MQTT subscription fail (rc=%d)\n\r", basename(__FILE__), rc);  // DEBUG: iPAS
                break;
            }

            appData.state = APP_STATE_MQTT_LOOP;

            uart_send_tx_queue("[%s] MQTT subscribe '%s'\n\r", basename(__FILE__), topics[0].topic_filter);  // DEBUG: iPAS
            break;
        }

        case APP_STATE_MQTT_LOOP:
        {
            int rc = 0;

            /* Keep Alive */
            if (APP_TIMER_Expired(&appData.mqttKeepAlive, MQTT_KEEP_ALIVE))
            {
                rc = MqttClient_Ping(&appData.mqttClient);
                if (rc != MQTT_CODE_SUCCESS)
                {
                    appData.state = APP_TCPIP_ERROR;
                }

                APP_TIMER_Set(&appData.mqttKeepAlive);  // Reset keep alive timer
                break;
            }

            /* Check for incoming messages */
            rc = MqttClient_WaitMessage(&appData.mqttClient, MQTT_DEFAULT_CMD_TIMEOUT_MS);
            if (rc == MQTT_CODE_ERROR_TIMEOUT)
            {
                /* Keep Alive */
                rc = MqttClient_Ping(&appData.mqttClient);
                if (rc != MQTT_CODE_SUCCESS)
                {
                    appData.state = APP_TCPIP_ERROR;
                    break;
                }
            }
            else
            if (rc == MQTT_CODE_ERROR_NETWORK)
            {
                appData.state = APP_TCPIP_ERROR;
                break;
            }
            else
            if (rc == APP_CODE_ERROR_CMD_TIMEOUT)  // No any message within DEFAULT_CMD_TIMEOUT_MS, then speak!
            {
                uart_send_tx_queue("[%s] No any message within %d ms, then speak! (APP_CODE_ERROR_CMD_TIMEOUT)\n\r", basename(__FILE__), MQTT_DEFAULT_CMD_TIMEOUT_MS);  // DEBUG: iPAS

                rc = mqttclient_publish(mqtt_topic_published, "No one can hear me??", packet_id++);
                if (rc != MQTT_CODE_SUCCESS)
                {
                    appData.state = APP_TCPIP_ERROR;
                }


                // ----------------------------------------
                rc = mqttclient_publish_random_number(mqtt_topic_number, packet_id++);
                if (rc != MQTT_CODE_SUCCESS)
                {
                    appData.state = APP_TCPIP_ERROR;
                }
                // ----------------------------------------

                // ----------------------------------------
                rc = mqttclient_publish_json(mqtt_topic_json, packet_id++);
                if (rc != MQTT_CODE_SUCCESS)
                {
                    appData.state = APP_TCPIP_ERROR;
                }
                // ----------------------------------------

                // ----------------------------------------
                #if BROKER_SELECTION == 1
                rc = mqttclient_publish_feed(mqtt_topic_feed, packet_id++);
                if (rc != MQTT_CODE_SUCCESS)
                {
                    appData.state = APP_TCPIP_ERROR;
                }
                #endif
                // ----------------------------------------

                // ----------------------------------------
                #if BROKER_SELECTION == 2
                rc = mqttclient_publish_shadow(packet_id++);
                if (rc != MQTT_CODE_SUCCESS)
                {
                    appData.state = APP_TCPIP_ERROR;
                }
                #endif
                // ----------------------------------------

                APP_TIMER_Set(&appData.mqttKeepAlive);  // Reset keep alive timer since we sent a publish
                break;
            }
            else
            if ( rc != MQTT_CODE_SUCCESS)
            {
                appData.state = APP_TCPIP_ERROR;
                break;
            }

            break;
        }

        /* Connection lost */
        case APP_TCPIP_ERROR:
        {
            appData.socket_connected = appData.mqtt_connected = false;
            NET_PRES_SocketClose(appData.socket);
            appData.state = APP_STATE_MQTT_NET_CONNECT;

            uart_send_tx_queue("[%s] APP_TCPIP_ERROR, going to APP_STATE_MQTT_NET_CONNECT\n\r", basename(__FILE__));  // DEBUG: iPAS
            break;
        }

        /* This error requires system reset or hardware debugging */
        case APP_FATAL_ERROR:
        {
            uart_send_tx_queue("[%s] APP_FATAL_ERROR\n\r", basename(__FILE__));  // DEBUG: iPAS
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
