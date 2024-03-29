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

#if defined(DO_TRACE) && (DO_TRACE != 0)
#include "app_uart_term.h"
#define TRACE_LOG(...) uart_send_tx_queue(__VA_ARGS__)
#elif defined(DO_LOG) && (DO_LOG != 0)
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

#if ! defined(MAIN_IFACE)
//#define PREF_IFACE "eth0"
#define PREF_IFACE "wlan0"
#else
#define STRINGIZE(x) #x
#define STRINGIZE_VALUE_OF(x) STRINGIZE(x)
#define PREF_IFACE STRINGIZE_VALUE_OF(MAIN_IFACE)
#endif

#define MQTT_WAIT_IFACE 5000

#define MQTT_DEFAULT_CMD_TIMEOUT_MS 30000
#if !defined(MQTT_KEEP_ALIVE_TIMEOUT)
#define MQTT_KEEP_ALIVE_TIMEOUT 15
#endif
#define MQTT_UPDATE_STATUS_TIMEOUT 60

#if defined(NEXPIE) && (NEXPIE != 0)
// -- NEXPIE --
// server: mqtt.nexpie.io 1883 (mqtt)
//         mqtt.nexpie.io 1884 (mqtts)
const char mqtt_broker[] = "mqtt.nexpie.io";
#else
// -- NETPIE2020 --
// server: broker.netpie.io 1883 (mqtt)
//         broker.netpie.io 1884 (mqtts)
const char mqtt_broker[] = "broker.netpie.io";
#endif

#if defined(USE_MQTTS) && (USE_MQTTS != 0)
#define NETPIE_TCP_PORT 1884
#define NETPIE_NET_PRES_SOCKET_TYPE NET_PRES_SKT_ENCRYPTED_STREAM_CLIENT
#else
#define NETPIE_TCP_PORT 1883
#define NETPIE_NET_PRES_SOCKET_TYPE NET_PRES_SKT_UNENCRYPTED_STREAM_CLIENT
#endif

const char mqtt_client_id[] = NETPIE_CLIENT_ID;
const char mqtt_user[]      = NETPIE_TOKEN;
const char mqtt_password[]  = NETPIE_SECRET;

// -- MQTT topics for updating register --
const char mqtt_topic_status[]   = "@shadow/data/update";                   // Publish the device status on the 'shadow'
                                                                            // https://docs-v2.nexpie.io/mqtt-api.html#shadow-api-topic
const char mqtt_topic_filter[]   = "@msg/" NETPIE_DEVICE_NAME "/#";         // Subscripted topics
const char mqtt_topic_update[]   = "@msg/" NETPIE_DEVICE_NAME "/update";    // Get request for updating the register/%d
const char mqtt_topic_register[] = "@msg/" NETPIE_DEVICE_NAME "/register";  // Publish an update for register/%d
const char mqtt_topic_log[]      = "@msg/" NETPIE_DEVICE_NAME "/log";       // log

static netpie_callback_t subscription_callback = NULL;

TaskHandle_t xTaskHandleNetpie;


// *****************************************************************************
// *****************************************************************************
// Section: Application Local Functions
// *****************************************************************************
// *****************************************************************************

static bool APP_timerExpired_ms(uint32_t *timer, uint32_t ms_seconds)
{
    bool ret = ((SYS_TMR_TickCountGet() - *timer) > ms_seconds)? true : false;
    if (ret == false) vTaskDelay(1 / portTICK_PERIOD_MS);  // Prevent taking much burden
    return ret;
}


static bool APP_timerExpired(uint32_t *timer, uint32_t seconds)
{
    return APP_timerExpired_ms(timer, seconds * 1000);
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

    TRACE_LOG("[NETPIE:%d] #%d publish topic:'%s' msg:'%s'\n\r", __LINE__, pkg_id, topic, buf);

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
    TRACE_LOG("[NETPIE:%d] APP_tcpipConnect_cb() get started\n\r", __LINE__);  // DEBUG: iPAS

    uint32_t timeout;
    timeout = SYS_TMR_TickCountGet();

    appNetpieData.socket_connected = false;

    TCPIP_DNS_RESULT dnsResult = TCPIP_DNS_Resolve((const char *)appNetpieData.host, TCPIP_DNS_TYPE_A);
    if(dnsResult < 0)
    {
        TRACE_LOG("[NETPIE:%d] TCPIP_DNS_Resolve() fail code %d\n\r", __LINE__, dnsResult);  // DEBUG: iPAS
        return APP_CODE_ERROR_FAILED_TO_BEGIN_DNS_RESOLUTION;  // DNS resolving problem
    }

    while((dnsResult = TCPIP_DNS_IsResolved((const char *)appNetpieData.host, &appNetpieData.host_ipv4, IP_ADDRESS_TYPE_IPV4)
          ) == TCPIP_DNS_RES_PENDING)
    {
        if(APP_timerExpired_ms(&timeout, timeout_ms))
        {
            TRACE_LOG("[NETPIE:%d] TCPIP_DNS_IsResolve() timeout in %d ms\n\r", __LINE__, timeout_ms);  // DEBUG: iPAS
            return APP_CODE_ERROR_CMD_TIMEOUT;
        }
    }
    if(dnsResult != (TCPIP_DNS_RES_OK))
    {
        TRACE_LOG("[NETPIE:%d] TCPIP_DNS_IsResolve() fail code %d\n\r", __LINE__, dnsResult);  // DEBUG: iPAS
        return APP_CODE_ERROR_DNS_FAILED;
    }

    appNetpieData.socket = NET_PRES_SocketOpen(0, NETPIE_NET_PRES_SOCKET_TYPE, IP_ADDRESS_TYPE_IPV4,
                                         (NET_PRES_SKT_PORT_T)port,
                                         (NET_PRES_ADDRESS *)&appNetpieData.host_ipv4,
                                         (NET_PRES_SKT_ERROR_T*)&appNetpieData.error);
    NET_PRES_SocketWasReset(appNetpieData.socket);

    if (appNetpieData.socket == INVALID_SOCKET)
    {
        TRACE_LOG("[NETPIE:%d] NET_PRES_SocketOpen() socket invalid\n\r", __LINE__);  // DEBUG: iPAS

        NET_PRES_SocketClose(appNetpieData.socket);
        return APP_CODE_ERROR_INVALID_SOCKET;
    }

    while (!NET_PRES_SKT_IsConnected(appNetpieData.socket))
    {
        if (APP_timerExpired_ms(&timeout, timeout_ms))
        {
            TRACE_LOG("[NETPIE:%d] NET_PRES_SKT_IsConnected() timeout\n\r", __LINE__);  // DEBUG: iPAS
            return APP_CODE_ERROR_CMD_TIMEOUT;
        }
    }

    while (NET_PRES_SKT_IsNegotiatingEncryption(appNetpieData.socket))
    {
        if (APP_timerExpired_ms(&timeout, timeout_ms))
        {
            TRACE_LOG("[NETPIE:%d] NET_PRES_SKT_IsNegotiatingEncryption() timeout\n\r", __LINE__);  // DEBUG: iPAS
            return APP_CODE_ERROR_CMD_TIMEOUT;
        }
    }

    #if defined(USE_MQTTS) && (USE_MQTTS != 0)
    if (!NET_PRES_SKT_IsSecure(appNetpieData.socket))
    {
        TRACE_LOG("[NETPIE:%d] NET_PRES_SKT_IsSecure() fail\n\r", __LINE__);  // DEBUG: iPAS
        NET_PRES_SocketClose(appNetpieData.socket);
        return APP_CODE_ERROR_FAILED_SSL_NEGOTIATION;
    }
    #endif

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


    TRACE_LOG("[NETPIE:%d] recv #%d topic:%s msg:'%s'\n\r", __LINE__, msg->packet_id, topic, message);  // DEBUG: iPAS


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
            TRACE_LOG("[NETPIE:%d] A subscribed message is received ,but , subscription_callback is NULL!\n\r", __LINE__);
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
    bool ret = (appNetpieData.socket_connected && appNetpieData.mqtt_connected)? true : false;
    //TRACE_LOG("[NETPIE:%d] netpie_ready socket:%d mqtt:%d\n\r", __LINE__, appNetpieData.socket_connected, appNetpieData.mqtt_connected);  // DEBUG: iPAS
    return ret;
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


/**
 * Set running status
 * @param sts
 * @return
 */
bool netpie_set_running(bool sts)
{
    switch (sts) {
    case true:
        vTaskResume(xTaskHandleNetpie);
        break;

    case false:
        appNetpieData.do_suspend = true;
        break;
    }

    return sts;
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
    appNetpieData.do_suspend = false;

    /* Place the App state machine in its initial state. */
    appNetpieData.state = APP_NETPIE_STATE_INIT;

    /* Initialize MQTT */
    sprintf(appNetpieData.macAddress, "Null");
    strncpy(appNetpieData.host, mqtt_broker, sizeof(appNetpieData.host));
    appNetpieData.port = NETPIE_TCP_PORT;

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
    if (appNetpieData.do_suspend) {
        MqttClient_Disconnect(&appNetpieData.mqttClient);  // FIXME: disable for fix a crash, don't know why
        MqttClient_NetDisconnect(&appNetpieData.mqttClient);
        appNetpieData.socket_connected = false;  // netpie_ready() => false
        appNetpieData.mqtt_connected   = false;

        vTaskSuspend(NULL);
        // ... Long deep sleep ...

        appNetpieData.do_suspend = false;
        appNetpieData.state = APP_NETPIE_STATE_INIT;
    }


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
                TRACE_LOG("[NETPIE:%d] TCPIP ready!\n\r", __LINE__);

                // Now the stack is ready, we can check the available interfaces
                nNets = TCPIP_STACK_NumberOfNetworksGet();
                for (i = 0; i < nNets; i++)
                {
                    netH = TCPIP_STACK_IndexToNet(i);
                    const char* net_name = TCPIP_STACK_NetNameGet(netH);
                    //const char* netbios_name = TCPIP_STACK_NetBIOSName(netH);

                    // Retrieve MAC Address to store and convert to string
                    TCPIP_MAC_ADDR* pAddr = (TCPIP_MAC_ADDR *)TCPIP_STACK_NetAddressMac(netH);
                    sprintf(appNetpieData.macAddress, "%02x%02x%02x%02x%02x%02x",
                            pAddr->v[0], pAddr->v[1], pAddr->v[2], pAddr->v[3], pAddr->v[4], pAddr->v[5]);

                    // Show interface's MAC
                    char ifname[10];
                    ifname[ TCPIP_STACK_NetAliasNameGet(netH, ifname, sizeof(ifname)) ] = '\0';
                    TRACE_LOG("[NETPIE:%d] '%s':%s MAC:%s\n\r", __LINE__, ifname, net_name, appNetpieData.macAddress);  // DEBUG: iPAS
                }

                //APP_timerSet(&appNetpieData.genericUseTimer);  // XXX: It can be removed?
                appNetpieData.state = APP_NETPIE_STATE_TCPIP_WAIT_FOR_IP;
            }
            break;
        }

        case APP_NETPIE_STATE_TCPIP_WAIT_FOR_IP:
        {
            TCPIP_NET_HANDLE    netH;
            int                 i, nNets;

            // XXX: It can be removed?
            //if (APP_timerExpired(&appNetpieData.genericUseTimer, 10))
            //{
            //    APP_timerSet(&appNetpieData.genericUseTimer);
            //}

            vTaskDelay(MQTT_WAIT_IFACE / portTICK_PERIOD_MS);

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
                        uint8_t ifname_len = TCPIP_STACK_NetAliasNameGet(netH, ifname, sizeof(ifname));
                        ifname[ ifname_len ] = '\0';

                        //const char *handlers[] = {"PIC32INT", "MRF24W"};  TCPIP_STACK_IF_NAME_PIC32INT  TCPIP_STACK_IF_NAME_MRF24W
                        //TCPIP_NET_HANDLE netH = TCPIP_STACK_NetHandleGet("PIC32INT");

                        //const char *ifnames[] = {"eth0", "wlan0"};
                        if (strcmp(ifname, PREF_IFACE) == 0)  // Waiting for the preferred default interface
                        {
                            appNetpieData.board_ipAddr.v4Add.Val = ipAddr.Val;  // saved for debugging

                            IPV4_ADDR ipDNS;
                            ipDNS.Val = TCPIP_STACK_NetAddressDnsPrimary(netH);  // DNS
                            TCPIP_STACK_NetDefaultSet(netH);  // Set default network interface

                            appNetpieData.state = APP_NETPIE_STATE_MQTT_INIT;

                            TRACE_LOG("[NETPIE:%d] Preferred '%s' found! IP:%d.%d.%d.%d DNS:%d.%d.%d.%d\n\r", __LINE__, ifname,
                                    ipAddr.v[0], ipAddr.v[1], ipAddr.v[2], ipAddr.v[3],
                                    ipDNS.v[0], ipDNS.v[1], ipDNS.v[2], ipDNS.v[3]);  // DEBUG: iPAS
                        }
                        else
                        {
                            /* XXX: @iPAS > useful code
                            if (TCPIP_DNS_IsEnabled(netH))
                            {
                                TCPIP_STACK_NetDown(netH);  // XXX: quick fix DNS-request-timeout on multiple interfaces
                                TCPIP_DNS_Disable(netH, true);  // XXX: quick fix DNS-request-timeout on multiple interfaces
                            }
                            */

                            TRACE_LOG("[NETPIE:%d] '%s' IP:%d.%d.%d.%d\n\r", __LINE__, ifname,
                                    ipAddr.v[0], ipAddr.v[1], ipAddr.v[2], ipAddr.v[3]);  // DEBUG: iPAS
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
                TRACE_LOG("[NETPIE:%d] MQTT init fail (rc=%d)\n\r", __LINE__, rc);  // DEBUG: iPAS
                appNetpieData.state = APP_NETPIE_STATE_FATAL_ERROR;
                break;
            }

            //APP_timerSet(&appNetpieData.genericUseTimer);  // XXX: It can be removed?
            appNetpieData.state = APP_NETPIE_STATE_MQTT_NET_CONNECT;

            TRACE_LOG("[NETPIE:%d] MQTT init ready\n\r", __LINE__);  // DEBUG: iPAS
            break;
        }

        case APP_NETPIE_STATE_MQTT_NET_CONNECT:
        {
            int rc = MqttClient_NetConnect(&appNetpieData.mqttClient,
                                           (const char *)appNetpieData.host, appNetpieData.port,
                                           MQTT_DEFAULT_CMD_TIMEOUT_MS,
                                           NULL,    // TODO: TLS enable/disable
                                           NULL);   // TODO: TLS callback

            //
            // DEBUG: 'rc' is still 'MQTT_CODE_SUCCESS', even if the MQTT broker has been shutdown.
            //

            if (rc != MQTT_CODE_SUCCESS)
            {
                TRACE_LOG("[NETPIE:%d] MQTT connect server %s fail (rc=%d)\n\r", __LINE__, appNetpieData.host, rc);  // DEBUG: iPAS
                appNetpieData.state = APP_NETPIE_STATE_TCPIP_ERROR;

                // XXX: It can be removed?
                //APP_timerSet(&appNetpieData.genericUseTimer);
                //while (!APP_timerExpired(&appNetpieData.genericUseTimer, 5));  // Delay 5 seconds before trying next
                vTaskDelay(1000 / portTICK_PERIOD_MS);
                break;
            }

            appNetpieData.state = APP_NETPIE_STATE_MQTT_PROTOCOL_CONNECT;

            TRACE_LOG("[NETPIE:%d] MQTT network ready\n\r", __LINE__);  // DEBUG: iPAS
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
                TRACE_LOG("[NETPIE:%d] MQTT protocol negotiation fail (rc=%d)\n\r", __LINE__, rc);  // DEBUG: iPAS
                appNetpieData.state = APP_NETPIE_STATE_TCPIP_ERROR;

                // XXX: It can be removed?
                //APP_timerSet(&appNetpieData.genericUseTimer);
                //while (!APP_timerExpired(&appNetpieData.genericUseTimer, 5));
                vTaskDelay(1000 / portTICK_PERIOD_MS);
                break;
            }
            appNetpieData.mqtt_connected = true;
            APP_timerSet(&appNetpieData.mqttKeepAlive);
            APP_timerSet(&appNetpieData.mqttUpdateStatus);
            appNetpieData.mqttKeepAlive = 0;  // Do it at the first time. Required for some broker, e.g., NETPIE
            appNetpieData.state = APP_NETPIE_STATE_MQTT_SUBSCRIBE;

            TRACE_LOG("[NETPIE:%d] MQTT protocol negotiation success\n\r", __LINE__);  // DEBUG: iPAS
            break;
        }

        case APP_NETPIE_STATE_MQTT_SUBSCRIBE:
        {
            MqttSubscribe subscribe;
            MqttTopic topics[1];
            MqttPublish publish;
            int rc;

            /* Build list of topics */
            topics[0].topic_filter = mqtt_topic_filter;
            topics[0].qos = 0;

            /* Subscribe Topic */
            XMEMSET(&subscribe, 0, sizeof(MqttSubscribe));
            subscribe.packet_id = packet_id++;
            subscribe.topic_count = sizeof(topics)/sizeof(MqttTopic);
            subscribe.topics = topics;

            rc = MqttClient_Subscribe(&appNetpieData.mqttClient, &subscribe);
            if (rc != MQTT_CODE_SUCCESS)
            {
                TRACE_LOG("[NETPIE:%d] MQTT subscription fail (rc=%d)\n\r", __LINE__, rc);  // DEBUG: iPAS
                appNetpieData.state = APP_NETPIE_STATE_TCPIP_ERROR;
                // XXX: It can be removed?
                //APP_timerSet(&appNetpieData.genericUseTimer);
                //while (!APP_timerExpired(&appNetpieData.genericUseTimer, 5));
                vTaskDelay(1000 / portTICK_PERIOD_MS);
                break;
            }

            appNetpieData.state = APP_NETPIE_STATE_MQTT_LOOP;

            TRACE_LOG("[NETPIE:%d] MQTT subscribe '%s'\n\r", __LINE__, topics[0].topic_filter);  // DEBUG: iPAS
            break;
        }

        case APP_NETPIE_STATE_MQTT_LOOP:
        {
            int rc = 0;

            /* Keep alive */
            if (APP_timerExpired(&appNetpieData.mqttKeepAlive, MQTT_KEEP_ALIVE_TIMEOUT))
            {
                TRACE_LOG("[NETPIE:%d] Keep alive MQTT every %ds\n\r", __LINE__, MQTT_KEEP_ALIVE_TIMEOUT);  // DEBUG: iPAS

                APP_timerSet(&appNetpieData.mqttKeepAlive);  // Reset keep alive timer

                rc = MqttClient_Ping(&appNetpieData.mqttClient);
                if (rc != MQTT_CODE_SUCCESS)
                {
                    appNetpieData.state = APP_NETPIE_STATE_TCPIP_ERROR;
                    break;
                }
            }

            /* Update status */
            if (APP_timerExpired(&appNetpieData.mqttUpdateStatus, MQTT_UPDATE_STATUS_TIMEOUT))
            {
                TRACE_LOG("[NETPIE:%d] Update MQTT status every %ds\n\r", __LINE__, MQTT_UPDATE_STATUS_TIMEOUT);  // DEBUG: iPAS

                APP_timerSet(&appNetpieData.mqttUpdateStatus);  // Reset status update timer
                APP_timerSet(&appNetpieData.mqttKeepAlive);  // Since we sent a publish, saving bandwidth

                rc = netpie_publish_status();
                if (rc != MQTT_CODE_SUCCESS)
                {
                    appNetpieData.state = APP_NETPIE_STATE_TCPIP_ERROR;
                    break;
                }
            }

            /* Check for incoming messages */
            rc = MqttClient_WaitMessage(&appNetpieData.mqttClient, MQTT_DEFAULT_CMD_TIMEOUT_MS);
            if (rc == MQTT_CODE_SUCCESS)
            {
                // Nothing to do, the callback function will handle the rest.
            }
            else
            if (rc == MQTT_CODE_ERROR_TIMEOUT)
            {
                TRACE_LOG("[NETPIE:%d] No any message within %d ms\n\r", __LINE__, MQTT_DEFAULT_CMD_TIMEOUT_MS);  // DEBUG: iPAS
            }
            else
            {
                TRACE_LOG("[NETPIE:%d] Waiting message and got error %d:'%s'\n\r", __LINE__, rc, MqttClient_ReturnCodeToString(rc));  // DEBUG: iPAS
                //appNetpieData.state = APP_NETPIE_STATE_TCPIP_ERROR;  // XXX: It's OK for now.
            }

            break;
        }

        /* Connection lost */
        case APP_NETPIE_STATE_TCPIP_ERROR:
        {
            appNetpieData.socket_connected = appNetpieData.mqtt_connected = false;
            MqttClient_NetDisconnect(&appNetpieData.mqttClient);
            NET_PRES_SocketClose(appNetpieData.socket);
            appNetpieData.state = APP_NETPIE_STATE_MQTT_NET_CONNECT;

            TRACE_LOG("[NETPIE:%d] APP_TCPIP_ERROR, going to APP_STATE_MQTT_NET_CONNECT\n\r", __LINE__);  // DEBUG: iPAS
            break;
        }

        /* This error requires system reset or hardware debugging */
        case APP_NETPIE_STATE_FATAL_ERROR:
        {
            TRACE_LOG("[NETPIE:%d] APP_FATAL_ERROR\n\r", __LINE__);  // DEBUG: iPAS
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
