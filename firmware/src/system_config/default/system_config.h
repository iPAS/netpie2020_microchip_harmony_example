/*******************************************************************************
  MPLAB Harmony System Configuration Header

  File Name:
    system_config.h

  Summary:
    Build-time configuration header for the system defined by this MPLAB Harmony
    project.

  Description:
    An MPLAB Project may have multiple configurations.  This file defines the
    build-time options for a single configuration.

  Remarks:
    This configuration header must not define any prototypes or data
    definitions (or include any files that do).  It only provides macro
    definitions for build-time configuration options that are not instantiated
    until used by another MPLAB Harmony module or application.

    Created with MPLAB Harmony Version 1.11
*******************************************************************************/

// DOM-IGNORE-BEGIN
/*******************************************************************************
Copyright (c) 2013-2015 released Microchip Technology Inc.  All rights reserved.

Microchip licenses to you the right to use, modify, copy and distribute
Software only when embedded on a Microchip microcontroller or digital signal
controller that is integrated into your product or third party product
(pursuant to the sublicense terms in the accompanying license agreement).

You should refer to the license agreement accompanying this Software for
additional information regarding your rights and obligations.

SOFTWARE AND DOCUMENTATION ARE PROVIDED AS IS WITHOUT WARRANTY OF ANY KIND,
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

#ifndef _SYSTEM_CONFIG_H
#define _SYSTEM_CONFIG_H

// *****************************************************************************
// *****************************************************************************
// Section: Included Files
// *****************************************************************************
// *****************************************************************************
/*  This section Includes other configuration headers necessary to completely
    define this configuration.
*/

// DOM-IGNORE-BEGIN
#ifdef __cplusplus  // Provide C++ Compatibility

extern "C" {

#endif
// DOM-IGNORE-END

// *****************************************************************************
// *****************************************************************************
// Section: System Service Configuration
// *****************************************************************************
// *****************************************************************************
// *****************************************************************************
/* Common System Service Configuration Options
*/
#define SYS_VERSION_STR           "1.11"
#define SYS_VERSION               11100

// *****************************************************************************
/* Clock System Service Configuration Options
*/
#define SYS_CLK_FREQ                        100000000ul
#define SYS_CLK_BUS_PERIPHERAL_1            50000000ul
#define SYS_CLK_BUS_PERIPHERAL_2            50000000ul
#define SYS_CLK_BUS_PERIPHERAL_3            50000000ul
#define SYS_CLK_BUS_PERIPHERAL_4            50000000ul
#define SYS_CLK_BUS_PERIPHERAL_5            50000000ul
#define SYS_CLK_BUS_PERIPHERAL_7            100000000ul
#define SYS_CLK_BUS_PERIPHERAL_8            100000000ul
#define SYS_CLK_CONFIG_PRIMARY_XTAL         24000000ul
#define SYS_CLK_CONFIG_SECONDARY_XTAL       0ul
   
/*** Interrupt System Service Configuration ***/
#define SYS_INT                     true

/*** Ports System Service Configuration ***/

#define SYS_PORT_A_ANSEL        0x621
#define SYS_PORT_A_TRIS         0xc4ed
#define SYS_PORT_A_LAT          0x202
#define SYS_PORT_A_ODC          0x0
#define SYS_PORT_A_CNPU         0x0
#define SYS_PORT_A_CNPD         0x0
#define SYS_PORT_A_CNEN         0x0

#define SYS_PORT_B_ANSEL        0xc65b
#define SYS_PORT_B_TRIS         0xfd57
#define SYS_PORT_B_LAT          0x280
#define SYS_PORT_B_ODC          0x0
#define SYS_PORT_B_CNPU         0x0
#define SYS_PORT_B_CNPD         0x0
#define SYS_PORT_B_CNEN         0x0

#define SYS_PORT_C_ANSEL        0x6000
#define SYS_PORT_C_TRIS         0xf00e
#define SYS_PORT_C_LAT          0x10
#define SYS_PORT_C_ODC          0x0
#define SYS_PORT_C_CNPU         0x8000
#define SYS_PORT_C_CNPD         0x0
#define SYS_PORT_C_CNEN         0x0

#define SYS_PORT_D_ANSEL        0xc000
#define SYS_PORT_D_TRIS         0xfc3f
#define SYS_PORT_D_LAT          0x200
#define SYS_PORT_D_ODC          0x200
#define SYS_PORT_D_CNPU         0x0
#define SYS_PORT_D_CNPD         0x0
#define SYS_PORT_D_CNEN         0x0

#define SYS_PORT_E_ANSEL        0x1f0
#define SYS_PORT_E_TRIS         0x3f9
#define SYS_PORT_E_LAT          0x6
#define SYS_PORT_E_ODC          0x0
#define SYS_PORT_E_CNPU         0x0
#define SYS_PORT_E_CNPD         0x0
#define SYS_PORT_E_CNEN         0x0

#define SYS_PORT_F_ANSEL        0x0
#define SYS_PORT_F_TRIS         0x2039
#define SYS_PORT_F_LAT          0x0
#define SYS_PORT_F_ODC          0x0
#define SYS_PORT_F_CNPU         0x0
#define SYS_PORT_F_CNPD         0x0
#define SYS_PORT_F_CNEN         0x0

#define SYS_PORT_G_ANSEL        0xc0
#define SYS_PORT_G_TRIS         0x43c2
#define SYS_PORT_G_LAT          0x3000
#define SYS_PORT_G_ODC          0x0
#define SYS_PORT_G_CNPU         0x0
#define SYS_PORT_G_CNPD         0x0
#define SYS_PORT_G_CNEN         0x0
/*** Timer System Service Configuration ***/
#define SYS_TMR_POWER_STATE             SYS_MODULE_POWER_RUN_FULL
#define SYS_TMR_DRIVER_INDEX            DRV_TMR_INDEX_0
#define SYS_TMR_MAX_CLIENT_OBJECTS      5
#define SYS_TMR_FREQUENCY               1000
#define SYS_TMR_FREQUENCY_TOLERANCE     10
#define SYS_TMR_UNIT_RESOLUTION         10000
#define SYS_TMR_CLIENT_TOLERANCE        10
#define SYS_TMR_INTERRUPT_NOTIFICATION  false

// *****************************************************************************
/* Random System Service Configuration Options
*/

#define SYS_RANDOM_CRYPTO_SEED_SIZE  32


// *****************************************************************************
// *****************************************************************************
// Section: Driver Configuration
// *****************************************************************************
// *****************************************************************************
/*** Timer Driver Configuration ***/
#define DRV_TMR_INTERRUPT_MODE             true
#define DRV_TMR_INSTANCES_NUMBER           1
#define DRV_TMR_CLIENTS_NUMBER             1

/*** Timer Driver 0 Configuration ***/
#define DRV_TMR_PERIPHERAL_ID_IDX0          TMR_ID_2
#define DRV_TMR_INTERRUPT_SOURCE_IDX0       INT_SOURCE_TIMER_2
#define DRV_TMR_INTERRUPT_VECTOR_IDX0       INT_VECTOR_T2
#define DRV_TMR_ISR_VECTOR_IDX0             _TIMER_2_VECTOR
#define DRV_TMR_INTERRUPT_PRIORITY_IDX0     INT_PRIORITY_LEVEL1
#define DRV_TMR_INTERRUPT_SUB_PRIORITY_IDX0 INT_SUBPRIORITY_LEVEL0
#define DRV_TMR_CLOCK_SOURCE_IDX0           DRV_TMR_CLKSOURCE_INTERNAL
#define DRV_TMR_PRESCALE_IDX0               TMR_PRESCALE_VALUE_256
#define DRV_TMR_OPERATION_MODE_IDX0         DRV_TMR_OPERATION_MODE_16_BIT
#define DRV_TMR_ASYNC_WRITE_ENABLE_IDX0     false
#define DRV_TMR_POWER_STATE_IDX0            SYS_MODULE_POWER_RUN_FULL

 // *****************************************************************************
/* USART Driver Configuration Options
*/
#define DRV_USART_INSTANCES_NUMBER                  2
#define DRV_USART_CLIENTS_NUMBER                    2
#define DRV_USART_INTERRUPT_MODE                    true
#define DRV_USART_BYTE_MODEL_SUPPORT                true
#define DRV_USART_READ_WRITE_MODEL_SUPPORT          false
#define DRV_USART_BUFFER_QUEUE_SUPPORT              false
#define DRV_USART_SUPPORT_TRANSMIT_DMA              false
#define DRV_USART_SUPPORT_RECEIVE_DMA               false

// *****************************************************************************
// *****************************************************************************
// Section: Middleware & Other Library Configuration
// *****************************************************************************
// *****************************************************************************
/*** OSAL Configuration ***/
#define OSAL_USE_RTOS          9


// *****************************************************************************
// *****************************************************************************
// Section: TCPIP Stack Configuration
// *****************************************************************************
// *****************************************************************************
#define TCPIP_STACK_USE_IPV4
#define TCPIP_STACK_USE_TCP
#define TCPIP_STACK_USE_UDP

#define TCPIP_STACK_TICK_RATE		        		5
#define TCPIP_STACK_SECURE_PORT_ENTRIES             10

/* TCP/IP stack event notification */
#define TCPIP_STACK_USE_EVENT_NOTIFICATION
#define TCPIP_STACK_USER_NOTIFICATION   false
#define TCPIP_STACK_DOWN_OPERATION   true
#define TCPIP_STACK_IF_UP_DOWN_OPERATION   true
#define TCPIP_STACK_MAC_DOWN_OPERATION  true
#define TCPIP_STACK_CONFIGURATION_SAVE_RESTORE   true
/*** TCPIP Heap Configuration ***/
#define TCPIP_STACK_USE_INTERNAL_HEAP
#define TCPIP_STACK_DRAM_SIZE                       39250
#define TCPIP_STACK_DRAM_RUN_LIMIT                  2048
#define TCPIP_STACK_MALLOC_FUNC                     malloc

#define TCPIP_STACK_CALLOC_FUNC                     calloc

#define TCPIP_STACK_FREE_FUNC                       free



#define TCPIP_STACK_HEAP_USE_FLAGS                   TCPIP_STACK_HEAP_FLAG_ALLOC_UNCACHED

#define TCPIP_STACK_HEAP_USAGE_CONFIG                TCPIP_STACK_HEAP_USE_DEFAULT

#define TCPIP_STACK_SUPPORTED_HEAPS                  1

/*** ARP Configuration ***/
#define TCPIP_ARP_CACHE_ENTRIES                 		5
#define TCPIP_ARP_CACHE_DELETE_OLD		        	true
#define TCPIP_ARP_CACHE_SOLVED_ENTRY_TMO			1200
#define TCPIP_ARP_CACHE_PENDING_ENTRY_TMO			60
#define TCPIP_ARP_CACHE_PENDING_RETRY_TMO			2
#define TCPIP_ARP_CACHE_PERMANENT_QUOTA		    		50
#define TCPIP_ARP_CACHE_PURGE_THRESHOLD		    		75
#define TCPIP_ARP_CACHE_PURGE_QUANTA		    		1
#define TCPIP_ARP_CACHE_ENTRY_RETRIES		    		3
#define TCPIP_ARP_GRATUITOUS_PROBE_COUNT			1
#define TCPIP_ARP_TASK_PROCESS_RATE		        	2



/*** DNS Client Configuration ***/
#define TCPIP_STACK_USE_DNS
#define TCPIP_DNS_CLIENT_SERVER_TMO					60
#define TCPIP_DNS_CLIENT_TASK_PROCESS_RATE			200
#define TCPIP_DNS_CLIENT_CACHE_ENTRIES				5
#define TCPIP_DNS_CLIENT_CACHE_ENTRY_TMO			0
#define TCPIP_DNS_CLIENT_CACHE_PER_IPV4_ADDRESS		5
#define TCPIP_DNS_CLIENT_CACHE_PER_IPV6_ADDRESS		1
#define TCPIP_DNS_CLIENT_ADDRESS_TYPE			    IP_ADDRESS_TYPE_IPV4
#define TCPIP_DNS_CLIENT_CACHE_DEFAULT_TTL_VAL		1200
#define TCPIP_DNS_CLIENT_CACHE_UNSOLVED_ENTRY_TMO	10
#define TCPIP_DNS_CLIENT_LOOKUP_RETRY_TMO			3
#define TCPIP_DNS_CLIENT_MAX_HOSTNAME_LEN			32
#define TCPIP_DNS_CLIENT_MAX_SELECT_INTERFACES		4
#define TCPIP_DNS_CLIENT_DELETE_OLD_ENTRIES			true
#define TCPIP_DNS_CLIENT_USER_NOTIFICATION   false





/*** ICMPv4 Server Configuration ***/
#define TCPIP_STACK_USE_ICMP_SERVER










/*** TCP Configuration ***/
#define TCPIP_TCP_MAX_SEG_SIZE_TX		        	1460
#define TCPIP_TCP_MAX_SEG_SIZE_RX_LOCAL		    		1460
#define TCPIP_TCP_MAX_SEG_SIZE_RX_NON_LOCAL			536
#define TCPIP_TCP_SOCKET_DEFAULT_TX_SIZE			512
#define TCPIP_TCP_SOCKET_DEFAULT_RX_SIZE			512
#define TCPIP_TCP_DYNAMIC_OPTIONS             			true
#define TCPIP_TCP_START_TIMEOUT_VAL		        	1000
#define TCPIP_TCP_DELAYED_ACK_TIMEOUT		    		100
#define TCPIP_TCP_FIN_WAIT_2_TIMEOUT		    		5000
#define TCPIP_TCP_KEEP_ALIVE_TIMEOUT		    		10000
#define TCPIP_TCP_CLOSE_WAIT_TIMEOUT		    		200
#define TCPIP_TCP_MAX_RETRIES		            		5
#define TCPIP_TCP_MAX_UNACKED_KEEP_ALIVES			6
#define TCPIP_TCP_MAX_SYN_RETRIES		        	3
#define TCPIP_TCP_AUTO_TRANSMIT_TIMEOUT_VAL			40
#define TCPIP_TCP_WINDOW_UPDATE_TIMEOUT_VAL			200
#define TCPIP_TCP_MAX_SOCKETS		                10
#define TCPIP_TCP_TASK_TICK_RATE		        	5
#define TCPIP_TCP_MSL_TIMEOUT		        	    0
#define TCPIP_TCP_QUIET_TIME		        	    0


/*** TCPIP MAC Configuration ***/
#define TCPIP_EMAC_TX_DESCRIPTORS				8
#define TCPIP_EMAC_RX_DESCRIPTORS				10
#define TCPIP_EMAC_RX_DEDICATED_BUFFERS				4
#define TCPIP_EMAC_RX_INIT_BUFFERS				    0
#define TCPIP_EMAC_RX_LOW_THRESHOLD				    1
#define TCPIP_EMAC_RX_LOW_FILL				        2
#define TCPIP_EMAC_RX_BUFF_SIZE		    			1536
#define TCPIP_EMAC_RX_MAX_FRAME		    			1536
#define TCPIP_EMAC_RX_FILTERS                       \
                                                    TCPIP_MAC_RX_FILTER_TYPE_BCAST_ACCEPT |\
                                                    TCPIP_MAC_RX_FILTER_TYPE_MCAST_ACCEPT |\
                                                    TCPIP_MAC_RX_FILTER_TYPE_UCAST_ACCEPT |\
                                                    TCPIP_MAC_RX_FILTER_TYPE_RUNT_REJECT |\
                                                    TCPIP_MAC_RX_FILTER_TYPE_CRC_ERROR_REJECT |\
                                                    0
#define TCPIP_EMAC_RX_FRAGMENTS		    			1
#define TCPIP_EMAC_ETH_OPEN_FLAGS       			\
                                                    TCPIP_ETH_OPEN_AUTO |\
                                                    TCPIP_ETH_OPEN_FDUPLEX |\
                                                    TCPIP_ETH_OPEN_HDUPLEX |\
                                                    TCPIP_ETH_OPEN_100 |\
                                                    TCPIP_ETH_OPEN_10 |\
                                                    TCPIP_ETH_OPEN_MDIX_AUTO |\
                                                    TCPIP_ETH_OPEN_RMII |\
                                                    0
#define TCPIP_EMAC_PHY_CONFIG_FLAGS     			\
                                                    DRV_ETHPHY_CFG_RMII | \
                                                    0                                                    
#define TCPIP_EMAC_PHY_LINK_INIT_DELAY  			500
#define TCPIP_EMAC_PHY_ADDRESS		    			0
#define TCPIP_EMAC_MODULE_ID		    			ETH_ID_0
#define TCPIP_EMAC_INTERRUPT_MODE        			true
#define DRV_ETHPHY_INSTANCES_NUMBER				1
#define DRV_ETHPHY_CLIENTS_NUMBER				1
#define DRV_ETHPHY_INDEX		        		1
#define DRV_ETHPHY_PERIPHERAL_ID				1
#define DRV_ETHPHY_NEG_INIT_TMO		    			1
#define DRV_ETHPHY_NEG_DONE_TMO		    			2000
#define DRV_ETHPHY_RESET_CLR_TMO				500
#define DRV_ETHMAC_INSTANCES_NUMBER				1
#define DRV_ETHMAC_CLIENTS_NUMBER				1
#define DRV_ETHMAC_INDEX	    	    			1
#define DRV_ETHMAC_PERIPHERAL_ID				1
#define DRV_ETHMAC_INTERRUPT_VECTOR				INT_VECTOR_ETHERNET
#define DRV_ETHMAC_INTERRUPT_SOURCE				INT_SOURCE_ETH_1
#define DRV_ETHMAC_POWER_STATE		    			SYS_MODULE_POWER_RUN_FULL

#define DRV_ETHMAC_INTERRUPT_MODE        			true



/*** UDP Configuration ***/
#define TCPIP_UDP_MAX_SOCKETS		                	10
#define TCPIP_UDP_SOCKET_DEFAULT_TX_SIZE		    	512
#define TCPIP_UDP_SOCKET_DEFAULT_TX_QUEUE_LIMIT    	 	3
#define TCPIP_UDP_SOCKET_DEFAULT_RX_QUEUE_LIMIT			3
#define TCPIP_UDP_USE_POOL_BUFFERS   false
#define TCPIP_UDP_USE_TX_CHECKSUM             			true

#define TCPIP_UDP_USE_RX_CHECKSUM             			true


/*** Network Configuration Index 0 ***/
#define TCPIP_NETWORK_DEFAULT_INTERFACE_NAME 			"PIC32INT"
#define TCPIP_IF_PIC32INT
#define TCPIP_NETWORK_DEFAULT_HOST_NAME 			"uRTU"
#define TCPIP_NETWORK_DEFAULT_MAC_ADDR 				"00:04:a3:11:22:33"
#define TCPIP_NETWORK_DEFAULT_IP_ADDRESS 			"192.168.1.70"
#define TCPIP_NETWORK_DEFAULT_IP_MASK 				"255.255.255.0"
#define TCPIP_NETWORK_DEFAULT_GATEWAY	 			"192.168.1.1"
#define TCPIP_NETWORK_DEFAULT_DNS 				"8.8.8.8"
#define TCPIP_NETWORK_DEFAULT_SECOND_DNS 			"8.8.4.4"
#define TCPIP_NETWORK_DEFAULT_POWER_MODE 			"full"
#define TCPIP_NETWORK_DEFAULT_INTERFACE_FLAGS                       \
                                                    TCPIP_NETWORK_CONFIG_DHCP_CLIENT_ON |\
                                                    TCPIP_NETWORK_CONFIG_DNS_CLIENT_ON |\
                                                    TCPIP_NETWORK_CONFIG_IP_STATIC
#define TCPIP_NETWORK_DEFAULT_MAC_DRIVER 		    DRV_ETHMAC_PIC32MACObject
#define TCPIP_NETWORK_DEFAULT_IPV6_ADDRESS 			0
#define TCPIP_NETWORK_DEFAULT_IPV6_PREFIX_LENGTH    0
#define TCPIP_NETWORK_DEFAULT_IPV6_GATEWAY 		    0


/*** IPv4 Configuration ***/

/*** OSAL Configuration ***/
#define OSAL_USE_RTOS          9

/* MPLAB Harmony Net Presentation Layer Definitions*/
#define NET_PRES_NUM_INSTANCE 1
#define NET_PRES_NUM_SOCKETS 10



// *****************************************************************************
// *****************************************************************************
// Section: Application Configuration
// *****************************************************************************
// *****************************************************************************

/*** Application Instance 0 Configuration ***/
#define APP_UART_TERM_DRV_USART                     0

/*** Application Instance 1 Configuration ***/

/*** Application Instance 2 Configuration ***/

/*** Application Instance 3 Configuration ***/
#define APP_LOGGER_DRV_USART                     1

//DOM-IGNORE-BEGIN
#ifdef __cplusplus
}
#endif
//DOM-IGNORE-END


#endif // _SYSTEM_CONFIG_H
/*******************************************************************************
 End of File
*/

