/*******************************************************************************
 System Interrupt Source File

  File Name:
    sys_interrupt_a.S

  Summary:
    Raw ISR definitions.

  Description:
    This file contains a definitions of the raw ISRs required to support the 
    interrupt sub-system.
*******************************************************************************/

// DOM-IGNORE-BEGIN
/*******************************************************************************
Copyright (c) 2011-2014 released Microchip Technology Inc.  All rights reserved.

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
/*
*********************************************************************************************************
*                                           INCLUDES
*********************************************************************************************************
*/
#include <xc.h>



#include "ISR_Support.h"


/* External Interrupt Instance 0 Interrupt */
   .extern  IntHandlerExternalInterruptInstance0

   .section	.vector_8,code, keep
   .equ     __vector_dispatch_8, IntVectorExternalInterruptInstance0
   .global  __vector_dispatch_8
   .set     nomicromips
   .set     noreorder
   .set     nomips16
   .set     noat
   .ent  IntVectorExternalInterruptInstance0

IntVectorExternalInterruptInstance0:
    portSAVE_CONTEXT
    la    s6,  IntHandlerExternalInterruptInstance0
    jalr  s6
    nop
    portRESTORE_CONTEXT
    .end	IntVectorExternalInterruptInstance0



/* TMR Instance 0 Interrupt */
   .extern  IntHandlerDrvTmrInstance0

   .section	.vector_14,code, keep
   .equ     __vector_dispatch_14, IntVectorDrvTmrInstance0
   .global  __vector_dispatch_14
   .set     nomicromips
   .set     noreorder
   .set     nomips16
   .set     noat
   .ent  IntVectorDrvTmrInstance0

IntVectorDrvTmrInstance0:
    portSAVE_CONTEXT
    la    s6,  IntHandlerDrvTmrInstance0
    jalr  s6
    nop
    portRESTORE_CONTEXT
    .end	IntVectorDrvTmrInstance0




/* USART Instance 0 Interrupt */

   .extern  IntHandlerDrvUsartReceiveInstance0

   .section	.vector_189,code, keep
   .equ     __vector_dispatch_189, IntVectorDrvUsartReceiveInstance0
   .global  __vector_dispatch_189
   .set     nomicromips
   .set     noreorder
   .set     nomips16
   .set     noat
   .ent  IntVectorDrvUsartReceiveInstance0

IntVectorDrvUsartReceiveInstance0:
    portSAVE_CONTEXT
    la    s6,  IntHandlerDrvUsartReceiveInstance0
    jalr  s6
    nop
    portRESTORE_CONTEXT
    .end	IntVectorDrvUsartReceiveInstance0


   .extern  IntHandlerDrvUsartTransmitInstance0

   .section	.vector_190,code, keep
   .equ     __vector_dispatch_190, IntVectorDrvUsartTransmitInstance0
   .global  __vector_dispatch_190
   .set     nomicromips
   .set     noreorder
   .set     nomips16
   .set     noat
   .ent  IntVectorDrvUsartTransmitInstance0

IntVectorDrvUsartTransmitInstance0:
    portSAVE_CONTEXT
    la    s6,  IntHandlerDrvUsartTransmitInstance0
    jalr  s6
    nop
    portRESTORE_CONTEXT
    .end	IntVectorDrvUsartTransmitInstance0


   .extern  IntHandlerDrvUsartErrorInstance0

   .section	.vector_188,code, keep
   .equ     __vector_dispatch_188, IntVectorDrvUsartErrorInstance0
   .global  __vector_dispatch_188
   .set     nomicromips
   .set     noreorder
   .set     nomips16
   .set     noat
   .ent  IntVectorDrvUsartErrorInstance0

IntVectorDrvUsartErrorInstance0:
    portSAVE_CONTEXT
    la    s6,  IntHandlerDrvUsartErrorInstance0
    jalr  s6
    nop
    portRESTORE_CONTEXT
    .end	IntVectorDrvUsartErrorInstance0



/* USART Instance 1 Interrupt */

   .extern  IntHandlerDrvUsartReceiveInstance1

   .section	.vector_146,code, keep
   .equ     __vector_dispatch_146, IntVectorDrvUsartReceiveInstance1
   .global  __vector_dispatch_146
   .set     nomicromips
   .set     noreorder
   .set     nomips16
   .set     noat
   .ent  IntVectorDrvUsartReceiveInstance1

IntVectorDrvUsartReceiveInstance1:
    portSAVE_CONTEXT
    la    s6,  IntHandlerDrvUsartReceiveInstance1
    jalr  s6
    nop
    portRESTORE_CONTEXT
    .end	IntVectorDrvUsartReceiveInstance1


   .extern  IntHandlerDrvUsartTransmitInstance1

   .section	.vector_147,code, keep
   .equ     __vector_dispatch_147, IntVectorDrvUsartTransmitInstance1
   .global  __vector_dispatch_147
   .set     nomicromips
   .set     noreorder
   .set     nomips16
   .set     noat
   .ent  IntVectorDrvUsartTransmitInstance1

IntVectorDrvUsartTransmitInstance1:
    portSAVE_CONTEXT
    la    s6,  IntHandlerDrvUsartTransmitInstance1
    jalr  s6
    nop
    portRESTORE_CONTEXT
    .end	IntVectorDrvUsartTransmitInstance1


   .extern  IntHandlerDrvUsartErrorInstance1

   .section	.vector_145,code, keep
   .equ     __vector_dispatch_145, IntVectorDrvUsartErrorInstance1
   .global  __vector_dispatch_145
   .set     nomicromips
   .set     noreorder
   .set     nomips16
   .set     noat
   .ent  IntVectorDrvUsartErrorInstance1

IntVectorDrvUsartErrorInstance1:
    portSAVE_CONTEXT
    la    s6,  IntHandlerDrvUsartErrorInstance1
    jalr  s6
    nop
    portRESTORE_CONTEXT
    .end	IntVectorDrvUsartErrorInstance1



/* SPI Instance 0 Interrupt */
   .extern  IntHandlerSPIRxInstance0

   .section	.vector_110,code, keep
   .equ     __vector_dispatch_110, IntVectorSPIRxInstance0
   .global  __vector_dispatch_110
   .set     nomicromips
   .set     noreorder
   .set     nomips16
   .set     noat
   .ent  IntVectorSPIRxInstance0

IntVectorSPIRxInstance0:
    portSAVE_CONTEXT
    la    s6,  IntHandlerSPIRxInstance0
    jalr  s6
    nop
    portRESTORE_CONTEXT
    .end	IntVectorSPIRxInstance0


   .extern  IntHandlerSPITxInstance0

   .section	.vector_111,code, keep
   .equ     __vector_dispatch_111, IntVectorSPITxInstance0
   .global  __vector_dispatch_111
   .set     nomicromips
   .set     noreorder
   .set     nomips16
   .set     noat
   .ent  IntVectorSPITxInstance0

IntVectorSPITxInstance0:
    portSAVE_CONTEXT
    la    s6,  IntHandlerSPITxInstance0
    jalr  s6
    nop
    portRESTORE_CONTEXT
    .end	IntVectorSPITxInstance0


   .extern  IntHandlerSPIFaultInstance0

   .section	.vector_109,code, keep
   .equ     __vector_dispatch_109, IntVectorSPIFaultInstance0
   .global  __vector_dispatch_109
   .set     nomicromips
   .set     noreorder
   .set     nomips16
   .set     noat
   .ent  IntVectorSPIFaultInstance0

IntVectorSPIFaultInstance0:
    portSAVE_CONTEXT
    la    s6,  IntHandlerSPIFaultInstance0
    jalr  s6
    nop
    portRESTORE_CONTEXT
    .end	IntVectorSPIFaultInstance0



/* Sytem DMA Instance 0 Interrupt */
   .extern  IntHandlerSysDmaInstance0

   .section	.vector_134,code, keep
   .equ     __vector_dispatch_134, IntVectorSysDmaInstance0
   .global  __vector_dispatch_134
   .set     nomicromips
   .set     noreorder
   .set     nomips16
   .set     noat
   .ent  IntVectorSysDmaInstance0

IntVectorSysDmaInstance0:
    portSAVE_CONTEXT
    la    s6,  IntHandlerSysDmaInstance0
    jalr  s6
    nop
    portRESTORE_CONTEXT
    .end	IntVectorSysDmaInstance0


/* Sytem DMA Instance 1 Interrupt */
   .extern  IntHandlerSysDmaInstance1

   .section	.vector_135,code, keep
   .equ     __vector_dispatch_135, IntVectorSysDmaInstance1
   .global  __vector_dispatch_135
   .set     nomicromips
   .set     noreorder
   .set     nomips16
   .set     noat
   .ent  IntVectorSysDmaInstance1

IntVectorSysDmaInstance1:
    portSAVE_CONTEXT
    la    s6,  IntHandlerSysDmaInstance1
    jalr  s6
    nop
    portRESTORE_CONTEXT
    .end	IntVectorSysDmaInstance1


/* ETHMAC Interrupt */
   .extern  IntHandler_ETHMAC

   .section	.vector_153,code, keep
   .equ     __vector_dispatch_153, IntVector_ETHMAC
   .global  __vector_dispatch_153
   .set     nomicromips
   .set     noreorder
   .set     nomips16
   .set     noat
   .ent  IntVector_ETHMAC

IntVector_ETHMAC:
    portSAVE_CONTEXT
    la    s6,  IntHandler_ETHMAC
    jalr  s6
    nop
    portRESTORE_CONTEXT
    .end	IntVector_ETHMAC



/*******************************************************************************
 End of File
 */

