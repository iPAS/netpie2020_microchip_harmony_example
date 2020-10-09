/*******************************************************************************
  SPI Driver Functions for Static Driver Tasks Functions

  Company:
    Microchip Technology Inc.

  File Name:
    drv_spi_static_dma_tasks.c

  Summary:
    SPI driver tasks functions

  Description:
    The SPI device driver provides a simple interface to manage the SPI
    modules on Microchip microcontrollers. This file contains implemenation
    for the SPI driver.

  Remarks:
  This file is generated from framework/driver/spi/template/drv_spi_static_dma_tasks.c.ftl
*******************************************************************************/

//DOM-IGNORE-BEGIN
/*******************************************************************************
Copyright (c) 2015 released Microchip Technology Inc.  All rights reserved.

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
//DOM-IGNORE-END

#include "system_config.h"
#include "system_definitions.h"

void DRV_SPI0_MasterDMASendDummy8BitISR(SYS_DMA_TRANSFER_EVENT event, SYS_DMA_CHANNEL_HANDLE handle, uintptr_t contextHandle)
{
    if (event != SYS_DMA_TRANSFER_EVENT_COMPLETE)
    {
        // Ignore for now
        return;
    }
    struct DRV_SPI_OBJ * pDrvObj = (struct DRV_SPI_OBJ *)contextHandle;
    DRV_SPI_JOB_OBJECT * currentJob = pDrvObj->currentJob;
    if (currentJob->dummyLeftToTx != 0)
    {
        uint8_t * ptr = sDrvSpiTxDummy;
        uint32_t len = MIN(MIN(MIN(PLIB_DMA_MAX_TRF_SIZE, currentJob->dummyLeftToTx), DRV_SPI_DMA_TXFER_SIZE), DRV_SPI_DMA_DUMMY_BUFFER_SIZE);
        void * spiPtr = PLIB_SPI_BufferAddressGet(SPI_ID_1);
        SYS_DMA_ChannelTransferAdd(pDrvObj->txDmaChannelHandle, ptr, len, spiPtr, 1, 1);
        currentJob->txDMAProgressStage = DRV_SPI_DMA_DUMMY_INPROGRESS;
        currentJob->dummyLeftToTx -= len;
    }
    else
    {
        // Job is done
        currentJob->txDMAProgressStage = DRV_SPI_DMA_COMPLETE;
    }

}


void DRV_SPI0_MasterDMASendData8BitISR(SYS_DMA_TRANSFER_EVENT event, SYS_DMA_CHANNEL_HANDLE handle, uintptr_t contextHandle)
{
    if (event != SYS_DMA_TRANSFER_EVENT_COMPLETE)
    {
        // Ignore for now
        return;
    }
    struct DRV_SPI_OBJ * pDrvObj = (struct DRV_SPI_OBJ *)contextHandle;
    DRV_SPI_JOB_OBJECT * currentJob = pDrvObj->currentJob;
    if (currentJob->dataLeftToTx != 0)
    {
        uint8_t * ptr = &(currentJob->txBuffer[currentJob->dataTxed]);
        uint32_t len = MIN(MIN(PLIB_DMA_MAX_TRF_SIZE, currentJob->dataLeftToTx), DRV_SPI_DMA_TXFER_SIZE);
        void * spiPtr = PLIB_SPI_BufferAddressGet(SPI_ID_1);
        SYS_DMA_ChannelTransferAdd(pDrvObj->txDmaChannelHandle, ptr, len, spiPtr, 1, 1);
        currentJob->txDMAProgressStage = DRV_SPI_DMA_DATA_INPROGRESS;
        currentJob->dataLeftToTx -= len;
        currentJob->dataTxed += len;
    }
    else
    {
        // Job is done
        currentJob->txDMAProgressStage = DRV_SPI_DMA_COMPLETE;
        if (currentJob->dummyLeftToTx != 0)
        {
        SYS_INT_SourceEnable(INT_SOURCE_SPI_1_TRANSMIT);
        }
    }
}


void DRV_SPI0_ISRDMAMasterSendEventHandler8bit(SYS_DMA_TRANSFER_EVENT event, SYS_DMA_CHANNEL_HANDLE handle, uintptr_t contextHandle)
{
    struct DRV_SPI_OBJ * pDrvObj = (struct DRV_SPI_OBJ * )contextHandle;
    register DRV_SPI_JOB_OBJECT * currentJob = pDrvObj->currentJob;

    switch(currentJob->txDMAProgressStage)
    {
        case DRV_SPI_DMA_DATA_INPROGRESS:
            DRV_SPI0_MasterDMASendData8BitISR(event, handle, contextHandle);
            break;
        case DRV_SPI_DMA_DUMMY_INPROGRESS:
            DRV_SPI0_MasterDMASendDummy8BitISR(event, handle, contextHandle);
            break;
        default:
            break;
    }
}


void DRV_SPI0_MasterDMAReceiveDummy8BitISR(SYS_DMA_TRANSFER_EVENT event, SYS_DMA_CHANNEL_HANDLE handle, uintptr_t contextHandle)
{
    if (event != SYS_DMA_TRANSFER_EVENT_COMPLETE)
    {
        // Ignore for now
        return;
    }
    struct DRV_SPI_OBJ * pDrvObj = (struct DRV_SPI_OBJ *)contextHandle;
    DRV_SPI_JOB_OBJECT * currentJob = pDrvObj->currentJob;
    if (currentJob->dummyLeftToRx != 0)
    {
        uint8_t * ptr = sDrvSpiRxDummy;
        uint32_t len = MIN(MIN(MIN(PLIB_DMA_MAX_TRF_SIZE, currentJob->dummyLeftToRx), DRV_SPI_DMA_TXFER_SIZE), DRV_SPI_DMA_DUMMY_BUFFER_SIZE);
        void * spiPtr = PLIB_SPI_BufferAddressGet(SPI_ID_1);
        SYS_DMA_ChannelTransferAdd(pDrvObj->rxDmaChannelHandle, spiPtr, 1, ptr, len, 1);
        currentJob->rxDMAProgressStage = DRV_SPI_DMA_DUMMY_INPROGRESS;
        currentJob->dummyLeftToRx -= len;
    }
    else
    {
        // Job is done
        currentJob->rxDMAProgressStage = DRV_SPI_DMA_COMPLETE;
        SYS_INT_SourceEnable(INT_SOURCE_SPI_1_RECEIVE);
    }
}


void DRV_SPI0_MasterDMAReceiveData8BitISR(SYS_DMA_TRANSFER_EVENT event, SYS_DMA_CHANNEL_HANDLE handle, uintptr_t contextHandle)
{
    if (event != SYS_DMA_TRANSFER_EVENT_COMPLETE)
    {
        // Ignore for now
        return;
    }
    struct DRV_SPI_OBJ * pDrvObj = (struct DRV_SPI_OBJ *)contextHandle;
    DRV_SPI_JOB_OBJECT * currentJob = pDrvObj->currentJob;
    if (currentJob->dataLeftToRx != 0)
    {
        uint8_t * ptr = &(currentJob->rxBuffer[currentJob->dataRxed]);
        uint32_t len = MIN(MIN(PLIB_DMA_MAX_TRF_SIZE, currentJob->dataLeftToRx), DRV_SPI_DMA_TXFER_SIZE);
        void * spiPtr = PLIB_SPI_BufferAddressGet(SPI_ID_1);
        SYS_DMA_ChannelTransferAdd(pDrvObj->rxDmaChannelHandle, spiPtr, 1, ptr, len, 1);
        currentJob->rxDMAProgressStage = DRV_SPI_DMA_DATA_INPROGRESS;
        currentJob->dataLeftToRx -= len;
        currentJob->dataRxed += len;
    }
    else
    {
        // Job is done
        currentJob->rxDMAProgressStage = DRV_SPI_DMA_COMPLETE;
        SYS_INT_SourceEnable(INT_SOURCE_SPI_1_RECEIVE);
    }
}


void DRV_SPI0_ISRDMAMasterReceiveEventHandler8bit(SYS_DMA_TRANSFER_EVENT event, SYS_DMA_CHANNEL_HANDLE handle, uintptr_t contextHandle)
{
    struct DRV_SPI_OBJ * pDrvObj = (struct DRV_SPI_OBJ * )contextHandle;
    register DRV_SPI_JOB_OBJECT * currentJob = pDrvObj->currentJob;

    switch(currentJob->rxDMAProgressStage)
    {
        case DRV_SPI_DMA_DATA_INPROGRESS:
            DRV_SPI0_MasterDMAReceiveData8BitISR(event, handle, contextHandle);
            break;
        case DRV_SPI_DMA_DUMMY_INPROGRESS:
            DRV_SPI0_MasterDMAReceiveDummy8BitISR(event, handle, contextHandle);
            break;
        default:
            break;
    }
}

