/**
 *   @file  cascade_lvds_stream.c
 *
 *   @brief
 *      Implements LVDS stream functionality.
 *
 *  \par
 *  NOTE:
 *      (C) Copyright 2019 - 2021 Texas Instruments, Inc.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *    Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 *    Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the
 *    distribution.
 *
 *    Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 *  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 *  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 *  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 *  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 *  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 *  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
 
/**************************************************************************
 *************************** Include Files ********************************
 **************************************************************************/
#include <stdint.h>
#include <string.h>
#include <stdio.h>

/* MCU+SDK Include Files */
#include <kernel/dpl/CacheP.h>

/* MMWAVE Cascade Include Files */
#include <ti/common/syscommon.h>
#include <ti/utils/test/cascade/am273x/cascade_lvds_stream.h>
#include <ti/utils/test/cascade/am273x/cascade_csirx.h>

/**************************************************************************
 ************************** Extern Definitions ****************************
 **************************************************************************/
extern MmwCascade_MCB    gMmwCascadeMCB;

/**************************************************************************
 *********************** LVDS Unit Test Functions **********************
 **************************************************************************/

/**
 *  @b Description
 *  @n
 *      Allocates Shawdow paramset
 */
static void allocateEDMAShadowChannel(EDMA_Handle handle, uint32_t *param)
{
    int32_t             testStatus = SystemP_SUCCESS;

    testStatus = EDMA_allocParam(handle, param);
    DebugP_assert(testStatus == SystemP_SUCCESS);

    return;
}

/**
 *  @b Description
 *  @n
 *      Allocates EDMA resource
 */
void allocateEDMAChannel(EDMA_Handle handle,
    uint32_t *dmaCh,
    uint32_t *tcc,
    uint32_t *param
)
{
    int32_t             testStatus = SystemP_SUCCESS;
    uint32_t            baseAddr, regionId;
    EDMA_Config        *config;
    EDMA_Object        *object;

    config = (EDMA_Config *) handle;
    object = config->object;

    if((object->allocResource.dmaCh[*dmaCh/32] & (1U << *dmaCh%32)) != (1U << *dmaCh%32))
    {
        testStatus = EDMA_allocDmaChannel(handle, dmaCh);
        DebugP_assert(testStatus == SystemP_SUCCESS);

        testStatus = EDMA_allocTcc(handle, tcc);
        DebugP_assert(testStatus == SystemP_SUCCESS);

        testStatus = EDMA_allocParam(handle, param);
        DebugP_assert(testStatus == SystemP_SUCCESS);

        baseAddr = EDMA_getBaseAddr(handle);
        DebugP_assert(baseAddr != 0);

        regionId = EDMA_getRegionId(handle);
        DebugP_assert(regionId < SOC_EDMA_NUM_REGIONS);

        /* Request channel */
        EDMA_configureChannelRegion(baseAddr, regionId, EDMA_CHANNEL_TYPE_DMA,
            *dmaCh, *tcc, *param, 0);
   }

    return;
}

/**
*  @b Description
*  @n
*      This function initializes/configures the LVDS
*      streaming EDMA resources.
*
*  @retval
*      Success -   0
*  @retval
*      Error   -   <0
*/
void Cascade_LVDSStream_EDMAInit (void)
{
    uint32_t index = 0;

    gMmwCascadeMCB.lvdsStreamcfg.lvdsStream.swSessionEDMAChannelAllocatorIndex = 0;

    /* Populate the LVDS Stream SW Session EDMA Channel Table: */
    gMmwCascadeMCB.lvdsStreamcfg.lvdsStream.swSessionEDMAChannelTable[0].chainChannelsId       = CASCADE_LVDS_STREAM_SW_SESSION_EDMA_CH_0;
    gMmwCascadeMCB.lvdsStreamcfg.lvdsStream.swSessionEDMAChannelTable[0].shadowLinkChannelsId  = CASCADE_LVDS_STREAM_SW_SESSION_EDMA_SHADOW_CH_0;
    gMmwCascadeMCB.lvdsStreamcfg.lvdsStream.swSessionEDMAChannelTable[1].chainChannelsId       = CASCADE_LVDS_STREAM_SW_SESSION_EDMA_CH_1;
    gMmwCascadeMCB.lvdsStreamcfg.lvdsStream.swSessionEDMAChannelTable[1].shadowLinkChannelsId  = CASCADE_LVDS_STREAM_SW_SESSION_EDMA_SHADOW_CH_1;
    gMmwCascadeMCB.lvdsStreamcfg.lvdsStream.swSessionEDMAChannelTable[2].chainChannelsId       = CASCADE_LVDS_STREAM_SW_SESSION_EDMA_CH_2;
    gMmwCascadeMCB.lvdsStreamcfg.lvdsStream.swSessionEDMAChannelTable[2].shadowLinkChannelsId  = CASCADE_LVDS_STREAM_SW_SESSION_EDMA_SHADOW_CH_2;

    for (index=0; index < 3; index++)
    {
        allocateEDMAChannel(gMmwCascadeMCB.lvdsStreamcfg.edmaHandle, &gMmwCascadeMCB.lvdsStreamcfg.lvdsStream.swSessionEDMAChannelTable[index].chainChannelsId,
                            &gMmwCascadeMCB.lvdsStreamcfg.lvdsStream.swSessionEDMAChannelTable[index].chainChannelsId,
                            &gMmwCascadeMCB.lvdsStreamcfg.lvdsStream.swSessionEDMAChannelTable[index].chainChannelsId);

        allocateEDMAShadowChannel(gMmwCascadeMCB.lvdsStreamcfg.edmaHandle, &gMmwCascadeMCB.lvdsStreamcfg.lvdsStream.swSessionEDMAChannelTable[index].shadowLinkChannelsId);
    }
}

 /**
 *  @b Description
 *  @n
 *      This is the LVDS streaming init function. 
 *      It initializes the necessary modules
 *      that implement the streaming.
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
int32_t Cascade_LVDSStreamInit (void)
{
    CBUFF_InitCfg           initCfg;
    int32_t                 status = SystemP_SUCCESS;
    int32_t                 errCode;

    /*************************************************************************************
     * Open the CBUFF Driver:
     *************************************************************************************/
    memset ((void *)&initCfg, 0, sizeof(CBUFF_InitCfg));

    /* Populate the configuration: */
    initCfg.enableECC                 = 0U;
    initCfg.crcEnable                 = 0U;
    /* Up to 1 SW session + 1 HW session can be configured for each frame. Therefore max session is 2. */
    initCfg.maxSessions               = 2U;
    initCfg.enableDebugMode           = false;
    initCfg.interface                 = CBUFF_Interface_LVDS;
    initCfg.outputDataFmt             = CBUFF_OutputDataFmt_16bit;
    initCfg.lvdsCfg.crcEnable         = 0U;
    initCfg.lvdsCfg.msbFirst          = 1U;
    /* Enable all lanes available on the platform*/
    initCfg.lvdsCfg.lvdsLaneEnable    = 0xFU;
    initCfg.lvdsCfg.ddrClockMode      = 1U;
    initCfg.lvdsCfg.ddrClockModeMux   = 1U;

    /* Initialize the CBUFF Driver: */
    gMmwCascadeMCB.lvdsStreamcfg.lvdsStream.cbuffHandle = CBUFF_open (&initCfg, &errCode);
    if (gMmwCascadeMCB.lvdsStreamcfg.lvdsStream.cbuffHandle == NULL)
    {
        /* Error: Unable to initialize the CBUFF Driver */
        test_print("Error: CBUFF_open failed with [Error=%d]\n", errCode);
        DebugP_assert(0);
    }

    /* Populate EDMA resources */
    Cascade_LVDSStream_EDMAInit();

    return status;
}

/**
 *  @b Description
 *  @n
 *      Function that allocates CBUFF-EDMA channel
 *
 *  @param[in]  ptrEDMAInfo
 *      Pointer to the EDMA Information
 *  @param[out]  ptrEDMACfg
 *      Populated EDMA channel configuration
 *
 */
static void Cascade_LVDSStream_EDMAAllocateCBUFFChannel
(
    CBUFF_EDMAInfo*         ptrEDMAInfo,
    CBUFF_EDMAChannelCfg*   ptrEDMACfg
)
{
    if(ptrEDMAInfo->dmaNum == 0)
    {
        ptrEDMACfg->chainChannelsId      = CASCADE_LVDS_STREAM_CBUFF_EDMA_CH_0;

        allocateEDMAChannel(gMmwCascadeMCB.lvdsStreamcfg.edmaHandle, &ptrEDMACfg->chainChannelsId,
                            &ptrEDMACfg->chainChannelsId, &ptrEDMACfg->chainChannelsId);

        ptrEDMACfg->shadowLinkChannelsId = CASCADE_LVDS_STREAM_CBUFF_EDMA_SHADOW_CH_0;

        allocateEDMAShadowChannel(gMmwCascadeMCB.lvdsStreamcfg.edmaHandle, &ptrEDMACfg->shadowLinkChannelsId);
    }
    else if(ptrEDMAInfo->dmaNum == 1)
    {
        ptrEDMACfg->chainChannelsId      = CASCADE_LVDS_STREAM_CBUFF_EDMA_CH_1;

        allocateEDMAChannel(gMmwCascadeMCB.lvdsStreamcfg.edmaHandle, &ptrEDMACfg->chainChannelsId,
                            &ptrEDMACfg->chainChannelsId, &ptrEDMACfg->chainChannelsId);

        ptrEDMACfg->shadowLinkChannelsId = CASCADE_LVDS_STREAM_CBUFF_EDMA_SHADOW_CH_1;

        allocateEDMAShadowChannel(gMmwCascadeMCB.lvdsStreamcfg.edmaHandle, &ptrEDMACfg->shadowLinkChannelsId);
    }    
    else
    {
        /* Max of 2 CBUFF sessions can be configured*/
        DebugP_assert (0);
    }
}


/**
 *  @b Description
 *  @n
 *      This is the registered CBUFF EDMA channel allocation function
 *      which allocates EDMA channels for CBUFF SW Session
 *
 *  @param[in]  ptrEDMAInfo
 *      Pointer to the EDMA Information
 *  @param[out]  ptrEDMACfg
 *      Populated EDMA channel configuration
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
static int32_t Cascade_LVDSStream_EDMAAllocateCBUFFSwChannel
(
    CBUFF_EDMAInfo*         ptrEDMAInfo,
    CBUFF_EDMAChannelCfg*   ptrEDMACfg
)
{
    int32_t         retVal = MINUS_ONE;
    Cascade_LVDSStreamCfg *streamMCBPtr =  &gMmwCascadeMCB.lvdsStreamcfg.lvdsStream;

    if(ptrEDMAInfo->isFirstEDMAChannel)
    {
        Cascade_LVDSStream_EDMAAllocateCBUFFChannel(ptrEDMAInfo,ptrEDMACfg);
        retVal = 0;
    }
    else
    {
        /* Sanity Check: Are there sufficient EDMA channels? */
        if (streamMCBPtr->swSessionEDMAChannelAllocatorIndex >= CASCADE_LVDS_STREAM_SW_SESSION_MAX_EDMA_CHANNEL)
        {
            /* Error: All the EDMA channels are allocated */
            test_print ("Error: Cascade_LVDSStream_EDMAAllocateCBUFFChannel failed. SW channel index=%d\n", 
                            streamMCBPtr->swSessionEDMAChannelAllocatorIndex);
            goto exit;
        }
        
        /* Copy over the allocated EDMA configuration. */
        memcpy ((void *)ptrEDMACfg,
                (void*)&streamMCBPtr->swSessionEDMAChannelTable[streamMCBPtr->swSessionEDMAChannelAllocatorIndex],
                sizeof(CBUFF_EDMAChannelCfg));
        
        /* Increment the allocator index: */
        streamMCBPtr->swSessionEDMAChannelAllocatorIndex++;
        
        /* EDMA Channel allocated successfully */
        retVal = 0;
    }    

exit:
    return retVal;
}


/**
 *  @b Description
 *  @n
 *      This is the registered CBUFF EDMA channel free function which frees EDMA channels
 *      which had been allocated for use by a CBUFF SW Session
 *
 *  @retval
 *      Not applicable
 */
static void Cascade_LVDSStream_EDMAFreeCBUFFSwChannel (CBUFF_EDMAChannelCfg* ptrEDMACfg)
{
    uint8_t    index;
    Cascade_LVDSStreamCfg *streamMCBPtr =  &gMmwCascadeMCB.lvdsStreamcfg.lvdsStream;

    if((ptrEDMACfg->chainChannelsId == CASCADE_LVDS_STREAM_CBUFF_EDMA_CH_0) ||
       (ptrEDMACfg->chainChannelsId == CASCADE_LVDS_STREAM_CBUFF_EDMA_CH_1))
    {
        /*This is the CBUFF trigger channel. It is not part of the resource table so
          nothing needs to be done*/
        goto exit;  
    }

    for (index = 0U; index < CASCADE_LVDS_STREAM_SW_SESSION_MAX_EDMA_CHANNEL; index++)
    {
        /* Do we have a match? */
        if (memcmp ((void*)ptrEDMACfg,
                    (void*)&streamMCBPtr->swSessionEDMAChannelTable[index],
                    sizeof(CBUFF_EDMAChannelCfg)) == 0)
        {
            /* Yes: Decrement the SW Session index */
            streamMCBPtr->swSessionEDMAChannelAllocatorIndex--;
            goto exit;
        }
    }

    /* Sanity Check: We should have had a match. An assertion is thrown to indicate that the EDMA channel
     * being cleaned up does not belong to the table*/
    DebugP_assert (0);

exit:
    return;
}


/**
 *  @b Description
 *  @n
 *      This function deletes the SW session and any HSI
 *      header associated with it. 
 *
 *  @retval
 *      Not applicable
 */
void Cascade_LVDSStreamDeleteSwSession (void)
{
    int32_t     errCode;
    Cascade_LVDSStreamCfg* streamMcb = &gMmwCascadeMCB.lvdsStreamcfg.lvdsStream;
    
    /* Delete session*/
    if (CBUFF_close (streamMcb->swSessionHandle, &errCode) < 0)
    {
        /* Error: Unable to delete the session. */
        test_print ("Error: Cascade_LVDSStreamDeleteSwSession CBUFF_close failed. Error code %d\n", errCode);
        DebugP_assert(0);
        return;
    }

    streamMcb->swSessionHandle = NULL;

    return;
}


/**
 *  @b Description
 *  @n
 *      This is the registered callback function which is invoked after the
 *      frame done interrupt is received for the SW session.
 *
 *  @param[in]  sessionHandle
 *      Handle to the session
 *
 *  @retval
 *      Not applicable
 */
static void Cascade_LVDSStream_SwTriggerFrameDone (CBUFF_SessionHandle sessionHandle)
{
    /* Increment stats*/
    gMmwCascadeMCB.lvdsStreamcfg.lvdsStream.swFrameDoneCount++;

    return;
}


/**
 *  @b Description
 *  @n
 *      This is the LVDS sw streaming config function.
 *      It configures the sw session for the LVDS streaming.
 *
 *  @param[in]  numObjOut      Number of detected objects to stream out
 *  @param[in]  objOut         Pointer to detected objects point cloud
 *  @param[in]  objOutSideInfo Pointer to detected objects side information
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
int32_t Cascade_LVDSStreamSwConfig (uint32_t userBuffer0Addr, uint32_t userBuffer1Addr, uint32_t dataSize)
{
    CBUFF_SessionCfg          sessionCfg;
    Cascade_LVDSStreamCfg* streamMcb = &gMmwCascadeMCB.lvdsStreamcfg.lvdsStream;
    int32_t                   errCode;
    int32_t                   retVal = MINUS_ONE;

    memset ((void*)&sessionCfg, 0, sizeof(CBUFF_SessionCfg));

    CacheP_wb((void*)&sessionCfg, sizeof(CBUFF_SessionCfg), CacheP_TYPE_ALLD);

    /* Populate the configuration: */
    sessionCfg.edmaHandle                        = gMmwCascadeMCB.lvdsStreamcfg.edmaHandle;
    sessionCfg.allocateEDMAChannelFxn            = Cascade_LVDSStream_EDMAAllocateCBUFFSwChannel;
    sessionCfg.freeEDMAChannelFxn                = Cascade_LVDSStream_EDMAFreeCBUFFSwChannel;
    sessionCfg.frameDoneCallbackFxn              = Cascade_LVDSStream_SwTriggerFrameDone;
    sessionCfg.dataType                          = CBUFF_DataType_REAL; 
    sessionCfg.swCfg.userBufferInfo[0].size    = (dataSize/2); // CBUFF units (16-bit data)
    sessionCfg.swCfg.userBufferInfo[0].address = (uint32_t)(userBuffer0Addr);

    /* Configure second CBUFF buffer if cascade is enabled. */
    if(MMWAVE_RADAR_DEVICES > 1)
    {
        sessionCfg.swCfg.userBufferInfo[1].size    = (dataSize/2); // CBUFF units (16-bit data)
        sessionCfg.swCfg.userBufferInfo[1].address = (uint32_t)(userBuffer1Addr);
    }

    /* Create the SW Session. */
    streamMcb->swSessionHandle = CBUFF_createSession (gMmwCascadeMCB.lvdsStreamcfg.lvdsStream.cbuffHandle, &sessionCfg, &errCode);

    if (streamMcb->swSessionHandle == NULL)
    {
        /* Error: Unable to create the CBUFF SW session */
        test_print("Error: Cascade_LVDSStreamSwConfig unable to create the CBUFF SW session with [Error=%d]\n", errCode);
        goto exit;
    }

    /* Control comes here implies that the LVDS Stream has been configured successfully */
    retVal = 0;

exit:
    return retVal;
}
