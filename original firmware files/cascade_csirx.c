/*
 *   @file  cascade_csirx.c
 *
 *   @brief
 *      Unit Test code for the MMWAVE Cascade 
 *
 *  \par
 *  NOTE:
 *      (C) Copyright 2021 Texas Instruments, Inc.
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

/* Standard Include Files. */
#include <stdint.h>
#include <stdlib.h>
#include <stddef.h>
#include <string.h>
#include <stdio.h>

#include <kernel/dpl/AddrTranslateP.h>
#include <kernel/dpl/SemaphoreP.h>
#include <kernel/dpl/ClockP.h>
#include <ti/utils/test/cascade/am273x/mssgenerated/ti_drivers_open_close.h>

#include <ti/control/mmwavelink/mmwavelink.h>
#include <ti/utils/test/cascade/am273x/cascade_csirx.h>
#include <ti/common/syscommon.h>

#define CBUFF_CONFIG_REG_0                       (0x06040000U)

uint32_t csirx0CommonCallBackArg = (uint32_t)CONFIG_CSIRX0;
uint32_t csirx1CommonCallBackArg = (uint32_t)CONFIG_CSIRX1;

/**************************************************************************
 ************************** Extern Definitions ****************************
 **************************************************************************/
extern MmwCascade_MCB    gMmwCascadeMCB;
extern MmwCascade_CSIRX_State gCSIRXState[MMWAVE_RADAR_DEVICES];

extern uint32_t gCSIRXErrorCode[MMWAVE_RADAR_DEVICES];
extern uint8_t CSIA_PingBuf[BOARD_DIAG_PING_OR_PONG_BUF_SIZE_ALIGNED];
extern uint8_t CSIA_PongBuf[BOARD_DIAG_PING_OR_PONG_BUF_SIZE_ALIGNED];
extern uint8_t CSIB_PingBuf[BOARD_DIAG_PING_OR_PONG_BUF_SIZE_ALIGNED];
extern uint8_t CSIB_PongBuf[BOARD_DIAG_PING_OR_PONG_BUF_SIZE_ALIGNED];

extern void configureTransfer(void);

/**************************************************************************
 *********************** Cascade Unit Test Functions **********************
 **************************************************************************/
/**
 *  @b Description
 *  @n
 *      Callback function for common.irq interrupt, generated when
 *      end of frame code and line code detected, as per the set configuration.
 * 
 *  @param[in] handle      CSIRX Handle
 *  @param[in] arg         Callback function argument
 *  @param[out] IRQ        CSIRX common irq
 *
 *  @retval
 *      Not Applicable.
 */
void MmwCascade_csirxCommonCallback(CSIRX_Handle handle, void *arg,
                              struct CSIRX_CommonIntr_s *IRQ)
{
    uint8_t i;
    uint32_t csiRxInstance = *((uint32_t*)arg);
    uint32_t frameCounter = gCSIRXState[csiRxInstance].contextIRQcounts[csiRxInstance].frameEndCodeDetect + 1;

    DebugP_assert (handle != NULL);

    gCSIRXState[csiRxInstance].callbackCount.common++;

    gCSIRXState[csiRxInstance].IRQ.common = *IRQ;

    /* Counts book-keeping */
    if(IRQ->isOcpError == true)
    {
        gCSIRXState[csiRxInstance].commonIRQcount.isOCPerror++;
    }
    if(IRQ->isComplexioError == true)
    {
        gCSIRXState[csiRxInstance].commonIRQcount.isComplexIOerror++; 
    }
    if(IRQ->isFifoOverflow == true)
    {
        gCSIRXState[csiRxInstance].commonIRQcount.isFIFOoverflow++;
    }

    if(IRQ->isComplexioError)
    {
        gCSIRXErrorCode[csiRxInstance] = CSIRX_complexioGetPendingIntr(handle, &gCSIRXState[csiRxInstance].IRQ.complexIOlanes);
        if(gCSIRXErrorCode[csiRxInstance] != SystemP_SUCCESS)
        {
            test_print("Error occured while recieving the frame-%d\n", frameCounter);
        }
        DebugP_assert(gCSIRXErrorCode[csiRxInstance] == SystemP_SUCCESS);

        gCSIRXErrorCode[csiRxInstance] = CSIRX_complexioClearAllIntr(handle);
        DebugP_assert(gCSIRXErrorCode[csiRxInstance] == SystemP_SUCCESS);
    }

    for(i = 0; i < CONFIG_CSIRX_NUM_INSTANCES; i++)
    {
        if(IRQ->isContextIntr[i] == true)
        {
            gCSIRXErrorCode[csiRxInstance] = CSIRX_contextGetPendingIntr(handle, i, &gCSIRXState[csiRxInstance].IRQ.context[i]);
            DebugP_assert(gCSIRXErrorCode[csiRxInstance] == SystemP_SUCCESS);

            if(gCSIRXState[csiRxInstance].IRQ.context[i].isFrameEndCodeDetect == true)
            {
                gCSIRXState[csiRxInstance].contextIRQcounts[i].frameEndCodeDetect++;
            }

            gCSIRXErrorCode[csiRxInstance] = CSIRX_contextClearAllIntr(handle, i);
            DebugP_assert(gCSIRXErrorCode[csiRxInstance] == SystemP_SUCCESS);
        }
    }
}

/**
 *  @b Description
 *  @n
 *      Callback function for when end of frame detected,
 *      used for debug purposes
 * 
 *  @param[in] handle      CSIRX Handle
 *  @param[in] arg         Callback function argument
 *
 *  @retval
 *      Not Applicable.
 */
void MmwCascade_csirxCombinedEOFcallback(CSIRX_Handle handle, void *arg)
{
    DebugP_assert(handle != NULL);

    gCSIRXState[*((uint32_t*)arg)].callbackCount.combinedEOF++;
}

/**
 *  @b Description
 *  @n
 *      Callback function for when end of frame detected,
 *      used for debug purposes
 * 
 *  @param[in] handle      CSIRX Handle
 *  @param[in] arg         Callback function argument
 *
 *  @retval
 *      Not Applicable.
 */
void MmwCascade_combinedEOLcallback(CSIRX_Handle handle, void *arg)
{
    int32_t errCode  = 0U;
    static Bool firstTimeFlag = true;

    DebugP_assert(handle != NULL);

    if(firstTimeFlag)
    {
        /* Activate CBUFF SW Session to transmit ADC data. */
        /* If SW LVDS stream is enabled, start the session here. User data will immediately
        start to stream over LVDS.*/
        if(CBUFF_activateSession (gMmwCascadeMCB.lvdsStreamcfg.lvdsStream.swSessionHandle, &errCode) < 0)
        {
            /* Error: Unable to activate the CBUFF SW session */
            test_print("Error: CBUFF_activateSession unable to activate CBUFF SW session with [Error=%d]\n", errCode);
            DebugP_assert(0);
        }

        firstTimeFlag = false;
    }
    else
    {
        /* Configure SRC Address of EDMA channel. */
        /* No change in the size of the data to be transferred. */
        configureTransfer();

        /* Trigger CBUFF SW Session. */
        *((volatile uint32_t *)CBUFF_CONFIG_REG_0) |= 1 << 25;
        *((volatile uint32_t *)CBUFF_CONFIG_REG_0) |= 1 << 24;
    }

    gCSIRXState[*((uint32_t*)arg)].callbackCount.combinedEOL++;
}


/**
 *  @b Description
 *  @n
 *      Call back function that was registered during config time and is going
 *      to be called whenever start of frame interrupt is registered.
 *
 *  @param[in] handle CSIRX Handle
 *  @param[in] contextId Context ID
 * 
 *  @retval None
 */
void mmwCascade_csirxSOF0callback(CSIRX_Handle handle, uint32_t arg, uint8_t contextId)
{
    DebugP_assert(handle != NULL);
    /* DebugP_assert(contextId == MMW_CASCADE_CSI2_CONTEXT); */
    gCSIRXState[arg].callbackCount.SOF0++;
}


/**
 *  @b Description
 *  @n
 *      Initializes  CSIRX.
 *      
 *  @param[in] obj      Pointer to data path object
 *
 *  @retval
 *      Not Applicable.
 */
void MmwCascade_csirxInit(MmwCascade_MCB  *CascadeMCB)
{
    int32_t errorCode;
    uint32_t u32DevIdx;

    /* Initialize global variables */
    memset(gCSIRXState, 0, MMWAVE_RADAR_DEVICES * sizeof(MmwCascade_CSIRX_State));

    for(u32DevIdx = 0U; u32DevIdx < MMWAVE_RADAR_DEVICES; u32DevIdx++)
    {
        CascadeMCB->csiRxHandle[u32DevIdx] = NULL;

        gCSIRXState[u32DevIdx].isReceivedPayloadCorrect = true;

        gCSIRXErrorCode[u32DevIdx] = 0;       /* Only for debug */
    }

    errorCode = CSIRX_init();
    if (errorCode != SystemP_SUCCESS)
    {
        test_print("Error: CSIRX initialization returned error %d\n", errorCode);
        DebugP_assert (0);
        return;
    }
}


/**
 *  @b Description
 *  @n
 *      Open CSI RX instance.
 * 
 *  @param[in] instanceId      CSIRX Instance ID
 *  @param[out] errCode        Error Code
 *
 *  @retval
 *      Not Applicable.
 */
void MmwCascade_csirxOpen(MmwCascade_MCB  *CascadeMCB, int32_t *errCode)
{
    uint32_t u32DevIdx;

     for(u32DevIdx = 0U; u32DevIdx < MMWAVE_RADAR_DEVICES; u32DevIdx++)
    {
        if(u32DevIdx == 0)
        {
            CascadeMCB->csiRxHandle[u32DevIdx] = CSIRX_open(CONFIG_CSIRX0);
        }
        else
        {
            CascadeMCB->csiRxHandle[u32DevIdx] = CSIRX_open(CONFIG_CSIRX1);
        }

        if(CascadeMCB->csiRxHandle[u32DevIdx] == NULL)
        {
            *errCode = -1;
        }
        else
        {
            /* Reset CSI-2 */
            *errCode = CSIRX_reset(CascadeMCB->csiRxHandle[u32DevIdx]);
        }
    }
    return;
}

/**
 *  @b Description
 *  @n
 *      Performs CSI Configuration
 * 
 *  @param[in] obj      Pointer to data path object
 * gCSIRXCfg
 *  @retval
 *      Not Applicable.
 */
void MmwCascade_CSIConfig(MmwCascade_MCB  *CascadeMCB)
{
    int32_t errorCode;
    uint32_t u32DevIdx;
    volatile bool isComplexIOresetDone;

    CSIRX_ContextConfig *ptrCsirxContextConfig = NULL;

    for(u32DevIdx = 0U; u32DevIdx < MMWAVE_RADAR_DEVICES; u32DevIdx++)
    {
        if(u32DevIdx == 0 )
        {
            gConfigCsirx0ContextConfig[MMW_CASCADE_CSI2_CONTEXT].pingPongConfig.pingAddress        = (uint32_t) AddrTranslateP_getLocalAddr((uint32_t) &CSIA_PingBuf);
            gConfigCsirx0ContextConfig[MMW_CASCADE_CSI2_CONTEXT].pingPongConfig.pongAddress        = (uint32_t) AddrTranslateP_getLocalAddr((uint32_t) &CSIA_PongBuf);
            ptrCsirxContextConfig = &gConfigCsirx0ContextConfig[MMW_CASCADE_CSI2_CONTEXT];
            gCsirxCommonConfig[CONFIG_CSIRX0].intrCallbacks.commonCallbackArgs = (void*)&csirx0CommonCallBackArg;
            gCsirxCommonConfig[CONFIG_CSIRX0].intrCallbacks.combinedEndOfFrameCallbackArgs = (void*)&csirx0CommonCallBackArg;
            gCsirxCommonConfig[CONFIG_CSIRX0].intrCallbacks.combinedEndOfLineCallbackArgs = (void*)&csirx0CommonCallBackArg;
        }
        else
        {
            gConfigCsirx1ContextConfig[MMW_CASCADE_CSI2_CONTEXT].pingPongConfig.pingAddress        = (uint32_t) AddrTranslateP_getLocalAddr((uint32_t) &CSIB_PingBuf);
            gConfigCsirx1ContextConfig[MMW_CASCADE_CSI2_CONTEXT].pingPongConfig.pongAddress        = (uint32_t) AddrTranslateP_getLocalAddr((uint32_t) &CSIB_PongBuf);
            ptrCsirxContextConfig = &gConfigCsirx1ContextConfig[MMW_CASCADE_CSI2_CONTEXT];
            gCsirxCommonConfig[CONFIG_CSIRX1].intrCallbacks.commonCallbackArgs = (void*)&csirx1CommonCallBackArg;
            gCsirxCommonConfig[CONFIG_CSIRX1].intrCallbacks.combinedEndOfFrameCallbackArgs = (void*)&csirx1CommonCallBackArg;
            gCsirxCommonConfig[CONFIG_CSIRX1].intrCallbacks.combinedEndOfLineCallbackArgs = (void*)&csirx1CommonCallBackArg;
        }

        /* config complex IO - lanes and IRQ */
        errorCode = CSIRX_complexioSetConfig(CascadeMCB->csiRxHandle[u32DevIdx], &gCsirxComplexioConfig[u32DevIdx]);
        if(errorCode != SystemP_SUCCESS)
        {
            test_print("CSIRX_configComplexIO failed, errorCode = %d\n", errorCode);
            DebugP_assert (0);
            return;
        }

        /* deassert complex IO reset */
        errorCode = CSIRX_complexioDeassertReset(CascadeMCB->csiRxHandle[u32DevIdx]);
        if(errorCode != SystemP_SUCCESS)
        {
            /* test_print("CSIRX_deassertComplexIOreset failed, errorCode = %d\n", errorCode); */
            DebugP_assert (0);
            return;
        }

        /* config DPHY */
        errorCode = CSIRX_dphySetConfig(CascadeMCB->csiRxHandle[u32DevIdx], &gCsirxDphyConfig[u32DevIdx]);
        if(errorCode != SystemP_SUCCESS)
        {
            /* test_print("CSIRX_configDPHY failed, errorCode = %d\n", errorCode); */
            DebugP_assert (0);
            return;
        }
         errorCode = CSIRX_complexioSetPowerCommand(CascadeMCB->csiRxHandle[u32DevIdx], 1);
        if(errorCode != SystemP_SUCCESS)
        {
            /* test_print("CSIRX_setComplexIOpowerCommand failed, errorCode = %d\n", errorCode); */
            DebugP_assert (0);
            return;
        }

        uint8_t isComplexIOpowerStatus = 1;
        do
        {
            errorCode = CSIRX_complexioGetPowerStatus(CascadeMCB->csiRxHandle[u32DevIdx], &isComplexIOpowerStatus);
            if(errorCode != SystemP_SUCCESS)
            {
                /* test_print("CSIRX_getComplexIOpowerStatus failed, errorCode = %d\n", errorCode); */
                DebugP_assert (0);
                return;
            }
        }while(isComplexIOpowerStatus == 0);

        /* config common */
        errorCode = CSIRX_commonSetConfig(CascadeMCB->csiRxHandle[u32DevIdx], &gCsirxCommonConfig[u32DevIdx]);
        if(errorCode != SystemP_SUCCESS)
        {
            /* test_print("CSIRX_configCommon failed, errorCode = %d\n", errorCode); */
            DebugP_assert (0);
            return;
        }
        errorCode = CSIRX_contextSetConfig(CascadeMCB->csiRxHandle[u32DevIdx], MMW_CASCADE_CSI2_CONTEXT, &ptrCsirxContextConfig[MMW_CASCADE_CSI2_CONTEXT]);
        if(errorCode != SystemP_SUCCESS)
        {
            /* test_print("CSIRX_configContext failed, errorCode = %d\n", errorCode); */
            DebugP_assert (0);
            return;
        }

        /* enable context */
        errorCode = CSIRX_contextEnable(CascadeMCB->csiRxHandle[u32DevIdx], MMW_CASCADE_CSI2_CONTEXT);
        if(errorCode != SystemP_SUCCESS)
        {
            /* test_print("CSIRX_enableContext failed, errorCode = %d\n", errorCode); */
            DebugP_assert (0);
            return;
        }

        /* enable interface */
        errorCode = CSIRX_commonEnable(CascadeMCB->csiRxHandle[u32DevIdx]);
        if(errorCode != SystemP_SUCCESS)
        {
            /* test_print("CSIRX_enableInterface failed, errorCode = %d\n", errorCode); */
            DebugP_assert (0);
            return;
        }
    }

    /* MSS initialization can continue */
    SemaphoreP_post(&gMmwCascadeMCB.CSI2RXConfigCompleteSemHandle);

    /* Reset will be really effective when both front-end will start to drive CSI-2 lines */
    for(u32DevIdx = 0U; u32DevIdx < MMWAVE_RADAR_DEVICES; u32DevIdx++)
    {
        /* Wait until complex IO reset complete */
        do
        {
            errorCode= CSIRX_complexioIsResetDone(CascadeMCB->csiRxHandle[u32DevIdx], (bool *)&isComplexIOresetDone);
            if(errorCode != SystemP_SUCCESS)
            {
                /* test_print("CSIRX_isComplexIOresetDone failed, errorCode = %d\n", errorCode); */
                DebugP_assert (0);
                return;
            }
            if (isComplexIOresetDone == false)
            {
                ClockP_usleep(1 * 1000);
            }
        }while(isComplexIOresetDone == false);
    }
    if(isComplexIOresetDone == false)
    {
        /* test_print("CSIRX_isComplexIOresetDone attempts exceeded\n"); */
        DebugP_assert (0);
        return;
    }
}

/**
 *  @b Description
 *  @n
 *      Close CSI RX instance.
 *
 *  @retval
 *      Not Applicable.
 */
void MmwCascade_csirxClose(MmwCascade_MCB  *CascadeMCB)
{
    int32_t             errorCode;
    uint32_t u32DevIdx;
    
    for(u32DevIdx = 0U; u32DevIdx < MMWAVE_RADAR_DEVICES; u32DevIdx++)
    {
        /* disable context */
        errorCode = CSIRX_contextDisable(CascadeMCB->csiRxHandle[u32DevIdx], 0);
        if(errorCode != SystemP_SUCCESS)
        {
            /* test_print("Error: CSIRX_disableContext failed, errorCode = %d\n", errorCode); */
            DebugP_assert (0);
            return;
        }

        /* disable interface */
        errorCode = CSIRX_commonDisable(CascadeMCB->csiRxHandle[u32DevIdx]);
        if(errorCode != SystemP_SUCCESS)
        {
            /* test_print("CSIRX_disableInterface failed, errorCode = %d\n", errorCode); */
            DebugP_assert (0);
            return;
        }

        /* close instance */
        errorCode = CSIRX_close(CascadeMCB->csiRxHandle[u32DevIdx]);
        if(errorCode != SystemP_SUCCESS)
        {
            /* test_print("CSIRX_close failed, errorCode = %d\n", errorCode); */
            DebugP_assert (0);
            return;
        }
    }
}
