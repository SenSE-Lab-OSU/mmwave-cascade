/*
 *   @file  cascade_csirx.h
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

#ifndef MMW_CASCADE_CSIRX_H
#define MMW_CASCADE_CSIRX_H

#include <drivers/csirx.h>
#include <kernel/dpl/SemaphoreP.h>
#include <ti/control/mmwave/mmwave.h>
#include <ti/utils/test/cascade/am273x/mssgenerated/ti_drivers_config.h>
#include <ti/utils/test/cascade/am273x/cascade_lvds_stream.h>

/**************************************************************************
 *************************** Macros and Typedefs **************************
 **************************************************************************/
/* mmWave SK Include Files: */
/* CSIRX Data and Clock Lane positions and polarity */
#define MMW_CASCADE_CSI2_CONTEXT                    (0U)

#define TEST_NUM_ADC_SAMPLES                (256U)
#define TEST_NUM_LOOPS                      (64U)
#define TEST_NUM_FRAMES                     (100U)
#define TEST_NUM_RX                         (4U) /* This many LSBs will be set in rxChannelEn in chCfg */
#define TEST_NUM_TX                         (3U)
#define TEST_BYTES_PER_ADC_SAMPLE           (4U) /* complex */
#define NUM_OF_PHASE_SHIFT_CHIRPS           (1) /* Number of chirps to configure Phase Shift. */
#define TEST_NUM_CHIRPS                     (1) /* Number of chirps for each Profile. */
#define TEST_CHIRP_END_INDEX                (TEST_NUM_CHIRPS - 1U)

#define BOARD_DIAG_TEST_NUM_BYTES_PER_FRAME \
    (TEST_NUM_ADC_SAMPLES * TEST_BYTES_PER_ADC_SAMPLE * TEST_NUM_RX)

#define BOARD_DIAG_PING_PONG_ALIGNMENT CSL_MAX(CSIRX_PING_PONG_ADDRESS_LINEOFFSET_ALIGNMENT_IN_BYTES, \
                                         CSL_CACHE_L1D_LINESIZE)

#define BOARD_DIAG_PING_OR_PONG_BUF_SIZE_ALIGNED CSL_NEXT_MULTIPLE_OF(BOARD_DIAG_TEST_NUM_BYTES_PER_FRAME, \
                                                                BOARD_DIAG_PING_PONG_ALIGNMENT)


/* CSIRX related structures */

/*! holds context IRQ counts */
typedef struct MmwCascade_CSIRX_ContextIRQcount_s
{
    volatile uint32_t frameEndCodeDetect;
    uint32_t lineEndCodeDetect;
} MmwCascade_CSIRX_ContextIRQcount;

/*! holds common IRQ counts */
typedef struct MmwCascade_CSIRX_CommonIRQcount_s
{
    uint32_t isOCPerror;
    uint32_t isGenericShortPacketReceive;
    uint32_t isECConeBitShortPacketErrorCorrect;
    uint32_t isECCmoreThanOneBitCannotCorrect;
    uint32_t isComplexIOerror;
    uint32_t isFIFOoverflow;
} MmwCascade_CSIRX_CommonIRQcount;

typedef struct MmwCascade_CSIRX_IRQs_s
{
    CSIRX_ContextIntr context[CONFIG_CSIRX_NUM_INSTANCES];
    CSIRX_CommonIntr common;
    CSIRX_ComplexioLanesIntr  complexIOlanes;
} MmwCascade_CSIRX_IRQs;

/* Holds callback counts for different events */
typedef struct MmwCascade_CSIRX_CallBackCounts_s
{
    uint32_t common;
    uint32_t combinedEOL;
    uint32_t combinedEOF;
    uint32_t EOF0;
    uint32_t EOF1;
    uint32_t SOF0;
    uint32_t SOF1;
    uint32_t contextEOL[CONFIG_CSIRX_NUM_INSTANCES];
} MmwCascade_CSIRX_CallBackCounts;

/* Holds CSIRX state information */
typedef struct MmwCascade_CSIRX_State_s
{
	MmwCascade_CSIRX_CommonIRQcount commonIRQcount;
    MmwCascade_CSIRX_IRQs IRQ;
    MmwCascade_CSIRX_CallBackCounts callbackCount;
    bool isReceivedPayloadCorrect;
    MmwCascade_CSIRX_ContextIRQcount contextIRQcounts[CONFIG_CSIRX_NUM_INSTANCES];
    uint32_t frameId;
    uint32_t lineId;
    uint32_t receivedBuffer;
} MmwCascade_CSIRX_State;

/**
 * @brief
 *  Millimeter Wave Demo Data Path Object
 *
 * @details
 *  The structure is used to hold all the relevant information for the
 *  Millimeter Wave demo's data path
 */
typedef struct MmwCascade_MCB_t
{
    /*! @brief Handle to mmWave layer. */
    MMWave_Handle        mmWaveHandle;

    /*! @brief   Handle to CSI RX  interface */
    CSIRX_Handle         csiRxHandle[MMWAVE_RADAR_DEVICES];

    /*! @brief   Semaphore handle to signal  CSI2RX config complete */
    SemaphoreP_Object     CSI2RXConfigCompleteSemHandle;

    /*! @brief LVDS streaming Configuration*/
    MmwCascade_LVDSStream_MCB lvdsStreamcfg;

    /*! @brief   Handle to PMIC SPI interface */
    MIBSPI_Handle          pmicMIBSPIhandle;
} MmwCascade_MCB;

/**************************************************************************
 *************************** Extern Definitions ***************************
 **************************************************************************/
extern void mmwCascade_csirxSOF0callback(CSIRX_Handle handle, uint32_t arg, uint8_t contextId);

extern void MmwCascade_csirxClose(MmwCascade_MCB  *CascadeMCB);
extern void MmwCascade_CSIConfig(MmwCascade_MCB  *CascadeMCB);

#endif /* MMW_CASCADE_CSIRX_H */
