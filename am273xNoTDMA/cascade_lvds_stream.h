/**
 *   @file  cascade_lvds_stream.h
 *
 *   @brief
 *      LVDS stream header file.
 *
 *  \par
 *  NOTE:
 *      (C) Copyright 2020 - 2021 Texas Instruments, Inc.
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
#ifndef MSS_LVDS_STREAM_H
#define MSS_LVDS_STREAM_H

#ifdef __cplusplus
extern "C" {
#endif

/* MCU+SDK Include Files. */
#include <kernel/dpl/SemaphoreP.h>
#include <drivers/cbuff.h>
/* ---- LVDS per-chirp sync header ---- */
#define LVDS_HDR_MAGIC   (0xA1B2C3D4U)

typedef struct LvdsFrameHeader_t
{
    uint32_t magic;        /* 0xA1B2C3D4 — host locks onto chirp boundaries */
    uint32_t globalCount;  /* monotonic chirp counter, never resets         */
    uint32_t frameId;      /* radar frame index                             */
    uint32_t chirpId;      /* chirp index within frame (0..TEST_NUM_LOOPS-1)*/
} LvdsFrameHeader;         /* 16 bytes */

/**
 * @brief   This is the maximum number of EDMA Channels which is used by
 * the SW Session
 */
#define CASCADE_LVDS_STREAM_SW_SESSION_MAX_EDMA_CHANNEL             3U

/*************************LVDS streaming EDMA resources*******************************/
#define CASCADE_LVDS_STREAM_EDMA_SHADOW_BASE         (64U)

/* CBUFF EDMA trigger channels */
#define CASCADE_LVDS_STREAM_CBUFF_EDMA_CH_0          (EDMA_DSS_TPCC_A_EVT_CBUFF_DMA_REQ0)
#define CASCADE_LVDS_STREAM_CBUFF_EDMA_CH_1          (EDMA_DSS_TPCC_A_EVT_CBUFF_DMA_REQ1)

/* SW Session*/
#define CASCADE_LVDS_STREAM_SW_SESSION_EDMA_CH_0     (EDMA_DSS_TPCC_A_EVT_FREE_0)
#define CASCADE_LVDS_STREAM_SW_SESSION_EDMA_CH_1     (EDMA_DSS_TPCC_A_EVT_FREE_1)
#define CASCADE_LVDS_STREAM_SW_SESSION_EDMA_CH_2     (EDMA_DSS_TPCC_A_EVT_FREE_2)

/*shadow*/
/*shadow CBUFF trigger channels*/
#define CASCADE_LVDS_STREAM_CBUFF_EDMA_SHADOW_CH_0   (CASCADE_LVDS_STREAM_EDMA_SHADOW_BASE + 0U)
#define CASCADE_LVDS_STREAM_CBUFF_EDMA_SHADOW_CH_1   (CASCADE_LVDS_STREAM_EDMA_SHADOW_BASE + 1U)

/* SW Session*/
#define CASCADE_LVDS_STREAM_SW_SESSION_EDMA_SHADOW_CH_0   (CASCADE_LVDS_STREAM_EDMA_SHADOW_BASE + 2U)
#define CASCADE_LVDS_STREAM_SW_SESSION_EDMA_SHADOW_CH_1   (CASCADE_LVDS_STREAM_EDMA_SHADOW_BASE + 3U)
#define CASCADE_LVDS_STREAM_SW_SESSION_EDMA_SHADOW_CH_2   (CASCADE_LVDS_STREAM_EDMA_SHADOW_BASE + 4U)
/*************************LVDS streaming EDMA resources END*******************************/

typedef struct Cascade_LVDSStreamCfg_t
{
    /**
    * @brief   Handle to the CBUFF Driver
    */
    CBUFF_Handle             cbuffHandle;

    /**
     * @brief   EDMA Channel Allocator Index for the SW Session
     */
    uint32_t                 swSessionEDMAChannelAllocatorIndex;

    /**
     * @brief   EDMA Channel Resource Table: This is used for creating the CBUFF Session.
     */
    CBUFF_EDMAChannelCfg     swSessionEDMAChannelTable[CASCADE_LVDS_STREAM_SW_SESSION_MAX_EDMA_CHANNEL];

    /**
     * @brief   Handle to the SW CBUFF Session Handle.
     */
    CBUFF_SessionHandle      swSessionHandle;

    /**
     * @brief   Number of SW frame done interrupt received.
     */
    volatile uint32_t        swFrameDoneCount;

} Cascade_LVDSStreamCfg;

typedef struct MmwCascade_LVDSStream_MCB_t
{
    /*! @brief   Handle of the EDMA driver, used for CBUFF */
    EDMA_Handle                  edmaHandle;

    /*! @brief   Number of EDMA event Queues (tc) */
    uint8_t                     numEdmaEventQueues;

    /*! @brief   this structure is used to hold all the relevant information
        for the mmw demo LVDS stream*/
    Cascade_LVDSStreamCfg       lvdsStream;

}MmwCascade_LVDSStream_MCB;

/**************************************************************************
 *************************** Extern Definitions ***************************
 **************************************************************************/
extern int32_t Cascade_LVDSStreamInit (void);
extern int32_t Cascade_LVDSStreamSwConfig (uint32_t userBuffer0Addr,
                                           uint32_t userBuffer1Addr,
                                           uint32_t numOfSamples);
extern void MmwDemo_LVDSStreamDeleteSwSession (void);

#ifdef __cplusplus
}
#endif

#endif /* MSS_LVDS_STREAM_H */
