/*
 *  Copyright (C) 2021 Texas Instruments Incorporated
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
/*
 * Auto generated file
 */

#include "ti_drivers_config.h"

/*
 * CBUFF
 */
/* CBUFF atrributes */
CBUFF_Attrs gCbuffAttrs[CONFIG_CBUFF_NUM_INSTANCES] =
{
    {
        .baseAddr                   = CSL_DSS_CBUFF_U_BASE,
        .fifoBaseAddr               = CSL_DSS_CBUFF_FIFO_U_BASE,
        .maxLVDSLanesSupported      = 4,
        .errorIntrNum               = 144,
        .intrNum                    = 143,
        .cbuffChannelId[0]             = 18,
        .cbuffChannelId[1]             = 19,
        .cbuffChannelId[2]             = 20,
        .cbuffChannelId[3]             = 21,
        .cbuffChannelId[4]             = 22,
        .cbuffChannelId[5]             = 23,
        .cbuffChannelId[6]             = 24,
    },
};

/* CBUFF objects - initialized by the driver */
CBUFF_Object gCbuffObject[CONFIG_CBUFF_NUM_INSTANCES];
/* CBUFF objects - storage for CBUFF driver object handles */
CBUFF_Object *gCbuffObjectPtr[CONFIG_CBUFF_NUM_INSTANCES] = { NULL };
/* CBUFF objects count */
uint32_t gCbuffConfigNum = CONFIG_CBUFF_NUM_INSTANCES;

/*
 * CSIRX
 */
/* CSIRX atrributes */
extern  CSIRX_HwAttrs  gCsirxHwAttrs_csirxA_r5f;
extern  CSIRX_HwAttrs  gCsirxHwAttrs_csirxB_r5f;

/* CSIRX objects - initialized by the driver */
static CSIRX_Object gCsirxObjects[CONFIG_CSIRX_NUM_INSTANCES];
/* CSIRX driver configuration */
CSIRX_Config gCsirxConfig[CONFIG_CSIRX_NUM_INSTANCES] =
{
    {
        .hwAttrs = &gCsirxHwAttrs_csirxA_r5f,
        .object  = &gCsirxObjects[CONFIG_CSIRX0],
    },
    {
        .hwAttrs = &gCsirxHwAttrs_csirxB_r5f,
        .object  = &gCsirxObjects[CONFIG_CSIRX1],
    },
};

uint32_t gCsirxConfigNum = CONFIG_CSIRX_NUM_INSTANCES;

/*
 * EDMA
 */
/* EDMA atrributes */
static EDMA_Attrs gEdmaAttrs[CONFIG_EDMA_NUM_INSTANCES] =
{
    {

        .baseAddr           = CSL_RCSS_TPCC_A_U_BASE,
        .compIntrNumber     = CSL_MSS_INTR_RCSS_TPCC_A_INTAGG,
        .intrAggEnableAddr  = CSL_RCSS_CTRL_U_BASE + CSL_RCSS_CTRL_RCSS_TPCC_A_INTAGG_MASK,
        .intrAggEnableMask  = 0x1FF & (~(2U << 2)),
        .intrAggStatusAddr  = CSL_RCSS_CTRL_U_BASE + CSL_RCSS_CTRL_RCSS_TPCC_A_INTAGG_STATUS,
        .intrAggClearMask   = (2U << 2),
        .initPrms           =
        {
            .regionId     = 2,
            .queNum       = 0,
            .initParamSet = FALSE,
            .ownResource    =
            {
                .qdmaCh      = 0x30U,
                .dmaCh[0]    = 0xFFFFFFFFU,
                .dmaCh[1]    = 0x00000001U,
                .tcc[0]      = 0xFFFFFFFFU,
                .tcc[1]      = 0x00000001U,
                .paramSet[0] = 0x00000000U,
                .paramSet[1] = 0x00000000U,
                .paramSet[2] = 0xFFFFFFFFU,
                .paramSet[3] = 0x00000000U,
            },
            .reservedDmaCh[0]    = 0x00000FFFU,
            .reservedDmaCh[1]    = 0x00000000U,
        },
    },
    {

        .baseAddr           = CSL_MSS_TPCC_A_U_BASE,
        .compIntrNumber     = CSL_MSS_INTR_MSS_TPCC_A_INTAGG,
        .intrAggEnableAddr  = CSL_MSS_CTRL_U_BASE + CSL_MSS_CTRL_MSS_TPCC_A_INTAGG_MASK,
        .intrAggEnableMask  = 0x1FF & (~(2U << 2)),
        .intrAggStatusAddr  = CSL_MSS_CTRL_U_BASE + CSL_MSS_CTRL_MSS_TPCC_A_INTAGG_STATUS,
        .intrAggClearMask   = (2U << 2),
        .initPrms           =
        {
            .regionId     = 2,
            .queNum       = 0,
            .initParamSet = FALSE,
            .ownResource    =
            {
                .qdmaCh      = 0x3FU,
                .dmaCh[0]    = 0xFFFFFFFFU,
                .dmaCh[1]    = 0x00FFFFFFU,
                .tcc[0]      = 0xFFFFFFFFU,
                .tcc[1]      = 0x00FFFFFFU,
                .paramSet[0] = 0xFFFFFFFFU,
                .paramSet[1] = 0xFFFFFFFFU,
                .paramSet[2] = 0xFFFFFFFFU,
                .paramSet[3] = 0x00FFFFFFU,
            },
            .reservedDmaCh[0]    = 0x00000001U,
            .reservedDmaCh[1]    = 0x00000000U,
        },
    },
    {

        .baseAddr           = CSL_DSS_TPCC_A_U_BASE,
        .compIntrNumber     = CSL_MSS_INTR_DSS_TPCC_A_INTAGG,
        .intrAggEnableAddr  = CSL_DSS_CTRL_U_BASE + CSL_DSS_CTRL_DSS_TPCC_A_INTAGG_MASK,
        .intrAggEnableMask  = 0x1FF & (~(2U << 2)),
        .intrAggStatusAddr  = CSL_DSS_CTRL_U_BASE + CSL_DSS_CTRL_DSS_TPCC_A_INTAGG_STATUS,
        .intrAggClearMask   = (2U << 2),
        .initPrms           =
        {
            .regionId     = 2,
            .queNum       = 0,
            .initParamSet = FALSE,
            .ownResource    =
            {
                .qdmaCh      = 0x01U,
                .dmaCh[0]    = 0xFFFFFFFFU,
                .dmaCh[1]    = 0xFFFFFFFFU,
                .tcc[0]      = 0xFFFFFFFFU,
                .tcc[1]      = 0xFFFFFFFFU,
                .paramSet[0] = 0xFFFFFFFFU,
                .paramSet[1] = 0xFFFFFFFFU,
                .paramSet[2] = 0xFFFFFFFFU,
                .paramSet[3] = 0xFFFFFFFFU,
            },
            .reservedDmaCh[0]    = 0x00000001U,
            .reservedDmaCh[1]    = 0x00000000U,
        },
    },
};

/* EDMA objects - initialized by the driver */
static EDMA_Object gEdmaObjects[CONFIG_EDMA_NUM_INSTANCES];
/* EDMA driver configuration */
EDMA_Config gEdmaConfig[CONFIG_EDMA_NUM_INSTANCES] =
{
    {
        &gEdmaAttrs[CONFIG_EDMA0],
        &gEdmaObjects[CONFIG_EDMA0],
    },
    {
        &gEdmaAttrs[CONFIG_EDMA1],
        &gEdmaObjects[CONFIG_EDMA1],
    },
    {
        &gEdmaAttrs[CONFIG_EDMA2],
        &gEdmaObjects[CONFIG_EDMA2],
    },
};

uint32_t gEdmaConfigNum = CONFIG_EDMA_NUM_INSTANCES;

/*
 * MIBSPI
 */
/* MIBSPI atrributes */
static MIBSPI_Attrs gMibspiAttrs[CONFIG_MIBSPI_NUM_INSTANCES] =
{
    {
        .mibspiInstId           = MIBSPI_INST_ID_RCSS_SPIA,
        .ptrSpiRegBase          = (CSL_mss_spiRegs *)CSL_RCSS_SPIA_U_BASE,
        .ptrMibSpiRam           = (CSL_mibspiRam   *)CSL_RCSS_SPIA_RAM_U_BASE,
        .clockSrcFreq           = 200000000U,
        .interrupt0Num          = 147U,
        .interrupt1Num          = 148U,
        .mibspiRamSize          = CSL_MIBSPIRAM_MAX_ELEMENTS,
        .numTransferGroups      = 1U,
        .numParallelModePins    = MIBSPI_FEATURE_PARALLEL_MODE_DIS,
        .featureBitMap          = MIBSPI_FEATURE_SPIENA_PIN_DIS,
        .numDmaReqLines         = 1U,
        .dmaReqlineCfg =
        {
            [0] =
            {
                EDMA_RCSS_TPCC_A_EVT_SPIA_DMA_REQ0,
                EDMA_RCSS_TPCC_A_EVT_SPIA_DMA_REQ1
            },
        }
    },
    {
        .mibspiInstId           = MIBSPI_INST_ID_RCSS_SPIB,
        .ptrSpiRegBase          = (CSL_mss_spiRegs *)CSL_RCSS_SPIB_U_BASE,
        .ptrMibSpiRam           = (CSL_mibspiRam   *)CSL_RCSS_SPIB_RAM_U_BASE,
        .clockSrcFreq           = 200000000U,
        .interrupt0Num          = 149U,
        .interrupt1Num          = 150U,
        .mibspiRamSize          = CSL_MIBSPIRAM_MAX_ELEMENTS,
        .numTransferGroups      = 1U,
        .numParallelModePins    = MIBSPI_FEATURE_PARALLEL_MODE_DIS,
        .featureBitMap          = MIBSPI_FEATURE_SPIENA_PIN_DIS,
        .numDmaReqLines         = 1U,
        .dmaReqlineCfg =
        {
            [0] =
            {
                EDMA_RCSS_TPCC_A_EVT_SPIB_DMA_REQ0,
                EDMA_RCSS_TPCC_A_EVT_SPIB_DMA_REQ1
            },
        }
    },
    {
        .mibspiInstId           = MIBSPI_INST_ID_MSS_SPIB,
        .ptrSpiRegBase          = (CSL_mss_spiRegs *)CSL_MSS_SPIB_U_BASE,
        .ptrMibSpiRam           = (CSL_mibspiRam   *)CSL_MSS_SPIB_RAM_U_BASE,
        .clockSrcFreq           = 200000000U,
        .interrupt0Num          = 33U,
        .interrupt1Num          = 34U,
        .mibspiRamSize          = CSL_MIBSPIRAM_MAX_ELEMENTS,
        .numTransferGroups      = 1U,
        .numParallelModePins    = MIBSPI_FEATURE_PARALLEL_MODE_DIS,
        .featureBitMap          = MIBSPI_FEATURE_SPIENA_PIN_DIS,
        .numDmaReqLines         = 1U,
        .dmaReqlineCfg =
        {
            [0] =
            {
                EDMA_MSS_TPCC_A_EVT_SPIB_DMA_REQ0,
                EDMA_MSS_TPCC_A_EVT_SPIB_DMA_REQ1
            },
        }
    },
};
/* MIBSPI objects - initialized by the driver */
static MIBSPI_Object gMibspiObjects[CONFIG_MIBSPI_NUM_INSTANCES];
/* MIBSPI driver configuration */
MIBSPI_Config gMibspiConfig[CONFIG_MIBSPI_NUM_INSTANCES] =
{
    {
        &gMibspiAttrs[CONFIG_MIBSPI0],
        &gMibspiObjects[CONFIG_MIBSPI0],
    },
    {
        &gMibspiAttrs[CONFIG_MIBSPI1],
        &gMibspiObjects[CONFIG_MIBSPI1],
    },
    {
        &gMibspiAttrs[CONFIG_MIBSPI2],
        &gMibspiObjects[CONFIG_MIBSPI2],
    },
};

uint32_t gMibspiConfigNum = CONFIG_MIBSPI_NUM_INSTANCES;


void Pinmux_init(void);
void PowerClock_init(void);
void PowerClock_deinit(void);

/*
 * Common Functions
 */
void System_init(void)
{
    /* DPL init sets up address transalation unit, on some CPUs this is needed
     * to access SCICLIENT services, hence this needs to happen first
     */
    Dpl_init();

    
    /* initialize PMU */
    CycleCounterP_init(SOC_getSelfCpuClk());


    PowerClock_init();
    /* Now we can do pinmux */
    Pinmux_init();
    /* finally we initialize all peripheral drivers */
    CSIRX_init();
    EDMA_init();
    MIBSPI_init();
}

void System_deinit(void)
{
    CSIRX_deinit();
    EDMA_deinit();
    MIBSPI_deinit();
    PowerClock_deinit();
    Dpl_deinit();
}
