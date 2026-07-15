/*
 *   @file  common_full.c
 *
 *   @brief
 *      The file contains functions which are reused among the DSS and
 *      MSS code base
 *
 *  \par
 *  NOTE:
 *      (C) Copyright 2016-2020 Texas Instruments, Inc.
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
#define DebugP_LOG_ENABLED 1 

/* Standard Include Files. */
#include <stdint.h>
#include <stdlib.h>
#include <stddef.h>
#include <string.h>
#include <stdio.h>

#include <kernel/dpl/CacheP.h>

/* mmWave SK Include Files: */
#include <ti/common/syscommon.h>
#include <ti/control/mmwavelink/mmwavelink.h>
#include <ti/control/mmwave/mmwave.h>
#include <ti/utils/testlogger/logger.h>
#include <ti/utils/test/cascade/am273x/cascade_csirx.h>

/**************************************************************************
 *************************** Local Definitions ****************************
 **************************************************************************/
#define TEST_MAX_PROFILE                  (2U)

#define TEST_MAX_BPM_CHIRP                (8U)

/* CHIRP tab settings */
/* CHIRP Sub-tab */
#define CHIRP_0_PROFILE_ID                (0U)
#define CHIRP_0_START_INDEX               (0U)
#define CHIRP_0_END_INDEX                 (0U)
#define CHIRP_0_START_FREQ_VAL            (0U)
#define CHIRP_0_FREQ_SLOPE_VAL            (0U)
#define CHIRP_0_IDLE_TIME_VAL             (0U)
#define CHIRP_0_ADC_START_TIME_VAL        (0U)
#define CHIRP_0_TX_CHANNEL                CHIRP_TX_1_CHANNEL_ENABLE

#define CHIRP_1_PROFILE_ID                (1U)
#define CHIRP_1_START_INDEX               (1U)
#define CHIRP_1_END_INDEX                 (1U)
#define CHIRP_1_START_FREQ_VAL            (0U)
#define CHIRP_1_FREQ_SLOPE_VAL            (0U)
#define CHIRP_1_IDLE_TIME_VAL             (0U)
#define CHIRP_1_ADC_START_TIME_VAL        (0U)
#define CHIRP_1_TX_CHANNEL                CHIRP_TX_1_CHANNEL_ENABLE

/* PROFILE Sub-tab */
#define PROFILE_0_PROFILE_ID              (0U)
#define PROFILE_0_HPFCORNER_FREQ1_VAL     RL_RX_HPF1_175_KHz
#define PROFILE_0_HPFCORNER_FREQ2_VAL     RL_RX_HPF2_350_KHz
#define PROFILE_0_RX_GAIN_VAL             (30U)
#define PROFILE_0_DIGOUT_SAMPLERATE_VAL   (10000U)
#define PROFILE_0_ADC_SAMPLE_VAL          (224U)
#define PROFILE_0_IDLE_TIME_VAL           (1500U)
#define PROFILE_0_RAMP_END_TIME_VAL       (3000U)
#define PROFILE_0_START_FREQ_VAL          (1454025386U)
#define PROFILE_0_TXOUT_POWER_BACKOFF     (0U)
#define PROFILE_0_TXPHASESHIFTER_VAL      (0U)
#define PROFILE_0_FREQ_SLOPE_VAL          (1657U)
#define PROFILE_0_TX_START_TIME_VAL       (100U)  /* 1us */
#define PROFILE_0_ADC_START_TIME_VAL      (600U)  /* 6us */

/* PROFILE Sub-tab */
#define PROFILE_1_PROFILE_ID              (1U)
#define PROFILE_1_HPFCORNER_FREQ1_VAL     RL_RX_HPF1_175_KHz
#define PROFILE_1_HPFCORNER_FREQ2_VAL     RL_RX_HPF2_350_KHz
#define PROFILE_1_RX_GAIN_VAL             (30U)
#define PROFILE_1_DIGOUT_SAMPLERATE_VAL   (10000U)
#define PROFILE_1_ADC_SAMPLE_VAL          (512U)
#define PROFILE_1_IDLE_TIME_VAL           (1500U)
#define PROFILE_1_RAMP_END_TIME_VAL       (6000U)
#define PROFILE_1_START_FREQ_VAL          (1454025386U)  /* 78Ghz */
#define PROFILE_1_TXOUT_POWER_BACKOFF     (0U)
#define PROFILE_1_TXPHASESHIFTER_VAL      (0U)
#define PROFILE_1_FREQ_SLOPE_VAL          (828U)
#define PROFILE_1_TX_START_TIME_VAL       (100U)  /* 1us */
#define PROFILE_1_ADC_START_TIME_VAL      (600U)  /* 6us */

/* FRAME Sub-tab */
#define FRAME_CHIRP_START_IDX           (0U)
#define FRAME_CHIRP_END_IDX             (0U)
#define FRAME_COUNT_VAL                 (0)
#define FRAME_LOOP_COUNT                (64)
#define FRAME_PRIODICITY_VAL            (16000000)
#define FRAME_TRIGGER_DELAY_VAL         (0U)
#define FRAME_ADC_SAMPLE_NUM            (512U)


/* CHIRP Tx Channel Settings */
#define CHIRP_TX_1_CHANNEL_ENABLE           (1U << 0U)
#define CHIRP_TX_2_CHANNEL_ENABLE           (1U << 1U)
#define CHIRP_TX_3_CHANNEL_ENABLE           (1U << 2U)
#define CHIRP_TX_1_2_CHANNEL_ENABLE         (CHIRP_TX_1_CHANNEL_ENABLE | CHIRP_TX_2_CHANNEL_ENABLE)
#define CHIRP_TX_1_3_CHANNEL_ENABLE         (CHIRP_TX_2_CHANNEL_ENABLE | CHIRP_TX_3_CHANNEL_ENABLE)
#define CHIRP_TX_2_3_CHANNEL_ENABLE         (CHIRP_TX_2_CHANNEL_ENABLE | CHIRP_TX_3_CHANNEL_ENABLE)
#define CHIRP_TX_1_2_3_CHANNEL_ENABLE       (CHIRP_TX_1_CHANNEL_ENABLE | CHIRP_TX_2_CHANNEL_ENABLE | CHIRP_TX_3_CHANNEL_ENABLE)

/* CHIRP Profile Settings */
#define CHIRP_HPF1_CORNER_FREQ_175K         (0U)
#define CHIRP_HPF1_CORNER_FREQ_235K         (1U)
#define CHIRP_HPF1_CORNER_FREQ_350K         (2U)
#define CHIRP_HPF1_CORNER_FREQ_700K         (3u)

#define CHIRP_HPF2_CORNER_FREQ_350K         (0U)
#define CHIRP_HPF2_CORNER_FREQ_700K         (1U)
#define CHIRP_HPF2_CORNER_FREQ_1_4M         (2U)
#define CHIRP_HPF2_CORNER_FREQ_2_8M         (3U)
#define CHIRP_HPF2_CORNER_FREQ_5M           (4U)
#define CHIRP_HPF2_CORNER_FREQ_7_5M         (5U)
#define CHIRP_HPF2_CORNER_FREQ_10M          (6U)
#define CHIRP_HPF2_CORNER_FREQ_15M          (7U)

rlRfPhaseShiftCfg_t phaseShifChirpCfgDev1[NUM_OF_PHASE_SHIFT_CHIRPS] =
{
    {
        .chirpStartIdx = 0,
        .chirpEndIdx   = 0,
        .tx0PhaseShift = 0,
        .tx1PhaseShift = 0,
        .tx2PhaseShift = 0,
        .reserved      = 0
    }
};

rlRfPhaseShiftCfg_t phaseShifChirpCfgDev2[NUM_OF_PHASE_SHIFT_CHIRPS] =
{
    {
        .chirpStartIdx = 0,
        .chirpEndIdx   = 0,
        .tx0PhaseShift = 0,
        .tx1PhaseShift = 0,
        .tx2PhaseShift = 0,
        .reserved      = 0
    }
};

/**************************************************************************
 ************************* Extern Declarations ****************************
 **************************************************************************/
extern MmwCascade_MCB    gMmwCascadeMCB;

/**************************************************************************
 ************************* Common Test Functions **************************
 **************************************************************************/

/**
 *  @b Description
 *  @n
 *      Utility function which populates the profile configuration with
 *      well defined defaults.
 *
 *  @param[out]  ptrProfileCfg
 *      Pointer to the populated profile configuration
 *
 *  @retval
 *      Not applicable
 */
static void Mmwave_populateDefaultProfileCfg (rlProfileCfg_t* ptrProfileCfg)
{
    /* Initialize the profile configuration: */
    memset ((void*)ptrProfileCfg, 0, sizeof(rlProfileCfg_t));

    /* Populate the *default* profile configuration: */
    ptrProfileCfg->profileId             = 0;
    ptrProfileCfg->startFreqConst        = (uint32_t) ((float)76 * (1U << 26) / 3.6); //76F
    ptrProfileCfg->idleTimeConst         = 7 * 1000 / 10; //7 us
    ptrProfileCfg->adcStartTimeConst     = 1.5 * 1000 / 10; // 1.5 us
    ptrProfileCfg->rampEndTime           = 2800U; //28 us;
    // ptrProfileCfg->rampEndTime           = 5400U; //54 us;

    ptrProfileCfg->txOutPowerBackoffCode = 0;
    ptrProfileCfg->txPhaseShifter        = 0;
    // ptrProfileCfg->freqSlopeConst        = 11 * 1000 / 48;
    ptrProfileCfg->freqSlopeConst        = 3108; //150.06 MHz/us
    // ptrProfileCfg->freqSlopeConst        = 1657; //80 MHz/us
    ptrProfileCfg->txStartTime           = 0 * 1000 / 10;
    ptrProfileCfg->numAdcSamples         = TEST_NUM_ADC_SAMPLES;
    ptrProfileCfg->digOutSampleRate      = 10000;
    ptrProfileCfg->hpfCornerFreq1        = 0;
    ptrProfileCfg->hpfCornerFreq2        = 0;
    ptrProfileCfg->rxGain                = 42;
}

/**
 *  @b Description
 *  @n
 *      Utility function which populates the chirp configuration with
 *      well defined defaults.
 *
 *  @param[out]  ptrChirpCfg
 *      Pointer to the populated chirp configuration
 *
 *  @retval
 *      Not applicable
 */
static void Mmwave_populateDefaultChirpCfg (rlChirpCfg_t* ptrChirpCfg)
{
    /* Initialize the chirp configuration: */
    memset ((void*)ptrChirpCfg, 0, sizeof(rlChirpCfg_t));

    /* Populate the chirp configuration: */
    ptrChirpCfg->chirpStartIdx   = 0;
    ptrChirpCfg->chirpEndIdx     = 0;
    ptrChirpCfg->profileId       = 0;
    ptrChirpCfg->startFreqVar    = 0;
    ptrChirpCfg->freqSlopeVar    = 0;
    ptrChirpCfg->idleTimeVar     = 0;
    ptrChirpCfg->adcStartTimeVar = 0;
    ptrChirpCfg->txEnable        = 7;
}

/**
 *  @b Description
 *  @n
 *      Utility function which populates the BPM chirp configuration with
 *      well defined defaults.
 *
 *  @param[out]  ptrBpmChirpCfg
 *      Pointer to the populated BPM chirp configuration
 *
 *  @retval
 *      Not applicable
 */
static void Mmwave_populateDefaultBpmChirpCfg (rlBpmChirpCfg_t* ptrBpmChirpCfg)
{
    /* Initialize the configuration: */
    memset ((void*)ptrBpmChirpCfg, 0, sizeof(rlBpmChirpCfg_t));

    ptrBpmChirpCfg->chirpStartIdx   = 0;
    ptrBpmChirpCfg->chirpEndIdx     = 1;
    ptrBpmChirpCfg->constBpmVal     = 2;
}

/**
 *  @b Description
 *  @n
 *      Utility function which populates the default advanced frame configuration
 *
 *  @param[out]  ptrAdvFrameCfg
 *      Pointer to the populated advanced frame configuration
 *
 *  @retval
 *      Not applicable
 */
static void Mmwave_populateDefaultAdvancedFrameCfg (rlAdvFrameCfg_t* ptrAdvFrameCfg)
{
    uint8_t numOfSubFrame = 0U;

    /* Initialize the advanced frame configuration: */
    memset ((void*)ptrAdvFrameCfg, 0, sizeof(rlAdvFrameCfg_t));

    ptrAdvFrameCfg->frameSeq.forceProfile  = 0; /* 1: force Profile,  0: Don't force profile */
    ptrAdvFrameCfg->frameSeq.numFrames     = 0; /* infinite */
    ptrAdvFrameCfg->frameSeq.triggerSelect = 1; /* SW Trigger */
    ptrAdvFrameCfg->frameSeq.frameTrigDelay= 0;

    ptrAdvFrameCfg->frameSeq.subFrameCfg[0].forceProfileIdx    = 0;
    ptrAdvFrameCfg->frameSeq.subFrameCfg[0].numLoops           = 64;
    ptrAdvFrameCfg->frameSeq.subFrameCfg[0].numOfBurst         = 1;
    ptrAdvFrameCfg->frameSeq.subFrameCfg[0].numOfBurstLoops    = 1;
    ptrAdvFrameCfg->frameSeq.subFrameCfg[0].chirpStartIdxOffset= 0;
    ptrAdvFrameCfg->frameSeq.subFrameCfg[0].numOfChirps        = 1;
    ptrAdvFrameCfg->frameSeq.subFrameCfg[0].chirpStartIdx      = 0;
    ptrAdvFrameCfg->frameSeq.subFrameCfg[0].subFramePeriodicity= 20200000; /* 101 msec */
    ptrAdvFrameCfg->frameSeq.subFrameCfg[0].burstPeriodicity   = 20000000; /* 100 msec */

    ptrAdvFrameCfg->frameData.subframeDataCfg[0].numAdcSamples = PROFILE_0_ADC_SAMPLE_VAL*2;
    ptrAdvFrameCfg->frameData.subframeDataCfg[0].totalChirps   = 64;
    ptrAdvFrameCfg->frameData.subframeDataCfg[0].numChirpsInDataPacket = 1;
    numOfSubFrame++;

    ptrAdvFrameCfg->frameSeq.subFrameCfg[1].forceProfileIdx    = 0;
    ptrAdvFrameCfg->frameSeq.subFrameCfg[1].numLoops           = 32;
    ptrAdvFrameCfg->frameSeq.subFrameCfg[1].numOfBurst         = 1;
    ptrAdvFrameCfg->frameSeq.subFrameCfg[1].numOfBurstLoops    = 1;
    ptrAdvFrameCfg->frameSeq.subFrameCfg[1].chirpStartIdxOffset= 0;
    ptrAdvFrameCfg->frameSeq.subFrameCfg[1].numOfChirps        = 1;
    ptrAdvFrameCfg->frameSeq.subFrameCfg[1].chirpStartIdx      = 1;
    ptrAdvFrameCfg->frameSeq.subFrameCfg[1].subFramePeriodicity= 20200000; /*101 msec */
    ptrAdvFrameCfg->frameSeq.subFrameCfg[1].burstPeriodicity   = 20000000; /* 100 msec */

    ptrAdvFrameCfg->frameData.subframeDataCfg[1].numAdcSamples = PROFILE_1_ADC_SAMPLE_VAL*2;
    ptrAdvFrameCfg->frameData.subframeDataCfg[1].totalChirps   = 32;
    ptrAdvFrameCfg->frameData.subframeDataCfg[1].numChirpsInDataPacket = 1;
    numOfSubFrame++;

    ptrAdvFrameCfg->frameSeq.numOfSubFrames = numOfSubFrame;
    ptrAdvFrameCfg->frameData.numSubFrames  = numOfSubFrame;

}

/**
 *  @b Description
 *  @n
 *      The function is used to populate the default open configuration.
 *
 *  @param[out]  ptrOpenCfg
 *      Pointer to the open configuration
 *
 *  @retval
 *      Not applicable
 */
void Mmwave_populateDefaultOpenCfg (MMWave_OpenCfg* ptrOpenCfg)
{
    uint32_t index = 0;

    /* Initialize the open configuration: */
    memset ((void*)ptrOpenCfg, 0, sizeof(MMWave_OpenCfg));

#ifdef SUBSYS_MSS
    CacheP_wbInv((void*)ptrOpenCfg, sizeof(MMWave_OpenCfg), CacheP_TYPE_ALLD);
#endif

    /* Setup the frequency for calibrations. */
    ptrOpenCfg->freqLimitLow  = 760U;
    ptrOpenCfg->freqLimitHigh = 810U;

    /* Enable start/stop async events */
    ptrOpenCfg->disableFrameStartAsyncEvent = false;
    ptrOpenCfg->disableFrameStopAsyncEvent  = false;

    for(index = 0; index < MMWAVE_RADAR_DEVICES; index++)
    {
        if(index == 0)
        {
            /* Initialize the channel configuration: */
            ptrOpenCfg->frontEndCfg[index].chCfg.rxChannelEn = (1U << TEST_NUM_RX) - 1U;
            ptrOpenCfg->frontEndCfg[index].chCfg.txChannelEn = 0x7;
            ptrOpenCfg->frontEndCfg[index].chCfg.cascading   = 0x0001; /* MULTICHIP_MASTER */
        }
        else
        {
            ptrOpenCfg->frontEndCfg[index].chCfg.rxChannelEn = (1U << TEST_NUM_RX) - 1U;
            ptrOpenCfg->frontEndCfg[index].chCfg.txChannelEn = 0x7;
            ptrOpenCfg->frontEndCfg[index].chCfg.cascading   = 0x0002; /* MULTICHIP_SLAVE */
            ptrOpenCfg->frontEndCfg[index].chCfg.cascadingPinoutCfg   |= (1 << 5); /* Disable OSC clock out for slave. */
        }
    }

    /* Initialize the low power mode configuration: */
    ptrOpenCfg->lowPowerMode.lpAdcMode     = 0;

    /* Initialize the ADCOut configuration: */
    ptrOpenCfg->adcOutCfg.fmt.b2AdcBits   = 2;
    ptrOpenCfg->adcOutCfg.fmt.b2AdcOutFmt = 1;
    
    /*Initialize the datapath configuration*/
    ptrOpenCfg->dataPathCfg.intfSel         = 0;
    ptrOpenCfg->dataPathCfg.transferFmtPkt0 = 1;
    ptrOpenCfg->dataPathCfg.transferFmtPkt1 = 0;
    ptrOpenCfg->dataPathCfg.cqConfig        = 0x02;
    ptrOpenCfg->dataPathCfg.cq0TransSize    = 132;
    ptrOpenCfg->dataPathCfg.cq1TransSize    = 132;
    ptrOpenCfg->dataPathCfg.cq2TransSize    = 72;
    ptrOpenCfg->dataPathCfg.reserved        = 0	;
    
    /*Initialize the datapath clock configuration*/
    ptrOpenCfg->dataPathClkCfg.laneClkCfg  = 1; /* DDR Clock */
    ptrOpenCfg->dataPathClkCfg.dataRate    = 1; /* 600Mbps */
    ptrOpenCfg->dataPathClkCfg.reserved    = 0;
    
    /*Initialize the HSI clock configuration*/
    ptrOpenCfg->hsiClkCfg.hsiClk   = 0x9; /* 600Mbps DDR */
    ptrOpenCfg->hsiClkCfg.reserved = 0;

    /*Initialize the CSI2 configuration*/
    ptrOpenCfg->csi2Cfg.lanePosPolSel = 0x35421;
    ptrOpenCfg->csi2Cfg.reserved1     = 0;

    /*Initialize the lane configuration*/
    ptrOpenCfg->laneEnCfg.laneEn   = 0xF;
    ptrOpenCfg->laneEnCfg.reserved = 0;

    /* No custom calibration: */
    ptrOpenCfg->useCustomCalibration        = false;
    ptrOpenCfg->customCalibrationEnableMask = 0x0;

    /* calibration monitoring base time unit
     * setting it to one frame duration as the test doesnt support any 
     * monitoring related functionality
     */
    ptrOpenCfg->calibMonTimeUnit            = 1;
}

/**
 *  @b Description
 *  @n
 *      The function is used to populate the default control configuration
 *      in chirp configuration mode
 *
 *  @param[out]  ptrCtrlCfg
 *      Pointer to the control configuration
 *
 *  @retval
 *      Not applicable
 */
void Mmwave_populateDefaultChirpControlCfg (MMWave_CtrlCfg* ptrCtrlCfg)
{
    rlProfileCfg_t      profileCfg;
    rlChirpCfg_t        chirpCfg;
    int32_t             errCode;
    MMWave_ChirpHandle  chirpHandle[TEST_MAX_PROFILE][TEST_NUM_CHIRPS];
    MMWave_PhaseShiftChirpHandle phasechirpHandle;
    int32_t             devidx;
    uint32_t            numChirps = 0;
    uint32_t            loopIndex = 0;

    /* Initialize the control configuration: */
    memset ((void*)ptrCtrlCfg, 0, sizeof(MMWave_CtrlCfg));

    /* This is frame mode configuration */
    ptrCtrlCfg->dfeDataOutputMode = MMWave_DFEDataOutputMode_FRAME;

    /* Populate the profile configuration: */
    Mmwave_populateDefaultProfileCfg (&profileCfg);
    
    for(devidx = 0 ; devidx < MMWAVE_RADAR_DEVICES ; devidx++)
    {
        /* Create the profile: */
        ptrCtrlCfg->u.frameCfg[devidx].profileHandle[0] = MMWave_addProfile (gMmwCascadeMCB.mmWaveHandle, &profileCfg, &errCode);
        if (ptrCtrlCfg->u.frameCfg[devidx].profileHandle[0] == NULL)
        {
            test_print ("Error: Unable to add the profile [Error code %d]\n", errCode);
            MCPI_setFeatureTestResult ("MMWave Add Profile", MCPI_TestResult_FAIL);
            return;
        }
        MCPI_setFeatureTestResult ("MMWave Add Profile", MCPI_TestResult_PASS);

        /**************************************************************************************************
         * Unit Test: Verify the Full Configuration Profile API
         **************************************************************************************************/
        {
            rlProfileCfg_t          profileCfgTmp;
            uint32_t                numProfiles;
            MMWave_ProfileHandle    tmpProfileHandle;

            /* Verify the number of profiles */
            if (MMWave_getNumProfiles (gMmwCascadeMCB.mmWaveHandle, &numProfiles, &errCode) < 0)
            {
                test_print ("Error: Unable to get the number of profiles [Error code %d]\n", errCode);
                MCPI_setFeatureTestResult ("MMWave Get Number Profile", MCPI_TestResult_FAIL);
                return;
            }

            MCPI_setFeatureTestResult ("MMWave Get Number Profile", MCPI_TestResult_PASS);

            /* Get the profile handle: */
            if (MMWave_getProfileHandle (gMmwCascadeMCB.mmWaveHandle, 0U, &tmpProfileHandle, &errCode) < 0)
            {
                test_print ("Error: Unable to get the profile handle [Error code %d]\n", errCode);
                MCPI_setFeatureTestResult ("MMWave Get Profile Handle", MCPI_TestResult_FAIL);
            }
            if (tmpProfileHandle != ptrCtrlCfg->u.frameCfg[devidx].profileHandle[0])
            {
                test_print ("Error: Invalid profile handle detected\n");
                MCPI_setFeatureTestResult ("MMWave Get Profile Handle", MCPI_TestResult_FAIL);
            }
            MCPI_setFeatureTestResult ("MMWave Get Profile Handle", MCPI_TestResult_PASS);

            /* Get the profile configuration */
            if (MMWave_getProfileCfg (ptrCtrlCfg->u.frameCfg[devidx].profileHandle[0], &profileCfgTmp, &errCode) < 0)
            {
                test_print ("Error: Unable to get the profile configuration [Error code %d]\n", errCode);
                MCPI_setFeatureTestResult ("MMWave Get Profile", MCPI_TestResult_FAIL);
                return;
            }
            if (memcmp ((void*)&profileCfg, (void*)&profileCfgTmp, sizeof(rlProfileCfg_t)) != 0)
            {
                test_print ("Error: Invalid profile configuration detected\n");
                MCPI_setFeatureTestResult ("MMWave Get Profile", MCPI_TestResult_FAIL);
                return;
            }
            MCPI_setFeatureTestResult ("MMWave Get Profile", MCPI_TestResult_PASS);
        }
        MCPI_setFeatureTestResult ("MMWave Profile Addition", MCPI_TestResult_PASS);

        /**********************************************************************************
         * Unit Test: Chirp Addition
         **********************************************************************************/
        numChirps = 0U;
        /* For each profile create the chirps */
        for (loopIndex = 0U; loopIndex < TEST_NUM_CHIRPS; loopIndex++)
        {
            /* Initialize the chirp configuration: */
            memset ((void*)&chirpCfg, 0, sizeof(rlChirpCfg_t));

            /* Populate the *default* chirp configuration: */
            Mmwave_populateDefaultChirpCfg(&chirpCfg);

            /* TDMA: exactly one TX fires per chirp.
            chirps 0-2 = device0 TX0/TX1/TX2,  chirps 3-5 = device1 TX0/TX1/TX2 */
            if (devidx == 0)
                chirpCfg.txEnable = (loopIndex < 3) ? (1U << loopIndex)       : 0U;
            else
                chirpCfg.txEnable = (loopIndex >= 3) ? (1U << (loopIndex - 3)) : 0U;

            /* Override the chirp configuration: */
            chirpCfg.chirpStartIdx   = numChirps;
            chirpCfg.chirpEndIdx     = numChirps;
            chirpCfg.profileId       = 0;

            CacheP_wb((void *)&chirpCfg, sizeof(rlChirpCfg_t), CacheP_TYPE_ALLD);

            /* Add the chirp: */
            chirpHandle[0][loopIndex] = MMWave_addChirp (ptrCtrlCfg->u.frameCfg[devidx].profileHandle[0], &chirpCfg, &errCode);
            if (chirpHandle[0][loopIndex] == NULL)
            {
                test_print ("Error: Unable to add the chirp index %d profile index %d [Error code %d]\n", loopIndex, 0, errCode);
                MCPI_setFeatureTestResult ("Chirp Addition", MCPI_TestResult_FAIL);
            }

            /* Increment the number of chirps which have been created */
            numChirps = numChirps + 1U;
        }
        MCPI_setFeatureTestResult ("Chirp Addition", MCPI_TestResult_PASS);

        /**************************************************************************************************
         * Unit Test: Verify the Full Configuration Chirp API
         **************************************************************************************************/
        {
            rlChirpCfg_t        chirpCfgTmp;
            uint32_t            numChirps;
            MMWave_ChirpHandle  chirpHandleTmp;

            /* Get the number of chirps attached to the profile */
            if (MMWave_getNumChirps (ptrCtrlCfg->u.frameCfg[devidx].profileHandle[0], &numChirps, &errCode) < 0)
            {
                test_print ("Error: Unable to get the number of chirps [Error code %d]\n", errCode);
                MCPI_setFeatureTestResult ("MMWave Get Number of Chirps", MCPI_TestResult_FAIL);
                return;
            }
            if (numChirps != TEST_NUM_CHIRPS)
            {
                test_print ("Error: Invalid number of chirps detected [%d]\n", numChirps);
                MCPI_setFeatureTestResult ("MMWave Get Number of Chirps", MCPI_TestResult_FAIL);
                return;
            }
            MCPI_setFeatureTestResult ("MMWave Get Number of Chirps", MCPI_TestResult_PASS);

            /* Get the Chirp Handle */
            if (MMWave_getChirpHandle (ptrCtrlCfg->u.frameCfg[devidx].profileHandle[0], 1U, &chirpHandleTmp, &errCode) < 0)
            {
                test_print ("Error: Unable to get the chirp handle [Error code %d]\n", errCode);
                MCPI_setFeatureTestResult ("MMWave Get Chirp Handle", MCPI_TestResult_FAIL);
                return;
            }
            if (chirpHandleTmp != chirpHandle[0][0])
            {
                test_print ("Error: Chirp handle validation failed [Error code %d]\n", errCode);
                MCPI_setFeatureTestResult ("MMWave Get Chirp Handle", MCPI_TestResult_FAIL);
                return;
            }
            MCPI_setFeatureTestResult ("MMWave Get Chirp Handle", MCPI_TestResult_PASS);

            /* Get the chirp configuration */
            if (MMWave_getChirpCfg (chirpHandle[0][0], &chirpCfgTmp, &errCode) < 0)
            {
                test_print ("Error: Unable to get the profile configuration [Error code %d]\n", errCode);
                MCPI_setFeatureTestResult ("MMWave Get Chirp", MCPI_TestResult_FAIL);
                return;
            }

            MCPI_setFeatureTestResult ("MMWave Get Chirp Configuration", MCPI_TestResult_PASS);
        }

        /**************************************************************************************************
         * Configure PhaseShift Chirps
         **************************************************************************************************/
        {
            for(loopIndex = 0; loopIndex < NUM_OF_PHASE_SHIFT_CHIRPS; loopIndex++)
            {
                if(devidx == 0)
                {
                    phasechirpHandle = MMWave_addPhaseShiftChirp (gMmwCascadeMCB.mmWaveHandle, &phaseShifChirpCfgDev1[loopIndex], &errCode);
                    ptrCtrlCfg->numOfPhaseShiftChirps[devidx] +=1U;
                }
                else
                {
                    phasechirpHandle = MMWave_addPhaseShiftChirp (gMmwCascadeMCB.mmWaveHandle, &phaseShifChirpCfgDev2[loopIndex], &errCode);
                    ptrCtrlCfg->numOfPhaseShiftChirps[devidx] +=1U;
                }

                /* */
                if (phasechirpHandle == NULL)
                {
                    test_print ("Error: Unable to add the phase shift chirp [Error code %d]\n", errCode);
                    return;
                }
            }
        }

        /* Populate the frame configuration: */
        ptrCtrlCfg->u.frameCfg[devidx].frameCfg.chirpStartIdx      = 0;
        ptrCtrlCfg->u.frameCfg[devidx].frameCfg.chirpEndIdx        = TEST_CHIRP_END_INDEX;
        ptrCtrlCfg->u.frameCfg[devidx].frameCfg.numLoops           = TEST_NUM_LOOPS;
        ptrCtrlCfg->u.frameCfg[devidx].frameCfg.numFrames          = TEST_NUM_FRAMES;
        ptrCtrlCfg->u.frameCfg[devidx].frameCfg.numAdcSamples      = TEST_NUM_ADC_SAMPLES * 2 ;
        // ptrCtrlCfg->u.frameCfg[devidx].frameCfg.framePeriodicity   = 200 * 1000000 / 5;
        ptrCtrlCfg->u.frameCfg[devidx].frameCfg.framePeriodicity   = 50 * 1000000 / 5;

        ptrCtrlCfg->u.frameCfg[devidx].frameCfg.frameTriggerDelay  = 0;
    }

    ptrCtrlCfg->u.frameCfg[0].frameCfg.triggerSelect      = 1; /* MULTICHIP_MASTER */
    ptrCtrlCfg->u.frameCfg[1].frameCfg.triggerSelect      = 2; /* MULTICHIP_SLAVE */

    return;
}


/**
 *  @b Description
 *  @n
 *      The function initializes the profile configuration with the default
 *      parameters.
 *
 *  @param[in] profileNum
 *      Profile number to be initialized
 *  @param[out] ptrProfileCfg
 *      Pointer to the profile configuration
 *
 *  @retval
 *      Not applicable
 */
void Mmwave_populateDefaultAdvancedFrameProfileCfg (uint8_t profileNum, rlProfileCfg_t* ptrProfileCfg)
{
    /* Initialize the configuration: */
    memset ((void*)ptrProfileCfg, 0, sizeof(rlProfileCfg_t));

    if (profileNum == 0U)
    {
        /* Populate the default configuration for profile0  */
        ptrProfileCfg->profileId             = PROFILE_0_PROFILE_ID;
        ptrProfileCfg->startFreqConst        = PROFILE_0_START_FREQ_VAL;
        ptrProfileCfg->idleTimeConst         = PROFILE_0_IDLE_TIME_VAL;
        ptrProfileCfg->adcStartTimeConst     = PROFILE_0_ADC_START_TIME_VAL;
        ptrProfileCfg->rampEndTime           = PROFILE_0_RAMP_END_TIME_VAL;
        ptrProfileCfg->txOutPowerBackoffCode = PROFILE_0_TXOUT_POWER_BACKOFF;
        ptrProfileCfg->txPhaseShifter        = PROFILE_0_TXPHASESHIFTER_VAL;
        ptrProfileCfg->freqSlopeConst        = PROFILE_0_FREQ_SLOPE_VAL;
        ptrProfileCfg->txStartTime           = PROFILE_0_TX_START_TIME_VAL;
        ptrProfileCfg->numAdcSamples         = PROFILE_0_ADC_SAMPLE_VAL;
        ptrProfileCfg->digOutSampleRate      = PROFILE_0_DIGOUT_SAMPLERATE_VAL;
        ptrProfileCfg->hpfCornerFreq1        = PROFILE_0_HPFCORNER_FREQ1_VAL;
        ptrProfileCfg->hpfCornerFreq2        = PROFILE_0_HPFCORNER_FREQ2_VAL;
        ptrProfileCfg->rxGain                = PROFILE_0_RX_GAIN_VAL;
    }
    else
    {
        /* Populate the default configuration for profile0  */
        ptrProfileCfg->profileId             = PROFILE_1_PROFILE_ID;
        ptrProfileCfg->startFreqConst        = PROFILE_1_START_FREQ_VAL;
        ptrProfileCfg->idleTimeConst         = PROFILE_1_IDLE_TIME_VAL;
        ptrProfileCfg->adcStartTimeConst     = PROFILE_1_ADC_START_TIME_VAL;
        ptrProfileCfg->rampEndTime           = PROFILE_1_RAMP_END_TIME_VAL;
        ptrProfileCfg->txOutPowerBackoffCode = PROFILE_1_TXOUT_POWER_BACKOFF;
        ptrProfileCfg->txPhaseShifter        = PROFILE_1_TXPHASESHIFTER_VAL;
        ptrProfileCfg->freqSlopeConst        = PROFILE_1_FREQ_SLOPE_VAL;
        ptrProfileCfg->txStartTime           = PROFILE_1_TX_START_TIME_VAL;
        ptrProfileCfg->numAdcSamples         = PROFILE_1_ADC_SAMPLE_VAL;
        ptrProfileCfg->digOutSampleRate      = PROFILE_1_DIGOUT_SAMPLERATE_VAL;
        ptrProfileCfg->hpfCornerFreq1        = PROFILE_1_HPFCORNER_FREQ1_VAL;
        ptrProfileCfg->hpfCornerFreq2        = PROFILE_1_HPFCORNER_FREQ2_VAL;
        ptrProfileCfg->rxGain                = PROFILE_1_RX_GAIN_VAL;
    }
    return;
}

/**
 *  @b Description
 *  @n
 *      The function initializes the chirp configuration with the default
 *      parameters.
 *
 *  @param[out] chirpNum
 *      Chirp Number to be configured
 *  @param[out] ptrChirpCfg
 *      Pointer to the chirp configuration
 *
 *  @retval
 *      Not applicable
 */
static void Mmwave_populateDefaultAdvancedFrameChirpCfg (uint8_t chirpNum, rlChirpCfg_t* ptrChirpCfg)
{
    /* Initialize the configuration: */
    memset ((void*)ptrChirpCfg, 0, sizeof(rlChirpCfg_t));

    if (chirpNum == 0U)
    {
        /* Populate the default configuration for chirp 0 */
        ptrChirpCfg->profileId       = CHIRP_0_PROFILE_ID;
        ptrChirpCfg->adcStartTimeVar = CHIRP_0_ADC_START_TIME_VAL;
        ptrChirpCfg->chirpEndIdx     = CHIRP_0_START_INDEX;
        ptrChirpCfg->chirpStartIdx   = CHIRP_0_END_INDEX;
        ptrChirpCfg->idleTimeVar     = CHIRP_0_IDLE_TIME_VAL;
        ptrChirpCfg->txEnable        = CHIRP_1_TX_CHANNEL;
        ptrChirpCfg->startFreqVar    = CHIRP_0_START_FREQ_VAL;
        ptrChirpCfg->freqSlopeVar    = CHIRP_0_FREQ_SLOPE_VAL;
    }
    else
    {
        /* Populate the default configuration for chirp 1 */
        ptrChirpCfg->profileId       = CHIRP_1_PROFILE_ID;
        ptrChirpCfg->adcStartTimeVar = CHIRP_1_ADC_START_TIME_VAL;
        ptrChirpCfg->chirpEndIdx     = CHIRP_1_START_INDEX;
        ptrChirpCfg->chirpStartIdx   = CHIRP_1_END_INDEX;
        ptrChirpCfg->idleTimeVar     = CHIRP_1_IDLE_TIME_VAL;
        ptrChirpCfg->txEnable        = CHIRP_1_TX_CHANNEL;
        ptrChirpCfg->startFreqVar    = CHIRP_1_START_FREQ_VAL;
        ptrChirpCfg->freqSlopeVar    = CHIRP_1_FREQ_SLOPE_VAL;
    }
    return;
}

/**
 *  @b Description
 *  @n
 *      The function is used to populate the default control configuration
 *      in chirp configuration mode
 *
 *  @param[out]  ptrCtrlCfg
 *      Pointer to the control configuration
 *
 *  @retval
 *      Not applicable
 */
void Mmwave_populateDefaultAdvancedControlCfg (MMWave_CtrlCfg* ptrCtrlCfg)
{
    rlProfileCfg_t         profileCfg;
    rlChirpCfg_t           chirpCfg;
    rlBpmChirpCfg_t        bpmChirpCfg;
    int32_t                errCode;
    MMWave_ChirpHandle     chirpHandle;
    MMWave_BpmChirpHandle  bpmChirpHandle;
    uint32_t devidx;

    /* Initialize the control configuration: */
    memset ((void*)ptrCtrlCfg, 0, sizeof(MMWave_CtrlCfg));

    /* This is frame mode configuration */
    ptrCtrlCfg->dfeDataOutputMode = MMWave_DFEDataOutputMode_ADVANCED_FRAME;

    /* Populate the profile configuration: */
    Mmwave_populateDefaultAdvancedFrameProfileCfg (0U, &profileCfg);
    for(devidx = 0; devidx < MMWAVE_RADAR_DEVICES ; devidx++)
    {
        /* Create the profile: */
        ptrCtrlCfg->u.advancedFrameCfg[devidx].profileHandle[0] = MMWave_addProfile (gMmwCascadeMCB.mmWaveHandle, &profileCfg, &errCode);
        if (ptrCtrlCfg->u.advancedFrameCfg[devidx].profileHandle[0] == NULL)
        {
            test_print ("Error: Unable to add the profile [Error code %d]\n", errCode);
            MCPI_setFeatureTestResult ("MMWave Add Profile", MCPI_TestResult_FAIL);
            return;
        }
        MCPI_setFeatureTestResult ("MMWave Add Profile", MCPI_TestResult_PASS);

        /* Populate the profile configuration: */
        Mmwave_populateDefaultAdvancedFrameProfileCfg (1U, &profileCfg);

        /* Create the profile: */
        ptrCtrlCfg->u.advancedFrameCfg[devidx].profileHandle[1] = MMWave_addProfile (gMmwCascadeMCB.mmWaveHandle, &profileCfg, &errCode);
        if (ptrCtrlCfg->u.advancedFrameCfg[devidx].profileHandle[1] == NULL)
        {
            test_print ("Error: Unable to add the profile [Error code %d]\n", errCode);
            MCPI_setFeatureTestResult ("MMWave Add Profile", MCPI_TestResult_FAIL);
            return;
        }
        MCPI_setFeatureTestResult ("MMWave Add Profile", MCPI_TestResult_PASS);

        /* Populate the default chirp configuration */
        Mmwave_populateDefaultAdvancedFrameChirpCfg (0U, &chirpCfg);

        /* Add the chirp to the profile: */
        chirpHandle = MMWave_addChirp (ptrCtrlCfg->u.advancedFrameCfg[devidx].profileHandle[0], &chirpCfg, &errCode);
        if (chirpHandle == NULL)
        {
            test_print ("Error: Unable to add the chirp [Error code %d]\n", errCode);
            MCPI_setFeatureTestResult ("MMWave Add Chirp", MCPI_TestResult_FAIL);
            return;
        }
        MCPI_setFeatureTestResult ("MMWave Add Chirp", MCPI_TestResult_PASS);

        /* Populate the default chirp configuration */
        Mmwave_populateDefaultAdvancedFrameChirpCfg (1U, &chirpCfg);

        /* Add the chirp to the profile: */
        chirpHandle = MMWave_addChirp (ptrCtrlCfg->u.advancedFrameCfg[devidx].profileHandle[1], &chirpCfg, &errCode);
        if (chirpHandle == NULL)
        {
            test_print ("Error: Unable to add the chirp [Error code %d]\n", errCode);
            MCPI_setFeatureTestResult ("MMWave Add Chirp", MCPI_TestResult_FAIL);
            return;
        }
        MCPI_setFeatureTestResult ("MMWave Add Chirp", MCPI_TestResult_PASS);

        /* Populate the BPM configuration. */
        Mmwave_populateDefaultBpmChirpCfg (&bpmChirpCfg);
        bpmChirpHandle = MMWave_addBpmChirp (gMmwCascadeMCB.mmWaveHandle, &bpmChirpCfg, &errCode);
        if (bpmChirpHandle == NULL)
        {
            test_print ("Error: Unable to add BPM chirp [Error code %d]\n", errCode);
            MCPI_setFeatureTestResult ("MMWave Add BPM Chirp", MCPI_TestResult_FAIL);
            return;
        }
        MCPI_setFeatureTestResult ("MMWave Add BPM Chirp", MCPI_TestResult_PASS);

        /* Populate the advanced frame configuration: */
        Mmwave_populateDefaultAdvancedFrameCfg(&ptrCtrlCfg->u.advancedFrameCfg[devidx].frameCfg);
    }
    return;
}


/**
 *  @b Description
 *  @n
 *      Registered event function which is invoked when an event from the
 *      BSS is received.
 *
 *  @param[in]  msgId
 *      Message Identifier
 *  @param[in]  sbId
 *      Subblock identifier
 *  @param[in]  sbLen
 *      Length of the subblock
 *  @param[in]  payload
 *      Pointer to the payload buffer
 *
 *  @retval
 *      Always return 0 to pass the event to the peer domain- there is no more peer domain, this can be void
 */
int32_t Mmwave_eventFxn(uint8_t devIndex, uint16_t msgId, uint16_t sbId, uint16_t sbLen, uint8_t *payload)
{
    uint16_t asyncSB = RL_GET_SBID_FROM_UNIQ_SBID(sbId);

    /* Process the received message: */
    switch (msgId)
    {
        case RL_RF_ASYNC_EVENT_MSG:
        {
            /* Received Asychronous Message: */
            switch (asyncSB)
            {
                case RL_RF_AE_INITCALIBSTATUS_SB:
                {
                    rlRfInitComplete_t*  ptrRFInitCompleteMessage;
                    uint32_t            calibrationStatus;

                    /* Get the RF-Init completion message: */
                    ptrRFInitCompleteMessage = (rlRfInitComplete_t*)payload;
                    calibrationStatus = ptrRFInitCompleteMessage->calibStatus & 0x1FFFU;

                    /* Display the calibration status: */
                    
                    DebugP_log("Debug: Device %d Init Calibration Status = 0x%x\n", devIndex, calibrationStatus);;
                    break;
                }
            }
        }
    }
    return 0;
}



/**
 *  @b Description
 *  @n
 *      The function is used to populate the default calibration
 *      configuration which is passed to start the mmWave module
 *
 *  @retval
 *      Not applicable
 */
void Mmwave_populateDefaultCalibrationCfg (MMWave_CalibrationCfg* ptrCalibrationCfg, MMWave_DFEDataOutputMode dfeOutputMode)
{
    /* Populate the calibration configuration: */
    ptrCalibrationCfg->dfeDataOutputMode                          = dfeOutputMode;
    ptrCalibrationCfg->u.chirpCalibrationCfg.enableCalibration    = false;
    ptrCalibrationCfg->u.chirpCalibrationCfg.enablePeriodicity    = false;
    ptrCalibrationCfg->u.chirpCalibrationCfg.periodicTimeInFrames = 10U;
    ptrCalibrationCfg->u.chirpCalibrationCfg.reportEn             = 0;
    return;
}


/**
 *  @b Description
 *  @n
 *      The task is used to provide an execution context for the mmWave
 *      control task
 *
 *  @retval
 *      Not Applicable.
 */
void Mmwave_ctrlTask(void* args)
{
    int32_t errCode;

    while (1)
    {
        /* Execute the mmWave control module: */
        if (MMWave_execute (gMmwCascadeMCB.mmWaveHandle, &errCode) < 0)
            test_print ("Error: mmWave control execution failed [Error code %d]\n", errCode);
    }
}
