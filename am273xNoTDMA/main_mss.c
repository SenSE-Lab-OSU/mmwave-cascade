/*
 *   @file  main_full_mss.c
 *
 *   @brief
 *      Unit Test code for the mmWave 
 *
 *  \par
 *  NOTE:
 *      (C) Copyright 2016-2021 Texas Instruments, Inc.
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

/* MCU Plus Include Files. */
#include <ti/utils/test/cascade/am273x/mssgenerated/ti_drivers_config.h>
#include <ti/utils/test/cascade/am273x/mssgenerated/ti_board_config.h>
#include <ti/utils/test/cascade/am273x/mssgenerated/ti_drivers_open_close.h>
#include <ti/utils/test/cascade/am273x/mssgenerated/ti_board_open_close.h>
#include <kernel/dpl/AddrTranslateP.h>
#include <kernel/dpl/SemaphoreP.h>
#include <kernel/dpl/SystemP.h>
#include <kernel/dpl/CacheP.h>
#include <kernel/dpl/DebugP.h>
#include "FreeRTOS.h"
#include "task.h"
#include <drivers/csirx.h>


/* mmWave SK Include Files: */
#include <ti/common/syscommon.h>
#include <ti/control/mmwavelink/mmwavelink.h>
#include <ti/control/mmwave/mmwave.h>
#include <ti/utils/testlogger/logger.h>
#include <ti/utils/test/cascade/am273x/cascade_csirx.h>

/**************************************************************************
 ******************************** MACROS  *********************************
 **************************************************************************/
/**
 * @brief
 *  The DCA1000EVM FPGA needs a minimum delay of 12ms between Bit clock starts and
 *  actual LVDS Data start to lock the LVDS PLL IP. This is documented in the DCA UG
 */
#define HSI_DCA_MIN_DELAY_MSEC     (12U * 1000U)

#define APP_TASK_PRI               (5U)
#define CSI_CONFIG_TASK_PRI        (6U)
#define MMW_CTRL_TASK_PRI          (7U)
#define MMW_TEST_TASK_PRI          (5U)

#define APP_TASK_STACK_SIZE        (32U * 1024U)
#define CSI_CONFIG_TASK_STACK_SIZE (32U * 1024U)
#define MMW_CTRL_TASK_STACK_SIZE   (16U * 1024U)
#define MMW_TEST_TASK_STACK_SIZE   (16U * 1024U)

/* size used for communication with PMIC. */
#define PMIC_MSGSIZE                  (4U)

/* OFFSET for configuring BUCK4. */
#define PMIC_CONFIG_BUCK4_REG_ADDR    (0x0A)

TaskHandle_t    gAppTask;
StaticTask_t    gAppTaskObj;

TaskHandle_t    gCsiConfigTask;
StaticTask_t    gCsiConfigTaskObj;

TaskHandle_t    gMmwCtrlTask;
StaticTask_t    gMmwCtrlTaskObj;

StackType_t gAppTskStackMain[APP_TASK_STACK_SIZE] __attribute__((aligned(32)));
StackType_t gMmwCtrlTskStack[MMW_CTRL_TASK_STACK_SIZE] __attribute__((aligned(32)));
StackType_t gCsiRxCfgTskStack[CSI_CONFIG_TASK_STACK_SIZE] __attribute__((aligned(32)));

/**************************************************************************
 *************************** Global Variables *****************************
 **************************************************************************/

uint8_t CSIA_PingBuf[BOARD_DIAG_PING_OR_PONG_BUF_SIZE_ALIGNED]__attribute__ ((aligned(64), section(".l3ram")));

uint8_t CSIA_PongBuf[BOARD_DIAG_PING_OR_PONG_BUF_SIZE_ALIGNED]__attribute__ ((aligned(64), section(".l3ram")));

uint8_t CSIB_PingBuf[BOARD_DIAG_PING_OR_PONG_BUF_SIZE_ALIGNED]__attribute__ ((aligned(64), section(".l3ram")));

uint8_t CSIB_PongBuf[BOARD_DIAG_PING_OR_PONG_BUF_SIZE_ALIGNED]__attribute__ ((aligned(64), section(".l3ram")));

/**
 * @brief
 *  Initialize the MCPI Log Message Buffer
 */
MCPI_LOGBUF_INIT(9216);


/**
 * @brief
 *  Global Variable for tracking information required by the mmw Demo
 */
MmwCascade_MCB    gMmwCascadeMCB = {0U};

/**
 * @brief
 *  Global Variable used for storing CSIRXA instance
 */
MmwCascade_CSIRX_State gCSIRXState[MMWAVE_RADAR_DEVICES] ;

/**
 * @brief
 *  Global Variable for CSIRX error monitoring
 */
uint32_t gCSIRXErrorCode[MMWAVE_RADAR_DEVICES];

/**************************************************************************
 ************************** Extern Definitions ****************************
 **************************************************************************/
extern void Mmwave_populateDefaultOpenCfg (MMWave_OpenCfg* ptrOpenCfg);
extern void Mmwave_populateDefaultChirpControlCfg (MMWave_CtrlCfg* ptrCtrlCfg);
extern void Mmwave_populateDefaultAdvancedControlCfg (MMWave_CtrlCfg* ptrCtrlCfg);
extern void Mmwave_populateDefaultCalibrationCfg (MMWave_CalibrationCfg* ptrCalibrationCfg, MMWave_DFEDataOutputMode dfeOutputMode);
extern int32_t Mmwave_eventFxn (uint8_t devIndex,uint16_t msgId, uint16_t sbId, uint16_t sbLen, uint8_t *payload);
extern void Mmwave_ctrlTask(void* args);

/* CSI RX */;
extern void MmwCascade_csirxInit(MmwCascade_MCB  *CascadeMCB);
extern void MmwCascade_csirxOpen(MmwCascade_MCB  *CascadeMCB, int32_t *errCode);
static void MmwCascade_CsiConfigTask(void* args);
/**************************************************************************
 *********************** mmWave Unit Test Functions ***********************
 **************************************************************************/
static uint8_t readPmicReg(MIBSPI_Handle handle, uint8_t regOffset)
{
    uint8_t txBuffer[PMIC_MSGSIZE];
    uint8_t rxBuffer[PMIC_MSGSIZE];
    uint8_t regValue = 0U;
    MIBSPI_Transaction spiTransaction;

    /* Configure Data Transfer */
    spiTransaction.count = PMIC_MSGSIZE-1;
    spiTransaction.txBuf = txBuffer;
    spiTransaction.rxBuf = rxBuffer;
    spiTransaction.slaveIndex = 0;
    txBuffer[0] = regOffset;
    // Indicate PMIC a read sequence */
    txBuffer[1] = 0x10;
    txBuffer[2] = 0;

    CacheP_wbInv((void *)&txBuffer[0], PMIC_MSGSIZE, CacheP_TYPE_ALLD);

    /* Start Data Transfer */
    MIBSPI_transfer(handle, &spiTransaction);

    CacheP_inv((void *)&rxBuffer[0], PMIC_MSGSIZE, CacheP_TYPE_ALLD);

    /*PMIC GPIO Out register value */
    regValue = rxBuffer[2];

    return regValue;
}

volatile uint8_t pmicRegRead = 0;
/**
 *  @b Description
 *  @n
 *      Configures PMIC BUCK4
 *
 *  @retval
 *      Not Applicable.
 */
void Enable_BUCK4_ViaPMIC(void)
{
    MIBSPI_Transaction stTransaction = {0U};
    uint8_t u8TxBuff[4] = {0U};
    int32_t transferOK;

    /* Now, configure the PMIC */
    stTransaction.slaveIndex = 0U;
    stTransaction.rxBuf      = NULL;
    stTransaction.txBuf      = (void *)&u8TxBuff[0];
    stTransaction.count      = PMIC_MSGSIZE - 1U;

    u8TxBuff[0] = PMIC_CONFIG_BUCK4_REG_ADDR;    /* Offset */
    u8TxBuff[1] = 0x00U;    /* Page number + Write access */
    u8TxBuff[2] = 0x33U;    /*  BUCK 4 configuration*/

    /* It is important to invalidate the cache because the SPI driver will use eDMA transfer
    *   between the memory and the internal SPI RAM buffer. */
    CacheP_wbInv((void *)&u8TxBuff[0], PMIC_MSGSIZE, CacheP_TYPE_ALLD);

    transferOK = MIBSPI_transfer(gMmwCascadeMCB.pmicMIBSPIhandle, &stTransaction);

    ClockP_sleep(1);

    if((SystemP_SUCCESS != transferOK) ||
        (MIBSPI_TRANSFER_COMPLETED != stTransaction.status))
    {
        DebugP_assert(FALSE); /* MIBSPI transfer failed!! */
    }

    /* Read back and verify BUCK4 configuration. */
    pmicRegRead = readPmicReg(gMmwCascadeMCB.pmicMIBSPIhandle, PMIC_CONFIG_BUCK4_REG_ADDR);

    if(pmicRegRead == u8TxBuff[2])
    {
        test_print ("PMIC register 0x0%X configured to 0x%X.\n", PMIC_CONFIG_BUCK4_REG_ADDR, pmicRegRead);
    }
    else
    {
        test_print ("PMIC register config failed.\n");
    }

    return;
}


/**
 *  @b Description
 *  @n
 *      Test implementation
 *
 *  @retval
 *      Not Applicable.
 */
void MmwCascade_mmWaveTest (void)
{
    MMWave_InitCfg          initCfg;
    MMWave_CtrlCfg          ctrlCfg;
    MMWave_OpenCfg          openCfg;
    int32_t                 errCode;
    MMWave_CalibrationCfg   calibrationCfg;
    int32_t                 retVal;
    MMWave_ErrorLevel       errorLevel;
    int16_t                 mmWaveErrorCode;
    int16_t                 subsysErrorCode;
    uint32_t                u32DevIdx;

    /* Initialize the configuration: */
    memset ((void *)&initCfg, 0, sizeof(MMWave_InitCfg));

    initCfg.domain                  = MMWave_Domain_MSS;
    initCfg.eventFxn                = Mmwave_eventFxn;
    initCfg.linkCRCCfg.crcBaseAddr  = (uint32_t) AddrTranslateP_getLocalAddr(CONFIG_CRC0_BASE_ADDR);
    initCfg.linkCRCCfg.useCRCDriver = 1U;
    initCfg.linkCRCCfg.crcChannel   = CRC_CHANNEL_1;
    initCfg.cfgMode                 = MMWave_ConfigurationMode_FULL;

    /* Initialize and setup the mmWave Control module */
    gMmwCascadeMCB.mmWaveHandle = MMWave_init (&initCfg, &errCode);
    if (gMmwCascadeMCB.mmWaveHandle == NULL)
    {
        /* Error: Unable to initialize the mmWave control module */
        MMWave_decodeError (errCode, &errorLevel, &mmWaveErrorCode, &subsysErrorCode);

        /* Debug Message: */
        test_print ("Error Level: %s mmWave: %d Subsys: %d\n",
                       (errorLevel == MMWave_ErrorLevel_ERROR) ? "Error" : "Warning",
                       mmWaveErrorCode, subsysErrorCode);

        /* Log into the MCPI Test Logger: */
        MCPI_setFeatureTestResult ("MMWave MSS Initialization", MCPI_TestResult_FAIL);
        return;
    }
    
    test_print ("MMWave MSS Initialization\n");

    /*****************************************************************************
     * Launch the mmWave control execution task
     * - This should have a higher priroity than any other task which uses the
     *   mmWave control API
     *****************************************************************************/
    /* Launch the CSIRX Task */
    gMmwCtrlTask = xTaskCreateStatic( Mmwave_ctrlTask,   /* Pointer to the function that implements the task. */
                                "test_mmw_ctrl_task", /* Text name for the task.  This is to facilitate debugging only. */
                                MMW_CTRL_TASK_STACK_SIZE,  /* Stack depth in units of StackType_t typically uint32_t on 32b CPUs */
                                NULL,              /* We are not using the task parameter. */
                                MMW_CTRL_TASK_PRI,      /* task priority, 0 is lowest priority, configMAX_PRIORITIES-1 is highest */
                                gMmwCtrlTskStack,  /* pointer to stack base */
                                &gMmwCtrlTaskObj );    /* pointer to statically allocated task object memory */
    configASSERT(gMmwCtrlTask != NULL);

    Mmwave_populateDefaultOpenCfg (&openCfg);
    Mmwave_populateDefaultChirpControlCfg (&ctrlCfg);
    /* Mmwave_populateDefaultOpenCfg is memsetting the openCfg to zero, that is why
       had to save the spi handle in local var and populate it here*/

    for(u32DevIdx = 0U; u32DevIdx < MMWAVE_RADAR_DEVICES; u32DevIdx++)
    {
        if(u32DevIdx == 0)
        {
            /* Master AWR2243 */
            openCfg.frontEndCfg[u32DevIdx].spiHandle = gMibspiHandle[CONFIG_MIBSPI0];
            openCfg.frontEndCfg[u32DevIdx].gpioBaseAddr = (uint32_t) AddrTranslateP_getLocalAddr(NRESET_FE1_BASE_ADDR);

            /* Inform mmwave which GPIO pin is used for the front end NRESET*/
            openCfg.frontEndCfg[u32DevIdx].nresetGpioIndex = (uint32_t) NRESET_FE1_PIN;

            /* Inform mmwave which GPIO pin is used for the SPI IRQ*/
            openCfg.frontEndCfg[u32DevIdx].spiIrqGpioIndex = (uint32_t) RCSS_MIBSPIA_HOST_IRQ_PIN;

            /* Inform mmwave which interrupt to be configured for SPI IRQ*/
            openCfg.frontEndCfg[u32DevIdx].gpioPinIntrNum = (uint32_t) RCSS_MIBSPIA_HOST_IRQ_INTR_HIGH;
        }
        else
        {
            /* Slave#1 AWR2243 */
            openCfg.frontEndCfg[u32DevIdx].spiHandle = gMibspiHandle[CONFIG_MIBSPI1];
            openCfg.frontEndCfg[u32DevIdx].gpioBaseAddr = (uint32_t) AddrTranslateP_getLocalAddr(NRESET_FE2_BASE_ADDR);

            /* Inform mmwave which GPIO pin is used for the front end NRESET*/
            openCfg.frontEndCfg[u32DevIdx].nresetGpioIndex = (uint32_t) NRESET_FE2_PIN;

            /* Inform mmwave which GPIO pin is used for the SPI IRQ*/
            openCfg.frontEndCfg[u32DevIdx].spiIrqGpioIndex = (uint32_t) RCSS_MIBSPIB_HOST_IRQ_PIN;

            /* Inform mmwave which interrupt to be configured for SPI IRQ*/
            openCfg.frontEndCfg[u32DevIdx].gpioPinIntrNum = (uint32_t) RCSS_MIBSPIB_HOST_IRQ_INTR_HIGH;
        }
    }

    openCfg.iqSwapSel = 0;
    openCfg.chInterleave = 1;

    /************************************************************************
     * Open the mmWave:
     ************************************************************************/
    if (MMWave_open (gMmwCascadeMCB.mmWaveHandle, &openCfg, NULL, &errCode) < 0)
    {
        /* Error: Unable to configure the mmWave control module */
        test_print ("Error: mmWave open failed [Error code %d]\n", errCode);
        MCPI_setFeatureTestResult ("MMWave MSS Open", MCPI_TestResult_FAIL);
        return;
    }
   
    test_print ("MMWave MSS Open done.\n");

    /************************************************************************
     * Configure the mmWave:
     ************************************************************************/
    if (MMWave_config (gMmwCascadeMCB.mmWaveHandle, &ctrlCfg, &errCode) < 0)
    {
        /* Error: Unable to configure the mmWave control module */
        test_print ("Error: mmWave configuration failed [Error code %d]\n", errCode);
        MCPI_setFeatureTestResult ("MMWave MSS Configuration", MCPI_TestResult_FAIL);
        return;
    }
    
    test_print ("MMWave MSS Configuration done\n");

    /* Populate the calibration configuration: */
    memset ((void *)&calibrationCfg, 0, sizeof(MMWave_CalibrationCfg));
    Mmwave_populateDefaultCalibrationCfg (&calibrationCfg, MMWave_DFEDataOutputMode_FRAME);

    /************************************************************************
     * Start the mmWave:
     ************************************************************************/
    if (MMWave_start (gMmwCascadeMCB.mmWaveHandle, &calibrationCfg, &errCode) < 0)
    {
        /* Error: Unable to configure the mmWave control module */
        test_print ("Error: mmWave start failed [Error code %d]\n", errCode);
        MCPI_setFeatureTestResult ("MMWave MSS Start", MCPI_TestResult_FAIL);
        return;
    }

    DebugP_log  ("MMWave MSS Start done\n");

    /* Wait till the configured number of frames are received. */
    while(gCSIRXState[0].callbackCount.combinedEOF != TEST_NUM_FRAMES)
    {
        ClockP_usleep(1000);
    }
    /************************************************************************
     * Stop the mmWave:
     ************************************************************************/
    retVal = MMWave_stop (gMmwCascadeMCB.mmWaveHandle, &errCode);
    if (retVal < 0)
    {
        /* Error: Stopping the sensor failed. Decode the error code. */
        MMWave_decodeError (errCode, &errorLevel, &mmWaveErrorCode, &subsysErrorCode);

        /* Debug Message: */
        test_print ("Error Level: %s mmWave: %d Subsys: %d\n",
                       (errorLevel == MMWave_ErrorLevel_ERROR) ? "Error" : "Warning",
                       mmWaveErrorCode, subsysErrorCode);

        /* Did we fail because of an error? */
        if (errorLevel == MMWave_ErrorLevel_ERROR)
        {
            /* Error Level: The test has failed. */
            MCPI_setFeatureTestResult ("MMWave MSS Stop", MCPI_TestResult_FAIL);
            return;
        }
        else
        {
            /* Informational Level: The test has passed. Fall through...*/
        }
    }
    test_print ("MMWave MSS Stop done.\n");

    /************************************************************************
     * Close the mmWave:
     ************************************************************************/
    if (MMWave_close (gMmwCascadeMCB.mmWaveHandle, &errCode) < 0)
    {
        /* Error: Unable to configure the mmWave control module */
        test_print ("Error: mmWave close failed [Error code %d]\n", errCode);
        MCPI_setFeatureTestResult ("MMWave MSS Close", MCPI_TestResult_FAIL);
        return;
    }
    MCPI_setFeatureTestResult ("MMWave MSS Close", MCPI_TestResult_PASS);
    test_print ("MMWave MSS close done.\n");
    
    /************************************************************************
     * Deinitialize the mmWave module:
     ************************************************************************/
    if (MMWave_deinit(gMmwCascadeMCB.mmWaveHandle, &errCode) < 0)
    {
        /* Error: Unable to deinitialize the mmWave control module */
        test_print ("Error: mmWave Deinitialization failed [Error code %d]\n", errCode);
        MCPI_setFeatureTestResult ("MMWave MSS Deinitialized", MCPI_TestResult_FAIL);
        return;
    }
    MCPI_setFeatureTestResult ("MMWave MSS Deinitialized", MCPI_TestResult_PASS);

    return;
}

static void MmwCascade_CsiConfigTask(void* args)
{
    MmwCascade_CSIConfig(&gMmwCascadeMCB);

    vTaskDelete(NULL);
}

/**
 *  @b Description
 *  @n
 *      System Initialization Task which initializes the various
 *      components in the system.
 *
 *  @retval
 *      Not Applicable.
 */
static void MmwCascade_initTask(void* args)
{
    int32_t   status = SystemP_SUCCESS;
    int32_t   errCode = 0;

    Drivers_open();
    Board_driversOpen();

    /* Debug Message: */
    test_print ("*********************************************\n");
    test_print ("Debug: Launching mmwave Cascade Application. \n");
    test_print ("*********************************************\n");

    /* Configure HSI interface Clock 
     * Clock Source selected: PLL_PER_CLK - 1728MHz
     */
    HW_WR_REG32(CSL_MSS_TOPRCM_U_BASE + CSL_MSS_TOPRCM_HSI_CLK_SRC_SEL, 0x333);

    /* Configure CSIRX interface Clock */
    HW_WR_REG32(CSL_MSS_TOPRCM_U_BASE + CSL_MSS_TOPRCM_CSIRX_CLK_SRC_SEL, 0x222);

    /* Initialize the result buffer: */
    memset ((void *)&gMmwCascadeMCB, 0, sizeof(MmwCascade_MCB));

    /* Populate edma handle. */
    gMmwCascadeMCB.lvdsStreamcfg.edmaHandle = gEdmaHandle[CONFIG_EDMA2];

    /* Populate PMIC SPI handle. */
    gMmwCascadeMCB.pmicMIBSPIhandle = gMibspiHandle[CONFIG_MIBSPI2];

    /* Initialize LVDS streaming components */
    if ((status = Cascade_LVDSStreamInit()) < 0 )
    {
        test_print ("Error: MMWCascade LVDS stream init failed with Error[%d]\n",status);
    }

    status = SemaphoreP_constructBinary(&gMmwCascadeMCB.CSI2RXConfigCompleteSemHandle, 0);
    DebugP_assert(SystemP_SUCCESS == status);

    /*The delay below is needed only if the DCA1000EVM is being used to capture the data traces.
      This is needed because the DCA1000EVM FPGA needs the delay to lock to the
      bit clock before they can start capturing the data correctly. */
    ClockP_usleep(HSI_DCA_MIN_DELAY_MSEC);

    /* Initialize CSIRX interface. */
    MmwCascade_csirxInit(&gMmwCascadeMCB);

    /* Open CSIRX-A handle. */
    MmwCascade_csirxOpen(&gMmwCascadeMCB, &errCode);

    /* Launch the CSIRX Task */
    gCsiConfigTask = xTaskCreateStatic( MmwCascade_CsiConfigTask,   /* Pointer to the function that implements the task. */
                                "test_csi_config_task", /* Text name for the task.  This is to facilitate debugging only. */
                                CSI_CONFIG_TASK_STACK_SIZE,  /* Stack depth in units of StackType_t typically uint32_t on 32b CPUs */
                                NULL,              /* We are not using the task parameter. */
                                CSI_CONFIG_TASK_PRI,      /* task priority, 0 is lowest priority, configMAX_PRIORITIES-1 is highest */
                                gCsiRxCfgTskStack,  /* pointer to stack base */
                                &gCsiConfigTaskObj );    /* pointer to statically allocated task object memory */
    configASSERT(gCsiConfigTask != NULL);

    /* Wait till CSI RX configuration is complete. */
    status = SemaphoreP_pend(&gMmwCascadeMCB.CSI2RXConfigCompleteSemHandle, SystemP_WAIT_FOREVER);
    DebugP_assert(SystemP_SUCCESS == status);

    Enable_BUCK4_ViaPMIC();

    /* Configure SW session for this LVDS Stream */
    if (Cascade_LVDSStreamSwConfig((uint32_t) AddrTranslateP_getLocalAddr((uint32_t) &CSIA_PingBuf),
                                   (uint32_t) AddrTranslateP_getLocalAddr((uint32_t) &CSIB_PingBuf),
                                   (uint32_t) (TEST_NUM_ADC_SAMPLES * TEST_BYTES_PER_ADC_SAMPLE * 
                                               TEST_NUM_RX)) < 0)
    {
        test_print("Failed LVDS stream SW configuration\n");
        DebugP_assert(0);
    }

    /* Configure the Front-End. */
    MmwCascade_mmWaveTest();

    /* Close CSIRX */
    MmwCascade_csirxClose(&gMmwCascadeMCB);

    test_print ("--- Test Completed ---\n");

    Board_driversClose();
    Drivers_close();

    vTaskDelete(NULL);

    return;
}

/**
 *  @b Description
 *  @n
 *      Configures CBUFF EDMA channel SRC address for Ping/Pong 
 *      Switch
 *
 *  @retval
 *      Not Applicable.
 */
void configureTransfer(void)
{
    static Bool pingPongSwitchFlag = true;

    if(pingPongSwitchFlag)
    {
        /* Update PaRAM set Source address for capturing CSIRX data received on PONG buffer. */
        EDMA_dmaSetPaRAMEntry(CONFIG_EDMA2_BASE_ADDR, CASCADE_LVDS_STREAM_CBUFF_EDMA_CH_0, EDMACC_PARAM_ENTRY_SRC, (uint32_t) SOC_virtToPhy((void *)&CSIA_PongBuf));
        EDMA_dmaSetPaRAMEntry(CONFIG_EDMA2_BASE_ADDR, CASCADE_LVDS_STREAM_CBUFF_EDMA_SHADOW_CH_0, EDMACC_PARAM_ENTRY_SRC, (uint32_t) SOC_virtToPhy((void *)&CSIA_PongBuf));

        EDMA_dmaSetPaRAMEntry(CONFIG_EDMA2_BASE_ADDR, CASCADE_LVDS_STREAM_SW_SESSION_EDMA_CH_0, EDMACC_PARAM_ENTRY_SRC, (uint32_t) SOC_virtToPhy((void *)&CSIB_PongBuf));
        EDMA_dmaSetPaRAMEntry(CONFIG_EDMA2_BASE_ADDR, CASCADE_LVDS_STREAM_SW_SESSION_EDMA_SHADOW_CH_0, EDMACC_PARAM_ENTRY_SRC, (uint32_t) SOC_virtToPhy((void *)&CSIB_PongBuf));

        pingPongSwitchFlag = false;
    }
    else
    {
        /* Update PaRAM set Source address for capturing CSIRX data received on PING buffer. */
        EDMA_dmaSetPaRAMEntry(CONFIG_EDMA2_BASE_ADDR, CASCADE_LVDS_STREAM_CBUFF_EDMA_CH_0, EDMACC_PARAM_ENTRY_SRC, (uint32_t) SOC_virtToPhy((void *)&CSIA_PingBuf));
        EDMA_dmaSetPaRAMEntry(CONFIG_EDMA2_BASE_ADDR, CASCADE_LVDS_STREAM_CBUFF_EDMA_SHADOW_CH_0, EDMACC_PARAM_ENTRY_SRC, (uint32_t) SOC_virtToPhy((void *)&CSIA_PingBuf));

        EDMA_dmaSetPaRAMEntry(CONFIG_EDMA2_BASE_ADDR, CASCADE_LVDS_STREAM_SW_SESSION_EDMA_CH_0, EDMACC_PARAM_ENTRY_SRC, (uint32_t) SOC_virtToPhy((void *)&CSIB_PingBuf));
        EDMA_dmaSetPaRAMEntry(CONFIG_EDMA2_BASE_ADDR, CASCADE_LVDS_STREAM_SW_SESSION_EDMA_SHADOW_CH_0, EDMACC_PARAM_ENTRY_SRC, (uint32_t) SOC_virtToPhy((void *)&CSIB_PingBuf));

        pingPongSwitchFlag = true;
    }

    return;
}

/**
 *  @b Description
 *  @n
 *      Entry point into the mmWave Unit Test
 *
 *  @retval
 *      Not Applicable.
 */
int32_t main (void)
{
    /* init SOC specific modules */
    System_init();
    Board_init();

    /* This task is created at highest priority, it should create more tasks and then delete itself */
    gAppTask = xTaskCreateStatic( MmwCascade_initTask,   /* Pointer to the function that implements the task. */
                                  "test_task_main", /* Text name for the task.  This is to facilitate debugging only. */
                                  APP_TASK_STACK_SIZE,  /* Stack depth in units of StackType_t typically uint32_t on 32b CPUs */
                                  NULL,              /* We are not using the task parameter. */
                                  APP_TASK_PRI,      /* task priority, 0 is lowest priority, configMAX_PRIORITIES-1 is highest */
                                  gAppTskStackMain,  /* pointer to stack base */
                                  &gAppTaskObj );    /* pointer to statically allocated task object memory */
    configASSERT(gAppTask != NULL);

    /* Start the scheduler to start the tasks executing. */
    vTaskStartScheduler();

    /* The following line should never be reached because vTaskStartScheduler()
    will only return if there was not enough FreeRTOS heap memory available to
    create the Idle and (if configured) Timer tasks.  Heap management, and
    techniques for trapping heap exhaustion, are described in the book text. */
    DebugP_assertNoLog(0);
}

