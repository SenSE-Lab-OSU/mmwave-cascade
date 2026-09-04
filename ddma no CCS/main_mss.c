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
#include <ti/utils/test/cascade/am273xDDMA/mssgenerated/ti_drivers_config.h>
#include <ti/utils/test/cascade/am273xDDMA/mssgenerated/ti_board_config.h>
#include <ti/utils/test/cascade/am273xDDMA/mssgenerated/ti_drivers_open_close.h>
#include <ti/utils/test/cascade/am273xDDMA/mssgenerated/ti_board_open_close.h>
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
#include <ti/utils/test/cascade/am273xDDMA/cascade_csirx.h>
#include <ti/utils/test/cascade/am273xDDMA/uart_go.h>

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

/* MSS_TOPRCM register offsets used for boot-path-independent HSI/CSIRX clock
 * setup (values from evmam273x.gel, which is what CCS applies on connect). */
#define APP_TOPRCM_LOCK0_KICK0                (0x1008U)
#define APP_TOPRCM_LOCK0_KICK1                (0x100CU)
#define APP_TOPRCM_HSI_DIV_VAL                (0x40U)
#define APP_TOPRCM_CSIRX_DIV_VAL              (0x44U)
#define APP_TOPRCM_HSI_CLK_GATE               (0x80U)
#define APP_TOPRCM_PLL_PER_CLKCTRL            (0x840U)
#define APP_TOPRCM_PLL_PER_TENABLE            (0x844U)
#define APP_TOPRCM_PLL_PER_TENABLEDIV         (0x848U)
#define APP_TOPRCM_PLL_PER_M2NDIV             (0x84CU)
#define APP_TOPRCM_PLL_PER_MN2DIV             (0x850U)
#define APP_TOPRCM_PLL_PER_STATUS             (0x860U)
#define APP_TOPRCM_PLL_PER_HSDIVIDER          (0x864U)
#define APP_TOPRCM_PLL_PER_HSDIVIDER_CLKOUT1  (0x86CU)
#define APP_TOPRCM_PLL_PER_HSDIVIDER_CLKOUT2  (0x870U)

/* DSS_CTRL memory-init registers (offsets from evmam273x.gel). CCS's GEL
 * initialises DSS L2, L3 and mailbox RAM before the app runs; the SBL only
 * initialises memory for cores it loads (R5 only here). The CSI ping/pong
 * buffers, the LVDS header and the system heap live in DSS L3. */
#define APP_DSS_CTRL_U_BASE                   (0x06020000U)
#define APP_DSS_CTRL_LOCK0_KICK0              (0x00001008U)
#define APP_DSS_CTRL_LOCK0_KICK1              (0x0000100CU)
#define APP_DSS_CTRL_DSP_L2RAM_MEMINIT_START  (0x00000080U)
#define APP_DSS_CTRL_DSP_L2RAM_MEMINIT_DONE   (0x00000088U)
#define APP_DSS_CTRL_L3RAM_MEMINIT_START      (0x00000098U)
#define APP_DSS_CTRL_L3RAM_MEMINIT_DONE       (0x000000A0U)
#define APP_DSS_CTRL_MAILBOX_MEMINIT_START    (0x000000B0U)
#define APP_DSS_CTRL_MAILBOX_MEMINIT_DONE     (0x000000B8U)

/* PER PLL configuration as programmed by evmam273x.gel (apll_en_mode1_default):
 * 0x6C0/(0x27+1) * 40 MHz = 1728 MHz, M2 = 1, N2 = 1. */
#define APP_PLL_PER_M2NDIV_GEL                (0x10027U)
#define APP_PLL_PER_MN2DIV_GEL                (0x106C0U)

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
/* ---- DDMA thermal monitor: polled every ~2 s from the frame-wait loop.
   Read gTemp1/gTemp2/gTempStat1/gTempStat2/gTempPolls in CCS Expressions;
   gMaxTxTemp is packed into the top byte of the LVDS header chirpId. ---- */
volatile rlRfTempData_t gTemp1, gTemp2;
volatile int32_t gTempStat1 = -1, gTempStat2 = -1;
volatile uint32_t gTempPolls = 0;
volatile uint8_t gMaxTxTemp = 0;

static void Mmwave_pollTemps(void)
{
    int16_t m;
    gTempStat1 = rlRfGetTemperatureReport(RL_DEVICE_MAP_CASCADED_1,
                                          (rlRfTempData_t *)&gTemp1);
    gTempStat2 = rlRfGetTemperatureReport(RL_DEVICE_MAP_CASCADED_2,
                                          (rlRfTempData_t *)&gTemp2);
    gTempPolls++;

    m = gTemp1.tmpTx0Sens;
    if (gTemp1.tmpTx1Sens > m) m = gTemp1.tmpTx1Sens;
    if (gTemp1.tmpTx2Sens > m) m = gTemp1.tmpTx2Sens;
    if (gTemp2.tmpTx0Sens > m) m = gTemp2.tmpTx0Sens;
    if (gTemp2.tmpTx1Sens > m) m = gTemp2.tmpTx1Sens;
    if (gTemp2.tmpTx2Sens > m) m = gTemp2.tmpTx2Sens;
    if (m > 127)  m = 127;
    if (m < -128) m = -128;
    gMaxTxTemp = (uint8_t)(int8_t)m;
}

/* Results of the DSS memory init done in main() (printed once the UART is up):
 * bit0 = L2 done, bit1 = L3 done, bit2 = mailbox done; poll counts for info. */
static uint32_t gDssMemInitDone   = 0U;
static uint32_t gDssMemInitPolls  = 0U;

/**
 *  @b Description
 *  @n
 *      Replicates the GEL's meminitDSSL2 / memInitDSSL3 / meminitDSSMBOX so
 *      the SBL boot path matches the CCS boot path. Must run before anything
 *      touches DSS L3 (CSI buffers, LVDS header, heap live there), i.e. first
 *      thing in main(). All waits are bounded.
 */
static void MmwCascade_dssMemInit (void)
{
    uint32_t polls;

    HW_WR_REG32(APP_DSS_CTRL_U_BASE + APP_DSS_CTRL_LOCK0_KICK0, 0x01234567U);
    HW_WR_REG32(APP_DSS_CTRL_U_BASE + APP_DSS_CTRL_LOCK0_KICK1, 0x0FEDCBA8U);

    /* DSS L2 */
    HW_WR_REG32(APP_DSS_CTRL_U_BASE + APP_DSS_CTRL_DSP_L2RAM_MEMINIT_START, 0xFFU);
    for (polls = 0U; polls < 2000000U; polls++)
    {
        if (HW_RD_REG32(APP_DSS_CTRL_U_BASE + APP_DSS_CTRL_DSP_L2RAM_MEMINIT_DONE) == 0xFFU) { gDssMemInitDone |= 1U; break; }
    }
    HW_WR_REG32(APP_DSS_CTRL_U_BASE + APP_DSS_CTRL_DSP_L2RAM_MEMINIT_DONE, 0xFFU);
    gDssMemInitPolls += polls;

    /* DSS L3 */
    HW_WR_REG32(APP_DSS_CTRL_U_BASE + APP_DSS_CTRL_L3RAM_MEMINIT_START, 0xFU);
    for (polls = 0U; polls < 2000000U; polls++)
    {
        if (HW_RD_REG32(APP_DSS_CTRL_U_BASE + APP_DSS_CTRL_L3RAM_MEMINIT_DONE) == 0xFU) { gDssMemInitDone |= 2U; break; }
    }
    HW_WR_REG32(APP_DSS_CTRL_U_BASE + APP_DSS_CTRL_L3RAM_MEMINIT_DONE, 0xFU);
    gDssMemInitPolls += polls;

    /* DSS mailbox */
    HW_WR_REG32(APP_DSS_CTRL_U_BASE + APP_DSS_CTRL_MAILBOX_MEMINIT_START, 0x1U);
    for (polls = 0U; polls < 2000000U; polls++)
    {
        if (HW_RD_REG32(APP_DSS_CTRL_U_BASE + APP_DSS_CTRL_MAILBOX_MEMINIT_DONE) == 0x1U) { gDssMemInitDone |= 4U; break; }
    }
    HW_WR_REG32(APP_DSS_CTRL_U_BASE + APP_DSS_CTRL_MAILBOX_MEMINIT_DONE, 0x1U);
    gDssMemInitPolls += polls;
}

/**
 *  @b Description
 *  @n
 *      Print CSI-RX error/event counters for both front ends over the UART.
 */
static void MmwCascade_printCsiCounters (void)
{
    uint32_t d;
    for (d = 0U; d < 2U; d++)
    {
        UartGo_printf ("CSI%u: EOF %u SOF %u payloadOK %u | ocp %u shortPkt %u ecc1 %u eccN %u cio %u fifoOvf %u\n",
            (unsigned int)d,
            (unsigned int)gCSIRXState[d].callbackCount.combinedEOF,
            (unsigned int)gCSIRXState[d].callbackCount.SOF0,
            (unsigned int)gCSIRXState[d].isReceivedPayloadCorrect,
            (unsigned int)gCSIRXState[d].commonIRQcount.isOCPerror,
            (unsigned int)gCSIRXState[d].commonIRQcount.isGenericShortPacketReceive,
            (unsigned int)gCSIRXState[d].commonIRQcount.isECConeBitShortPacketErrorCorrect,
            (unsigned int)gCSIRXState[d].commonIRQcount.isECCmoreThanOneBitCannotCorrect,
            (unsigned int)gCSIRXState[d].commonIRQcount.isComplexIOerror,
            (unsigned int)gCSIRXState[d].commonIRQcount.isFIFOoverflow);
    }
}

/**
 *  @b Description
 *  @n
 *      Print the HSI (LVDS) / CSIRX / PER PLL clock registers over the UART.
 *      Called at init (before/after programming) and on '?' from the host.
 */
static void MmwCascade_printClocks (const char *tag)
{
    UartGo_printf ("%s: HSI src 0x%03x div 0x%03x gate 0x%x | PER M2NDIV 0x%05x MN2DIV 0x%05x HSDIV1 0x%03x HSDIV2 0x%03x | CSIRX src 0x%03x div 0x%03x\n",
        tag,
        (unsigned int)HW_RD_REG32(CSL_MSS_TOPRCM_U_BASE + CSL_MSS_TOPRCM_HSI_CLK_SRC_SEL),
        (unsigned int)HW_RD_REG32(CSL_MSS_TOPRCM_U_BASE + APP_TOPRCM_HSI_DIV_VAL),
        (unsigned int)HW_RD_REG32(CSL_MSS_TOPRCM_U_BASE + APP_TOPRCM_HSI_CLK_GATE),
        (unsigned int)HW_RD_REG32(CSL_MSS_TOPRCM_U_BASE + APP_TOPRCM_PLL_PER_M2NDIV),
        (unsigned int)HW_RD_REG32(CSL_MSS_TOPRCM_U_BASE + APP_TOPRCM_PLL_PER_MN2DIV),
        (unsigned int)HW_RD_REG32(CSL_MSS_TOPRCM_U_BASE + APP_TOPRCM_PLL_PER_HSDIVIDER_CLKOUT1),
        (unsigned int)HW_RD_REG32(CSL_MSS_TOPRCM_U_BASE + APP_TOPRCM_PLL_PER_HSDIVIDER_CLKOUT2),
        (unsigned int)HW_RD_REG32(CSL_MSS_TOPRCM_U_BASE + CSL_MSS_TOPRCM_CSIRX_CLK_SRC_SEL),
        (unsigned int)HW_RD_REG32(CSL_MSS_TOPRCM_U_BASE + APP_TOPRCM_CSIRX_DIV_VAL));
}

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
        UartGo_printf ("ERROR: mmWave open failed [%d]\n", (int)errCode);
        MCPI_setFeatureTestResult ("MMWave MSS Open", MCPI_TestResult_FAIL);
        return;
    }
   
    test_print ("MMWave MSS Open done.\n");
    UartGo_puts ("OPEN OK\n");

    /************************************************************************
     * Configure the mmWave:
     ************************************************************************/
    if (MMWave_config (gMmwCascadeMCB.mmWaveHandle, &ctrlCfg, &errCode) < 0)
    {
        /* Error: Unable to configure the mmWave control module */
        test_print ("Error: mmWave configuration failed [Error code %d]\n", errCode);
        UartGo_printf ("ERROR: mmWave config failed [%d]\n", (int)errCode);
        MCPI_setFeatureTestResult ("MMWave MSS Configuration", MCPI_TestResult_FAIL);
        return;
    }
    
    test_print ("MMWave MSS Configuration done\n");
    UartGo_puts ("CONFIG OK\n");

    /* Populate the calibration configuration: */
    memset ((void *)&calibrationCfg, 0, sizeof(MMWave_CalibrationCfg));
    Mmwave_populateDefaultCalibrationCfg (&calibrationCfg, MMWave_DFEDataOutputMode_FRAME);

    UartGo_puts("INIT OK: front end open+configured, LVDS clock running\n");

    /************************************************************************
     * Host-triggered run loop:
     *   - wait for 'g' on the UART, then MMWave_start (frames run until
     *     stopped; numFrames = 0 in the frame config)
     *   - while running, poll temperatures every ~2 s and watch for 's'
     *   - on 's', MMWave_stop and go back to waiting for 'g'
     * This replaces the fixed TEST_NUM_FRAMES wait of the original test.
     * The host must have the DCA1000 configured and recording BEFORE it
     * sends 'g' (the DCA FPGA needs the LVDS bit clock, already running
     * since init, and needs to be armed before data starts).
     ************************************************************************/
    while (1)
    {
        int32_t ch;

        UartGo_puts("READY: send g to start, s to stop\n");

        /* Wait for 'g' ('?' re-prints status at any time) */
        do
        {
            ClockP_usleep(10000);
            ch = UartGo_getcNonBlock();
            if (ch == (int32_t)'?')
            {
                MmwCascade_printClocks ("STATUS idle");
                MmwCascade_printCsiCounters ();
                UartGo_puts ("READY: send g to start, s to stop\n");
            }
        } while (ch != (int32_t)'g');

        /* Populate the calibration configuration for this run: */
        memset ((void *)&calibrationCfg, 0, sizeof(MMWave_CalibrationCfg));
        Mmwave_populateDefaultCalibrationCfg (&calibrationCfg, MMWave_DFEDataOutputMode_FRAME);

        if (MMWave_start (gMmwCascadeMCB.mmWaveHandle, &calibrationCfg, &errCode) < 0)
        {
            /* Error: Unable to start the mmWave control module */
            MMWave_decodeError (errCode, &errorLevel, &mmWaveErrorCode, &subsysErrorCode);
            test_print ("Error: mmWave start failed [Error code %d]\n", errCode);
            UartGo_printf ("ERROR: mmWave start failed [%d] level %d mmWave %d subsys %d\n",
                           (int)errCode, (int)errorLevel, (int)mmWaveErrorCode, (int)subsysErrorCode);
            MCPI_setFeatureTestResult ("MMWave MSS Start", MCPI_TestResult_FAIL);
            continue;
        }

        DebugP_log  ("MMWave MSS Start done\n");
        UartGo_puts ("START: chirping\n");

        /* Run until 's' arrives; poll temperatures every ~2 s. */
        {
            uint32_t tick = 0;
            uint32_t running = 1U;
            while (running)
            {
                ClockP_usleep(1000);
                if (++tick >= 2000)
                {
                    tick = 0;
                    Mmwave_pollTemps();
                    UartGo_printf ("TEMP: maxTx %d C frames %u\n",
                                   (int)(int8_t)gMaxTxTemp,
                                   (unsigned int)gCSIRXState[0].callbackCount.combinedEOF);
                }
                ch = UartGo_getcNonBlock();
                if (ch == (int32_t)'s')
                {
                    running = 0U;
                }
                else if (ch == (int32_t)'?')
                {
                    MmwCascade_printClocks ("STATUS running");
                    MmwCascade_printCsiCounters ();
                    UartGo_printf ("frames %u maxTx %d C\n",
                                   (unsigned int)gCSIRXState[0].callbackCount.combinedEOF,
                                   (int)(int8_t)gMaxTxTemp);
                }
            }
            Mmwave_pollTemps();
        }

        /********************************************************************
         * Stop the mmWave:
         ********************************************************************/
        retVal = MMWave_stop (gMmwCascadeMCB.mmWaveHandle, &errCode);
        if (retVal < 0)
        {
            /* Error: Stopping the sensor failed. Decode the error code. */
            MMWave_decodeError (errCode, &errorLevel, &mmWaveErrorCode, &subsysErrorCode);

            /* Debug Message: */
            test_print ("Error Level: %s mmWave: %d Subsys: %d\n",
                           (errorLevel == MMWave_ErrorLevel_ERROR) ? "Error" : "Warning",
                           mmWaveErrorCode, subsysErrorCode);
            UartGo_printf ("ERROR: mmWave stop: level %d mmWave %d subsys %d\n",
                           (int)errorLevel, (int)mmWaveErrorCode, (int)subsysErrorCode);

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

        /********************************************************************
         * Cascade: make sure BOTH front ends leave the "frames started"
         * state. The slave is hardware-triggered by the master, so stopping
         * the master alone leaves the slave armed and the next start fails
         * with RL_RET_CODE_FRAME_ALREADY_STARTED (20). Stop the slave first,
         * then the master. RL_RET_CODE_FRAME_ALREADY_ENDED (21) is fine.
         ********************************************************************/
        {
            int32_t rlRet;

            rlRet = rlSensorStop (RL_DEVICE_MAP_CASCADED_2);
            UartGo_printf ("STOP: slave  rlSensorStop -> %d\n", (int)rlRet);

            rlRet = rlSensorStop (RL_DEVICE_MAP_CASCADED_1);
            UartGo_printf ("STOP: master rlSensorStop -> %d\n", (int)rlRet);
        }

        UartGo_puts ("STOP: idle\n");
    }

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

    /* Host control/status UART (MSS_SCIA -> XDS110 user UART). */
    UartGo_init();
    UartGo_puts("\nBOOT: mmwave cascade DDMA (host-triggered build)\n");
    UartGo_printf("DSS meminit: L2 %s L3 %s MBOX %s (%u polls)\n",
                  (gDssMemInitDone & 1U) ? "ok" : "TIMEOUT",
                  (gDssMemInitDone & 2U) ? "ok" : "TIMEOUT",
                  (gDssMemInitDone & 4U) ? "ok" : "TIMEOUT",
                  (unsigned int)gDssMemInitPolls);

    /* Debug Message: */
    test_print ("*********************************************\n");
    test_print ("Debug: Launching mmwave Cascade Application. \n");
    test_print ("*********************************************\n");

    /* Configure HSI (LVDS bit clock) and CSIRX clocks - boot-path independent.
     *
     * Under CCS the EVM GEL (apll_en_mode1_default) runs before this app: it
     * unlocks TOPRCM, programs PER PLL = 1728 MHz, sets HSI_DIV_VAL = /4 and
     * ungates HSI. The original app then only switched the HSI source to
     * PLL_PER_CLK and inherited the rest. Under SBL (flash) boot none of that
     * is guaranteed, and a locked TOPRCM silently drops the source-select
     * write. So: unlock, dump what we found, then program source + divider +
     * gate explicitly to the same values the GEL leaves behind.
     * Register offsets are the ones from evmam273x.gel.
     */
    MmwCascade_printClocks ("CLK before");

    /* Unlock TOPRCM (same kick sequence as the GEL) */
    HW_WR_REG32(CSL_MSS_TOPRCM_U_BASE + APP_TOPRCM_LOCK0_KICK0, 0x01234567U);
    HW_WR_REG32(CSL_MSS_TOPRCM_U_BASE + APP_TOPRCM_LOCK0_KICK1, 0x0FEDCBA8U);

    /* PER PLL: the SBL boot path leaves MN2DIV = 0x006C0 (N2 = 0) whereas the
     * GEL programs 0x106C0 (N2 = 1). N2 divides the pll_per_clk output that
     * feeds HSI, so the LVDS bit clock differs between the two boot paths.
     * If the PLL is not in the GEL configuration, re-latch it with the exact
     * GEL sequence. Safe here: the R5 runs from the CORE PLL, and CSI/LVDS
     * have not been initialised yet. Under CCS this block is skipped. */
    if ((HW_RD_REG32(CSL_MSS_TOPRCM_U_BASE + APP_TOPRCM_PLL_PER_MN2DIV) != APP_PLL_PER_MN2DIV_GEL) ||
        (HW_RD_REG32(CSL_MSS_TOPRCM_U_BASE + APP_TOPRCM_PLL_PER_M2NDIV) != APP_PLL_PER_M2NDIV_GEL))
    {
        uint32_t lockWait;

        HW_WR_REG32(CSL_MSS_TOPRCM_U_BASE + APP_TOPRCM_PLL_PER_M2NDIV,     APP_PLL_PER_M2NDIV_GEL);
        HW_WR_REG32(CSL_MSS_TOPRCM_U_BASE + APP_TOPRCM_PLL_PER_MN2DIV,     APP_PLL_PER_MN2DIV_GEL);
        HW_WR_REG32(CSL_MSS_TOPRCM_U_BASE + APP_TOPRCM_PLL_PER_CLKCTRL,    0x29131000U);
        HW_WR_REG32(CSL_MSS_TOPRCM_U_BASE + APP_TOPRCM_PLL_PER_TENABLE,    0x1U);
        HW_WR_REG32(CSL_MSS_TOPRCM_U_BASE + APP_TOPRCM_PLL_PER_CLKCTRL,    0x29131001U);
        HW_WR_REG32(CSL_MSS_TOPRCM_U_BASE + APP_TOPRCM_PLL_PER_TENABLE,    0x0U);
        HW_WR_REG32(CSL_MSS_TOPRCM_U_BASE + APP_TOPRCM_PLL_PER_TENABLEDIV, 0x1U);
        HW_WR_REG32(CSL_MSS_TOPRCM_U_BASE + APP_TOPRCM_PLL_PER_TENABLEDIV, 0x0U);

        /* Wait for PHASELOCK (bit 10), bounded. */
        for (lockWait = 0U; lockWait < 1000000U; lockWait++)
        {
            if ((HW_RD_REG32(CSL_MSS_TOPRCM_U_BASE + APP_TOPRCM_PLL_PER_STATUS) & 0x400U) != 0U)
            {
                break;
            }
        }

        /* HSDIV outputs as the GEL: CLKOUT1 = 1728/9 = 192 MHz, CLKOUT2 = 1728/18 = 96 MHz */
        HW_WR_REG32(CSL_MSS_TOPRCM_U_BASE + APP_TOPRCM_PLL_PER_HSDIVIDER_CLKOUT1, 0x8U);
        HW_WR_REG32(CSL_MSS_TOPRCM_U_BASE + APP_TOPRCM_PLL_PER_HSDIVIDER_CLKOUT2, 0x11U);
        HW_WR_REG32(CSL_MSS_TOPRCM_U_BASE + APP_TOPRCM_PLL_PER_HSDIVIDER,         0x4U);
        HW_WR_REG32(CSL_MSS_TOPRCM_U_BASE + APP_TOPRCM_PLL_PER_HSDIVIDER,         0x0U);
        HW_WR_REG32(CSL_MSS_TOPRCM_U_BASE + APP_TOPRCM_PLL_PER_HSDIVIDER_CLKOUT1, 0x108U);
        HW_WR_REG32(CSL_MSS_TOPRCM_U_BASE + APP_TOPRCM_PLL_PER_HSDIVIDER_CLKOUT2, 0x111U);

        UartGo_printf ("CLK: PER PLL re-latched to GEL config (N2=1), lock %s after %u polls\n",
                       (lockWait < 1000000U) ? "OK" : "TIMEOUT", (unsigned int)lockWait);
    }

    /* HSI: PLL_PER_CLK (1728 MHz) / 4, ungated  -> matches the CCS/GEL state */
    HW_WR_REG32(CSL_MSS_TOPRCM_U_BASE + APP_TOPRCM_HSI_DIV_VAL, 0x333U);
    HW_WR_REG32(CSL_MSS_TOPRCM_U_BASE + CSL_MSS_TOPRCM_HSI_CLK_SRC_SEL, 0x333U);
    HW_WR_REG32(CSL_MSS_TOPRCM_U_BASE + APP_TOPRCM_HSI_CLK_GATE, 0x0U);

    /* CSIRX: PER HSDIV CLKOUT2 (96 MHz) / 1 -> matches the CCS/GEL state */
    HW_WR_REG32(CSL_MSS_TOPRCM_U_BASE + APP_TOPRCM_CSIRX_DIV_VAL, 0x000U);
    HW_WR_REG32(CSL_MSS_TOPRCM_U_BASE + CSL_MSS_TOPRCM_CSIRX_CLK_SRC_SEL, 0x222U);

    MmwCascade_printClocks ("CLK after ");

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
    /* GEL parity: init DSS L2/L3/mailbox RAM before anything uses DSS L3
     * (CSI ping/pong buffers, LVDS header and the system heap live there). */
    MmwCascade_dssMemInit();

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