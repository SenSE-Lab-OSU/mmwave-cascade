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

#ifndef TI_DRVIERS_OPEN_CLOSE_H_
#define TI_DRVIERS_OPEN_CLOSE_H_

#include <stdint.h>
#include "ti_drivers_config.h"

#ifdef __cplusplus
extern "C" {
#endif

/*
 * Common Functions
 */
void Drivers_open(void);
void Drivers_close(void);

/*
 * CSIRX
 */
#include <drivers/csirx.h>

/* CSIRX Context Macros */
#define CONFIG_CSIRX0_NUM_CONTEXT (1U)
#define CONFIG_CSIRX1_NUM_CONTEXT (1U)

/* CSIRX driver handles */
extern CSIRX_Handle gCsirxHandle[CONFIG_CSIRX_NUM_INSTANCES];

/*
 * CSIRX configuration
 */
extern bool gCsirxPhyEnable[CONFIG_CSIRX_NUM_INSTANCES];
extern CSIRX_DphyConfig gCsirxDphyConfig[CONFIG_CSIRX_NUM_INSTANCES];
extern CSIRX_ComplexioConfig gCsirxComplexioConfig[CONFIG_CSIRX_NUM_INSTANCES];
extern CSIRX_CommonConfig gCsirxCommonConfig[CONFIG_CSIRX_NUM_INSTANCES];

/* CSIRX context configurations */
extern CSIRX_ContextConfig gConfigCsirx0ContextConfig[CONFIG_CSIRX0_NUM_CONTEXT];
extern CSIRX_ContextConfig gConfigCsirx1ContextConfig[CONFIG_CSIRX1_NUM_CONTEXT];

/* CSIRX Driver open/close - can be used by application when Drivers_open() and
 * Drivers_close() is not used directly and app wants to control the various driver
 * open/close sequences */
int32_t Drivers_csirxOpen(void);
int32_t Drivers_csirxClose(void);

/* These APIs run the to open or close seqquence for one instance of CSIRX driver */
int32_t Drivers_csirxInstanceOpen(uint32_t instanceId, uint16_t numContexts, CSIRX_ContextConfig *pContextConfig);
int32_t Drivers_csirxInstanceClose(uint32_t instanceId, uint16_t numContexts);
/*
 * EDMA
 */
#include <drivers/edma.h>

/* EDMA Driver handles */
extern EDMA_Handle gEdmaHandle[CONFIG_EDMA_NUM_INSTANCES];

/*
 * EDMA Driver Advance Parameters - to be used only when Driver_open() and
 * Driver_close() is not used by the application
 */
/* EDMA Driver Open Parameters */
extern EDMA_Params gEdmaParams[CONFIG_EDMA_NUM_INSTANCES];
/* EDMA Driver open/close - can be used by application when Driver_open() and
 * Driver_close() is not used directly and app wants to control the various driver
 * open/close sequences */
void Drivers_edmaOpen(void);
void Drivers_edmaClose(void);

/*
 * GPIO
 */
#include <drivers/gpio.h>

/* GPIO Driver open/close - can be used by application when Driver_open() and
 * is not used directly and app wants to control the various driver
 * open sequences */
void Drivers_gpioOpen(void);


/*
 * MIBSPI
 */
#include <drivers/mibspi.h>

/* MIBSPI Driver handles */
extern MIBSPI_Handle gMibspiHandle[CONFIG_MIBSPI_NUM_INSTANCES];

/*
 * MIBSPI Driver Advance Parameters - to be used only when Driver_open() and
 * Driver_close() is not used by the application
 */
/* MIBSPI Driver default Open Parameters */
extern MIBSPI_OpenParams gMibspiParams;

/* MIBSPI Driver open/close - can be used by application when Driver_open() and
 * Driver_close() is not used directly and app wants to control the various driver
 * open/close sequences */
void Drivers_mibspiOpen(void);
void Drivers_mibspiClose(void);


#ifdef __cplusplus
}
#endif

#endif /* TI_DRVIERS_OPEN_CLOSE_H_ */
