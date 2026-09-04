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

#ifndef TI_DRIVERS_CONFIG_H_
#define TI_DRIVERS_CONFIG_H_

#include <stdint.h>
#include <drivers/hw_include/cslr_soc.h>
#include "ti_dpl_config.h"

#ifdef __cplusplus
extern "C" {
#endif

/*
 * Common Functions
 */
void System_init(void);
void System_deinit(void);

/*
 * CBUFF
 */
#include <drivers/cbuff.h>

/* CBUFF Instance Macros */
#define CONFIG_CBUFF0 (0U)
#define CONFIG_CBUFF_NUM_INSTANCES (1U)


/*
 * CRC
 */
#include <drivers/crc.h>
#include <drivers/soc.h>

/* CRC Instance Macros */
#define CONFIG_CRC0_BASE_ADDR (CSL_MSS_MCRC_U_BASE)
#define CONFIG_CRC0_INTR (24U)
#define CONFIG_CRC_NUM_INSTANCES (1U)


/*
 * CSIRX
 */
#include <drivers/csirx.h>

/* CSIRX Instance Macros */
#define CONFIG_CSIRX0 (0U)
#define CONFIG_CSIRX1 (1U)
#define CONFIG_CSIRX_NUM_INSTANCES (2U)

/*
 * EDMA
 */
#include <drivers/edma.h>
#include <drivers/soc.h>

/* EDMA Instance Macros */
#define CONFIG_EDMA0_BASE_ADDR (CSL_RCSS_TPCC_A_U_BASE)
#define CONFIG_EDMA1_BASE_ADDR (CSL_MSS_TPCC_A_U_BASE)
#define CONFIG_EDMA2_BASE_ADDR (CSL_DSS_TPCC_A_U_BASE)
#define CONFIG_EDMA0 (0U)
#define CONFIG_EDMA1 (1U)
#define CONFIG_EDMA2 (2U)
#define CONFIG_EDMA_NUM_INSTANCES (3U)
/*
 * GPIO
 */
#include <drivers/gpio.h>
#include <drivers/soc.h>

/* GPIO PIN Macros */
#define NRESET_FE1_BASE_ADDR (CSL_RCSS_GIO_U_BASE)
#define NRESET_FE1_PIN (18U)
#define NRESET_FE1_DIR (GPIO_DIRECTION_OUTPUT)
#define NRESET_FE1_TRIG_TYPE (GPIO_TRIG_TYPE_NONE)
#define NRESET_FE1_OPEN_DRAIN (GPIO_OPEN_DRAIN_DISABLE)
#define NRESET_FE1_INTR_LEVEL (GPIO_INTR_LEVEL_LOW)
#define NRESET_FE1_INTR_HIGH (CSL_MSS_INTR_RCSS_GIO_INT0)
#define NRESET_FE1_INTR_LOW (CSL_MSS_INTR_RCSS_GIO_INT1)
#define RCSS_MIBSPIA_HOST_IRQ_BASE_ADDR (CSL_RCSS_GIO_U_BASE)
#define RCSS_MIBSPIA_HOST_IRQ_PIN (2U)
#define RCSS_MIBSPIA_HOST_IRQ_DIR (GPIO_DIRECTION_INPUT)
#define RCSS_MIBSPIA_HOST_IRQ_TRIG_TYPE (GPIO_TRIG_TYPE_NONE)
#define RCSS_MIBSPIA_HOST_IRQ_OPEN_DRAIN (GPIO_OPEN_DRAIN_DISABLE)
#define RCSS_MIBSPIA_HOST_IRQ_INTR_LEVEL (GPIO_INTR_LEVEL_LOW)
#define RCSS_MIBSPIA_HOST_IRQ_INTR_HIGH (CSL_MSS_INTR_RCSS_GIO_INT0)
#define RCSS_MIBSPIA_HOST_IRQ_INTR_LOW (CSL_MSS_INTR_RCSS_GIO_INT1)
#define NRESET_FE2_BASE_ADDR (CSL_RCSS_GIO_U_BASE)
#define NRESET_FE2_PIN (19U)
#define NRESET_FE2_DIR (GPIO_DIRECTION_OUTPUT)
#define NRESET_FE2_TRIG_TYPE (GPIO_TRIG_TYPE_NONE)
#define NRESET_FE2_OPEN_DRAIN (GPIO_OPEN_DRAIN_DISABLE)
#define NRESET_FE2_INTR_LEVEL (GPIO_INTR_LEVEL_LOW)
#define NRESET_FE2_INTR_HIGH (CSL_MSS_INTR_RCSS_GIO_INT0)
#define NRESET_FE2_INTR_LOW (CSL_MSS_INTR_RCSS_GIO_INT1)
#define RCSS_MIBSPIB_HOST_IRQ_BASE_ADDR (CSL_RCSS_GIO_U_BASE)
#define RCSS_MIBSPIB_HOST_IRQ_PIN (3U)
#define RCSS_MIBSPIB_HOST_IRQ_DIR (GPIO_DIRECTION_INPUT)
#define RCSS_MIBSPIB_HOST_IRQ_TRIG_TYPE (GPIO_TRIG_TYPE_NONE)
#define RCSS_MIBSPIB_HOST_IRQ_OPEN_DRAIN (GPIO_OPEN_DRAIN_DISABLE)
#define RCSS_MIBSPIB_HOST_IRQ_INTR_LEVEL (GPIO_INTR_LEVEL_LOW)
#define RCSS_MIBSPIB_HOST_IRQ_INTR_HIGH (CSL_MSS_INTR_RCSS_GIO_INT0)
#define RCSS_MIBSPIB_HOST_IRQ_INTR_LOW (CSL_MSS_INTR_RCSS_GIO_INT1)
#define CONFIG_GPIO_NUM_INSTANCES (4U)

/*
 * MIBSPI
 */
#include <drivers/mibspi.h>

/* MIBSPI Instance Macros */
#define CONFIG_MIBSPI0 (0U)
#define CONFIG_MIBSPI1 (1U)
#define CONFIG_MIBSPI2 (2U)
#define CONFIG_MIBSPI_NUM_INSTANCES (3U)


#include <drivers/soc.h>
#include <kernel/dpl/CycleCounterP.h>


#ifdef __cplusplus
}
#endif

#endif /* TI_DRIVERS_CONFIG_H_ */
