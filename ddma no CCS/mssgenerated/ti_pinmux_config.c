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
#include <drivers/pinmux.h>

static Pinmux_PerCfg_t gPinMuxMainDomainCfg[] = {
            /* RCSS_GPIO0 pin config */
    /* RCSS_GPIO50 -> PAD_DW (B15) */
    {
        PIN_PAD_DW,
        ( PIN_MODE(12) | PIN_PULL_DISABLE )
    },
            /* RCSS_GPIO0 pin config */
    /* RCSS_GPIO34 -> PAD_CS (U18) */
    {
        PIN_PAD_CS,
        ( PIN_MODE(7) | PIN_PULL_DISABLE )
    },
            /* RCSS_GPIO0 pin config */
    /* RCSS_GPIO51 -> PAD_DX (A15) */
    {
        PIN_PAD_DX,
        ( PIN_MODE(12) | PIN_PULL_DISABLE )
    },
            /* RCSS_GPIO0 pin config */
    /* RCSS_GPIO35 -> PAD_CX (W18) */
    {
        PIN_PAD_CX,
        ( PIN_MODE(10) | PIN_PULL_DISABLE )
    },

            /* RCSS_MIBSPIA pin config */
    /* RCSS_MIBSPIA_CLK -> PAD_CQ (T18) */
    {
        PIN_PAD_CQ,
        ( PIN_MODE(1) | PIN_PULL_DIRECTION )
    },
    /* RCSS_MIBSPIA_MOSI -> PAD_CO (R18) */
    {
        PIN_PAD_CO,
        ( PIN_MODE(1) | PIN_PULL_DIRECTION )
    },
    /* RCSS_MIBSPIA_MISO -> PAD_CP (R17) */
    {
        PIN_PAD_CP,
        ( PIN_MODE(1) | PIN_PULL_DIRECTION )
    },
            /* RCSS_MIBSPIB pin config */
    /* RCSS_MIBSPIB_CLK -> PAD_CV (V19) */
    {
        PIN_PAD_CV,
        ( PIN_MODE(1) | PIN_PULL_DIRECTION )
    },
    /* RCSS_MIBSPIB_MOSI -> PAD_CT (U19) */
    {
        PIN_PAD_CT,
        ( PIN_MODE(1) | PIN_PULL_DIRECTION )
    },
    /* RCSS_MIBSPIB_MISO -> PAD_CU (V18) */
    {
        PIN_PAD_CU,
        ( PIN_MODE(1) | PIN_PULL_DIRECTION )
    },
            /* MSS_MIBSPIB pin config */
    /* MSS_MIBSPIB_CLK -> PAD_AJ (D18) */
    {
        PIN_PAD_AJ,
        ( PIN_MODE(1) | PIN_PULL_DIRECTION )
    },
    /* MSS_MIBSPIB_MOSI -> PAD_AH (C18) */
    {
        PIN_PAD_AH,
        ( PIN_MODE(1) | PIN_PULL_DIRECTION )
    },
    /* MSS_MIBSPIB_MISO -> PAD_AI (C19) */
    {
        PIN_PAD_AI,
        ( PIN_MODE(1) | PIN_PULL_DIRECTION )
    },

            /* RCSS_MIBSPIA_CS0 pin config */
    /* RCSS_MIBSPIA_CS0 -> PAD_CR (T19) */
    {
        PIN_PAD_CR,
        ( PIN_MODE(1) | PIN_PULL_DISABLE )
    },
            /* RCSS_MIBSPIB_CS0 pin config */
    /* RCSS_MIBSPIB_CS0 -> PAD_CW (U17) */
    {
        PIN_PAD_CW,
        ( PIN_MODE(1) | PIN_PULL_DISABLE )
    },
            /* MSS_MIBSPIB_CS0 pin config */
    /* MSS_MIBSPIB_CS0 -> PAD_AK (D19) */
    {
        PIN_PAD_AK,
        ( PIN_MODE(1) | PIN_PULL_DISABLE )
    },

    {PINMUX_END, PINMUX_END}
};


/*
 * Pinmux
 */
void Pinmux_init(void)
{
    Pinmux_config(gPinMuxMainDomainCfg, PINMUX_DOMAIN_ID_MAIN);
}

