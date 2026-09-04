/**
 *   @file  uart_go.c
 *
 *   @brief
 *      Minimal polled UART on MSS_SCIA for host go/stop control and status
 *      prints. Register-level TI SCI programming; deliberately avoids the
 *      SysConfig-generated driver framework so the existing mssgenerated/
 *      files stay untouched.
 *
 *      Pinmux pads and mode are taken verbatim from the MCU+ SDK
 *      sbl_uart example for AM273x (MSS_UARTA_RX -> PAD_DA mode 5,
 *      MSS_UARTA_TX -> PAD_DB mode 5), i.e. the same UART path the
 *      QSPI flashing procedure already proved out on this board.
 */

#include <stdio.h>
#include <stdarg.h>
#include <stdint.h>

#include <ti/utils/test/cascade/am273xDDMA/mssgenerated/ti_drivers_config.h>
#include <drivers/pinmux.h>
#include <drivers/soc.h>

#include <ti/utils/test/cascade/am273xDDMA/uart_go.h>

/* ---------------- TI SCI register block (standard SCI/SCILIN map) -------- */
typedef volatile struct UartGo_SciRegs_t
{
    uint32_t GCR0;          /* 0x00 global control 0                */
    uint32_t GCR1;          /* 0x04 global control 1                */
    uint32_t GCR2;          /* 0x08                                  */
    uint32_t SETINT;        /* 0x0C                                  */
    uint32_t CLEARINT;      /* 0x10                                  */
    uint32_t SETINTLVL;     /* 0x14                                  */
    uint32_t CLEARINTLVL;   /* 0x18                                  */
    uint32_t FLR;           /* 0x1C flags                            */
    uint32_t INTVECT0;      /* 0x20                                  */
    uint32_t INTVECT1;      /* 0x24                                  */
    uint32_t FORMAT;        /* 0x28 char length                      */
    uint32_t BRS;           /* 0x2C baud rate selector               */
    uint32_t ED;            /* 0x30 emulation data                   */
    uint32_t RD;            /* 0x34 receive data                     */
    uint32_t TD;            /* 0x38 transmit data                    */
    uint32_t PIO0;          /* 0x3C pin function                     */
} UartGo_SciRegs;

#define UARTGO_SCI          ((UartGo_SciRegs *)CSL_MSS_SCIA_U_BASE)

/* GCR1 bits */
#define UARTGO_GCR1_TIMING_MODE   (1U << 1)   /* asynchronous               */
#define UARTGO_GCR1_CLOCK         (1U << 5)   /* internal clock             */
#define UARTGO_GCR1_SWnRST        (1U << 7)   /* release from reset         */
#define UARTGO_GCR1_CONT          (1U << 17)  /* keep running on dbg halt   */
#define UARTGO_GCR1_RXENA         (1U << 24)
#define UARTGO_GCR1_TXENA         (1U << 25)

/* FLR bits */
#define UARTGO_FLR_TXRDY          (1U << 8)
#define UARTGO_FLR_RXRDY          (1U << 9)
#define UARTGO_FLR_PE             (1U << 24)
#define UARTGO_FLR_OE             (1U << 25)
#define UARTGO_FLR_FE             (1U << 26)

/* PIO0 bits: pins operate as SCI functional pins */
#define UARTGO_PIO0_RXFUNC        (1U << 1)
#define UARTGO_PIO0_TXFUNC        (1U << 2)

#define UARTGO_BAUD               (115200U)
#define UARTGO_DEFAULT_CLK_HZ     (200000000U)

/* MSS_UARTA pinmux, copied from the MCU+ SDK sbl_uart am273x example:
 * MSS_UARTA_RX -> PAD_DA (ball U3), MSS_UARTA_TX -> PAD_DB (ball W2). */
static Pinmux_PerCfg_t gUartGoPinMuxCfg[] = {
    {
        PIN_PAD_DA,
        ( PIN_MODE(5) | PIN_PULL_DISABLE )
    },
    {
        PIN_PAD_DB,
        ( PIN_MODE(5) | PIN_PULL_DISABLE )
    },
    {PINMUX_END, PINMUX_END}
};

void UartGo_init(void)
{
    UartGo_SciRegs *sci = UARTGO_SCI;
    uint32_t clkHz;
    uint32_t prescale;

    /* Mux the two UARTA pads (additive; the generated pinmux table for the
     * rest of the app has already run from System_init). */
    Pinmux_config(gUartGoPinMuxCfg, PINMUX_DOMAIN_ID_MAIN);

    /* Make sure the SCIA functional clock is on. Source/frequency choice
     * mirrors the MCU+ SDK generated code for UART0 on this device. */
    (void) SOC_rcmSetPeripheralClock(SOC_RcmPeripheralId_MSS_SCIA,
                                     SOC_RcmPeripheralClockSource_SYS_CLK,
                                     UARTGO_DEFAULT_CLK_HZ);
    clkHz = SOC_rcmGetPeripheralClock(SOC_RcmPeripheralId_MSS_SCIA);
    if ((clkHz == 0U) || (clkHz == 0xFFFFFFFFU))
    {
        clkHz = UARTGO_DEFAULT_CLK_HZ;
    }

    /* Bring the SCI out of module reset, program it while SWnRST is held. */
    sci->GCR0 = 0U;
    sci->GCR0 = 1U;

    sci->GCR1 = UARTGO_GCR1_TIMING_MODE | UARTGO_GCR1_CLOCK | UARTGO_GCR1_CONT;

    sci->FORMAT = 7U;                       /* 8 data bits                  */

    /* baud = clk / (16 * (P + 1)); rounded */
    prescale = ((clkHz + (8U * UARTGO_BAUD)) / (16U * UARTGO_BAUD));
    if (prescale > 0U)
    {
        prescale -= 1U;
    }
    sci->BRS = prescale;

    sci->PIO0 = UARTGO_PIO0_RXFUNC | UARTGO_PIO0_TXFUNC;

    sci->GCR1 |= (UARTGO_GCR1_RXENA | UARTGO_GCR1_TXENA);
    sci->GCR1 |= UARTGO_GCR1_SWnRST;        /* release: UART is live        */
}

static void UartGo_putc(char c)
{
    UartGo_SciRegs *sci = UARTGO_SCI;

    while ((sci->FLR & UARTGO_FLR_TXRDY) == 0U)
    {
        /* spin */
    }
    sci->TD = (uint32_t)(uint8_t)c;
}

void UartGo_puts(const char *s)
{
    while (*s != '\0')
    {
        if (*s == '\n')
        {
            UartGo_putc('\r');
        }
        UartGo_putc(*s);
        s++;
    }
}

void UartGo_printf(const char *fmt, ...)
{
    static char buf[256];
    va_list ap;

    va_start(ap, fmt);
    vsnprintf(buf, sizeof(buf), fmt, ap);
    va_end(ap);

    UartGo_puts(buf);
}

int32_t UartGo_getcNonBlock(void)
{
    UartGo_SciRegs *sci = UARTGO_SCI;
    uint32_t flr = sci->FLR;

    /* Clear any receive error flags so the receiver can't wedge. */
    if ((flr & (UARTGO_FLR_PE | UARTGO_FLR_OE | UARTGO_FLR_FE)) != 0U)
    {
        sci->FLR = (flr & (UARTGO_FLR_PE | UARTGO_FLR_OE | UARTGO_FLR_FE));
    }

    if ((flr & UARTGO_FLR_RXRDY) != 0U)
    {
        return (int32_t)(sci->RD & 0xFFU);
    }
    return -1;
}
