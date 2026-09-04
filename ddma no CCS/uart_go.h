/**
 *   @file  uart_go.h
 *
 *   @brief
 *      Minimal polled UART (MSS_SCIA) for host go/stop control and status
 *      prints. Register-level, no SysConfig/driver-framework dependency.
 *      MSS_SCIA is routed to the XDS110 "Application/User UART" on the
 *      AWR2243-2X-CAS-EVM (COM port on Windows, /dev/ttyACM0 on Linux).
 *      115200 baud, 8N1.
 */
#ifndef UART_GO_H
#define UART_GO_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/* One-time init: pinmux (pads DA/DB, mode 5), peripheral clock, SCI setup. */
extern void UartGo_init(void);

/* Blocking write of a NUL-terminated string ('\n' expands to CRLF). */
extern void UartGo_puts(const char *s);

/* printf-style convenience wrapper (256-byte line buffer, not reentrant). */
extern void UartGo_printf(const char *fmt, ...);

/* Non-blocking read: returns next received byte (0..255) or -1 if none. */
extern int32_t UartGo_getcNonBlock(void);

#ifdef __cplusplus
}
#endif

#endif /* UART_GO_H */
