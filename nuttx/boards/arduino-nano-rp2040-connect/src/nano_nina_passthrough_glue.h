/* SPDX-License-Identifier: MIT */
/* The NuttX side of the NINA passthrough mode (nano_nina_passthrough.c)
 * between the shared decisions (nina_passthrough_logic.h of
 * common/drivers/nina) and the hardware, without the hardware, so that the
 * host test (nuttx/tests/nina_passthrough.c) runs it as the board does:
 *
 * - The host's lines and rate as cdcacm keeps them (CAIOC_GETCTRLLINE:
 *   DTR is bit 0, RTS bit 1, CDCACM_UART_DTR and CDCACM_UART_RTS of
 *   <nuttx/usb/cdcacm.h>; CAIOC_GETLINECODING: struct cdc_linecoding_s,
 *   whose dwDTERate is 4 bytes little endian), turned into the module's
 *   lines, the way out and the count of closes (nano_pt_host_*()), in the
 *   order of FreeRTOS's tud_cdc_line_state_cb(): the way out is decided
 *   on the lines before the change.
 * - One word of UART1's data register (UARTDR) from the receive interrupt
 *   (nano_pt_uart_word()): a break is no data and is only counted, an
 *   overrun and a framing error are counted and the byte is kept, as in
 *   the FreeRTOS build's uart_irq().
 */

#ifndef __BOARDS_ARDUINO_NANO_RP2040_CONNECT_SRC_NANO_NINA_PASSTHROUGH_GLUE_H
#define __BOARDS_ARDUINO_NANO_RP2040_CONNECT_SRC_NANO_NINA_PASSTHROUGH_GLUE_H

#include <stdbool.h>
#include <stdint.h>

#include "nina_passthrough_logic.h"

/* cdcacm's control line bits (CDC_DTE_PRESENT, CDC_ACTIVATE_CARRIER). */

#define NANO_PT_CTRL_DTR      (1 << 0)
#define NANO_PT_CTRL_RTS      (1 << 1)

/* UARTDR's error bits (RP2040_UART_UARTDR_* of hardware/rp2040_uart01.h). */

#define NANO_PT_UARTDR_OE     (1u << 11)
#define NANO_PT_UARTDR_BE     (1u << 10)
#define NANO_PT_UARTDR_FE     (1u << 8)

/* The host side: its lines as last seen, the rate of its line coding. */

struct nano_pt_host
{
  struct nina_pt_lines lines;
  uint32_t rate;              /* the line coding's dwDTERate */
  uint32_t closes;            /* DTR set -> clear: the port closed */
  bool exit;                  /* the port closed at NINA_PT_EXIT_BAUD */
};

struct nano_pt_uart_counts
{
  uint32_t overruns;
  uint32_t breaks;
  uint32_t framing;
};

/* dwDTERate of a struct cdc_linecoding_s (its first 4 bytes). */

static inline uint32_t nano_pt_coding_rate(const uint8_t baud[4])
{
  return (uint32_t)baud[0] | (uint32_t)baud[1] << 8 |
         (uint32_t)baud[2] << 16 | (uint32_t)baud[3] << 24;
}

/* The port closed (DTR and RTS clear), at cdcacm's initial 115200 bps. */

static inline void nano_pt_host_init(struct nano_pt_host *h)
{
  nina_pt_lines_init(&h->lines);
  h->rate   = NINA_PT_BAUD_INIT;
  h->closes = 0;
  h->exit   = false;
}

/* A new line coding with the rate RATE.  Returns the UART's new rate for
 * its CURRENT one (nina_pt_uart_baud()).
 */

static inline uint32_t nano_pt_host_coding(struct nano_pt_host *h,
                                           uint32_t rate, uint32_t current)
{
  h->rate = rate;
  return nina_pt_uart_baud(rate, current);
}

/* New control lines CTRLLINE at NOW_MS.  Returns true when this change
 * closed the port (DTR set -> clear); h->exit is set when it closed it at
 * the exit rate.  The caller then applies nina_pt_lines_pins().
 */

static inline bool nano_pt_host_ctrlline(struct nano_pt_host *h,
                                         int ctrlline, uint32_t now_ms)
{
  bool dtr = (ctrlline & NANO_PT_CTRL_DTR) != 0;
  bool rts = (ctrlline & NANO_PT_CTRL_RTS) != 0;
  bool closed = h->lines.dtr && !dtr;

  if (nina_pt_exit(h->lines.dtr, dtr, h->rate))
    {
      h->exit = true;
    }

  if (closed)
    {
      h->closes++;
    }

  nina_pt_lines_set(&h->lines, dtr, rts, now_ms);
  return closed;
}

/* One word DR read from UARTDR. */

static inline void nano_pt_uart_word(struct nina_pt_ring *r,
                                     struct nano_pt_uart_counts *c,
                                     uint32_t dr)
{
  if ((dr & NANO_PT_UARTDR_OE) != 0)
    {
      c->overruns++;
    }

  if ((dr & NANO_PT_UARTDR_BE) != 0)
    {
      c->breaks++;
      return;
    }

  if ((dr & NANO_PT_UARTDR_FE) != 0)
    {
      c->framing++;
    }

  nina_pt_ring_put(r, (uint8_t)dr);
}

#endif /* __BOARDS_ARDUINO_NANO_RP2040_CONNECT_SRC_NANO_NINA_PASSTHROUGH_GLUE_H */
