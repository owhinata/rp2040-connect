/* SPDX-License-Identifier: MIT */
/* The host's side of the NINA passthrough mode (nina_passthrough.c): what
 * the core needs from the OS and its USB serial port.  Each OS binding
 * implements it next to its other parts:
 * common/drivers/nina/zephyr/nina_passthrough_os.c (Zephyr's USB device
 * stack, CDC ACM), common/drivers/nina/freertos/nina_passthrough_os.c
 * (TinyUSB and the Pico SDK, without the scheduler) and
 * common/drivers/nina/nuttx/nina_passthrough_os.c (cdcacm's
 * /dev/ttyACM0); the host tests replace it with fakes.  The binding also
 * implements nina_passthrough_run() and calls the core at its events
 * (nina_passthrough.h).
 */

#ifndef NINA_PASSTHROUGH_OS_H_
#define NINA_PASSTHROUGH_OS_H_

#include <stdbool.h>
#include <stdint.h>

#include "nina_passthrough.h"

#ifdef __cplusplus
extern "C" {
#endif

/* A clock in milliseconds (wrapping at 32 bits), from any context. */
uint32_t nina_pt_os_now_ms(void);

/* Waits MS milliseconds, serving the USB port meanwhile (in a thread or
 * the binding's loop: the way out).
 */
void nina_pt_os_sleep_ms(uint32_t ms);

/* Up to LEN bytes at DATA to the host, from the buffer's consumer
 * (nina_passthrough_to_host()).  Returns the number taken, 0 when the host
 * takes none now (the rest waits), or a negative errno when there is no
 * host to take them (USB not configured: the bytes are discarded).
 */
int nina_pt_os_write(const uint8_t *data, uint32_t len);

/* From the UART's interrupt, after its receive FIFO went into the buffer
 * (RECEIVED: some bytes did): the binding's part of the interrupt (waking
 * the consumer, the rest of the host's bytes, ...).
 */
void nina_pt_os_uart_irq(bool received);

/* The log: two lines at the info level of the module "nina_passthrough",
 * each the OS's LOG_INF() with the format and arguments below, so that
 * they read the same in every build.
 */
struct nina_pt_counts {
	uint64_t to_nina;   /* bytes from the host into the UART */
	uint64_t from_nina; /* bytes from the UART to the host */
	uint64_t discarded; /* bytes from the UART while USB was not configured */
	uint32_t lost;      /* bytes that found the buffer full */
	uint32_t peak;      /* most bytes seen waiting in the buffer */
	uint32_t size;      /* the buffer's size */
	uint32_t overruns;  /* the UART's flags of the bytes received */
	uint32_t breaks;
	uint32_t framing;
	uint32_t baud; /* the UART's rate */
};

#define NINA_PT_LOG_START                                                                          \
	"NINA passthrough: USB <-> UART1 (GPIO8 TX, GPIO9 RX) at %lu bps, "                        \
	"DTR -> GPIO0 (GPIO2), RTS -> RESETN (GPIO3); close the port at %u bps or "                \
	"press RESET to leave"
#define NINA_PT_LOG_START_ARGS(baud) (unsigned long)(baud), NINA_PT_EXIT_BAUD

#define NINA_PT_LOG_COUNTS                                                                         \
	"%s: %llu bytes to the NINA, %llu from it, %llu discarded without USB, "                   \
	"%lu lost (buffer full, peak %lu of %lu), UART: %lu overruns, %lu breaks, "                \
	"%lu framing errors, %lu bps"
#define NINA_PT_LOG_COUNTS_ARGS(why, c)                                                            \
	(why), (unsigned long long)(c)->to_nina, (unsigned long long)(c)->from_nina,               \
		(unsigned long long)(c)->discarded, (unsigned long)(c)->lost,                      \
		(unsigned long)(c)->peak, (unsigned long)(c)->size, (unsigned long)(c)->overruns,  \
		(unsigned long)(c)->breaks, (unsigned long)(c)->framing, (unsigned long)(c)->baud

/* The mode's first line, at the UART's rate BAUD. */
void nina_pt_os_log_start(uint32_t baud);

/* A line with the counts C after WHY. */
void nina_pt_os_log_counts(const char *why, const struct nina_pt_counts *c);

#ifdef __cplusplus
}
#endif

#endif /* NINA_PASSTHROUGH_OS_H_ */
