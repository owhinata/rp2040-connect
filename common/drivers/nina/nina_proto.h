/* SPDX-License-Identifier: Apache-2.0 */
#ifndef NINA_PROTO_H_
#define NINA_PROTO_H_
#include <stddef.h>
#include <stdint.h>
#include <stdbool.h>
struct nina_param {
	const uint8_t *data;
	size_t len;
};
int nina_request(uint8_t cmd, const struct nina_param *params, size_t nparams,
		 bool wide_params, bool wide_response, uint8_t *out, size_t cap,
		 size_t *out_len);
int nina_ip_config(uint8_t data[12]);
void nina_dns_init(void);

/* The module's capabilities (command 0x6A of firmware/nina-fw's patches
 * 0003 to 0006; firmware/nina-fw/README.md): bits of what it adds to the
 * Arduino firmware 3.0.2.
 */
#define NINA_CAP_SCAN_KEEPS_LINK (1u << 0) /* a scan while connected keeps the link */
#define NINA_CAP_ASYNC_SCAN      (1u << 1) /* 0x36 starts a scan, 0x6B reports it */
#define NINA_CAP_SOCKET_EVENTS   (1u << 2) /* socket events on the IRQ line */
#define NINA_CAP_KNOWN           0x7u

/* nina_probe_capabilities() asks the module once (nina_net.c iface_init(),
 * after the module's reset), starts the OS's waits for the socket events
 * when the module has them (nina_wait.c), and returns the bits in use.  A
 * firmware without the command (it answers 0xEF: -ENOTSUP of
 * nina_request()) or a failed request gives 0: the driver then works as
 * with 3.0.2.  Later calls return the first answer.
 * nina_capabilities() returns the bits in use: the module's known bits,
 * without NINA_CAP_SOCKET_EVENTS when the OS does not wait for the events
 * (nina_os_event_start() failed); 0 before the probe.
 * nina_module_capabilities() returns the module's own bits (0 before).
 */
uint32_t nina_probe_capabilities(void);
uint32_t nina_capabilities(void);
uint32_t nina_module_capabilities(void);
#endif
