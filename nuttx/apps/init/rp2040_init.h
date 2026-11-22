/****************************************************************************
 * nuttx/apps/init/rp2040_init.h
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Shared between the init task (rp2040_init.c) and the Telnet shell
 * (rp2040_telnet.c).
 ****************************************************************************/

#ifndef __NUTTX_APPS_INIT_RP2040_INIT_H
#define __NUTTX_APPS_INIT_RP2040_INIT_H

#include <nuttx/config.h>

/* An nsh session on the fd 0/1/2 the task inherits, without the login
 * greeting.  The task entry of both the USB and the Telnet sessions.
 */

int rp2040_nsh_session(int argc, FAR char *argv[]);

#ifdef CONFIG_INIT_TELNET
/* Opens the Telnet listening socket and starts the task that accepts
 * clients.  Returns once the socket is set up or has failed (the Zephyr
 * Telnet backend does this in a SYS_INIT, before main()).
 */

void rp2040_telnet_start(void);
#endif

#endif /* __NUTTX_APPS_INIT_RP2040_INIT_H */
