/****************************************************************************
 * nuttx/apps/init/nano_init.h
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Shared between the init task (nano_init.c) and the Telnet shell
 * (nano_telnet.c).
 ****************************************************************************/

#ifndef __NUTTX_APPS_INIT_NANO_INIT_H
#define __NUTTX_APPS_INIT_NANO_INIT_H

#include <nuttx/config.h>

/* An nsh session on the fd 0/1/2 the task inherits, without the login
 * greeting.  The task entry of both the USB and the Telnet sessions.
 */

int nano_nsh_session(int argc, FAR char *argv[]);

#ifdef CONFIG_NANO_INIT_TELNET
/* Opens the Telnet listening socket and starts the task that accepts
 * clients.  Returns once the socket is set up or has failed (the Zephyr
 * Telnet backend does this in a SYS_INIT, before main()).
 */

void nano_telnet_start(void);
#endif

#endif /* __NUTTX_APPS_INIT_NANO_INIT_H */
