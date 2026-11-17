/* SPDX-License-Identifier: Apache-2.0 */
/* Name resolution through the NINA-W102's own resolver
 * (nuttx/drivers/nina/nina_dns.c), the Zephyr build's
 * drivers/nina/nina_dns.c.  There it is registered as the socket offload's
 * getaddrinfo(), so zsock_getaddrinfo() resolves through the module; here
 * it is a function of its own (NuttX's getaddrinfo() comes with
 * CONFIG_LIBC_NETDB, which this build leaves out), and the Zephyr
 * compatibility layer's <zephyr/net/socket.h> maps zsock_getaddrinfo() and
 * zsock_freeaddrinfo() to it.
 *
 * nina_getaddrinfo() returns 0 or a getaddrinfo() error (EAI_*, NuttX's
 * values, the names of the Zephyr build's DNS_EAI_*).  It gives one IPv4
 * address.  Release the result with nina_freeaddrinfo().
 */

#ifndef __NUTTX_DRIVERS_INCLUDE_NINA_DNS_H
#define __NUTTX_DRIVERS_INCLUDE_NINA_DNS_H

#include <netdb.h>

#ifdef __cplusplus
extern "C"
{
#endif

int nina_getaddrinfo(const char *host, const char *service,
                     const struct addrinfo *hints, struct addrinfo **res);
void nina_freeaddrinfo(struct addrinfo *res);

#ifdef __cplusplus
}
#endif

#endif /* __NUTTX_DRIVERS_INCLUDE_NINA_DNS_H */
