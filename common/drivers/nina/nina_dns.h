/* SPDX-License-Identifier: Apache-2.0 */
/* Name resolution through the NINA-W102's own resolver (nina_dns.c).  The
 * Zephyr build registers it as the socket offload's getaddrinfo()
 * (NINA_OS_DNS_OFFLOAD, nina_os.h), so zsock_getaddrinfo() resolves through
 * the module.  On NuttX and FreeRTOS it is a function of its own (NuttX's
 * getaddrinfo() comes with CONFIG_LIBC_NETDB, which that build leaves out),
 * and the Zephyr compatibility layer's <zephyr/net/socket.h> maps
 * zsock_getaddrinfo() and zsock_freeaddrinfo() to it.
 *
 * nina_dns_getaddrinfo() returns 0 or a getaddrinfo() error (EAI_*: the OS's
 * values; on Zephyr its DNS_EAI_*).  It gives one IPv4 address.  Release
 * the result with nina_dns_freeaddrinfo().
 */

#ifndef NINA_DNS_H_
#define NINA_DNS_H_

#include <netdb.h>

#ifdef __cplusplus
extern "C"
{
#endif

int nina_dns_getaddrinfo(const char *host, const char *service,
                     const struct addrinfo *hints, struct addrinfo **res);
void nina_dns_freeaddrinfo(struct addrinfo *res);

#ifdef __cplusplus
}
#endif

#endif /* NINA_DNS_H_ */
