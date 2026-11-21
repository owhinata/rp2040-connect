/* SPDX-License-Identifier: MIT */
/* <netdb.h> for the shared NINA core on Zephyr (see sys/socket.h here):
 * struct addrinfo is Zephyr's struct zsock_addrinfo and the EAI_* errors
 * are its DNS_EAI_* (the names of Zephyr's include/zephyr/posix/netdb.h),
 * so nina_dns_getaddrinfo() and nina_dns_freeaddrinfo() (nina_dns.c) have the
 * types of the socket offload's resolver (struct socket_dns_offload).
 */

#ifndef NINA_POSIX_NETDB_H_
#define NINA_POSIX_NETDB_H_

#include <sys/socket.h>

#include <zephyr/net/dns_resolve.h>

#define addrinfo zsock_addrinfo

#define EAI_BADFLAGS DNS_EAI_BADFLAGS
#define EAI_NONAME   DNS_EAI_NONAME
#define EAI_AGAIN    DNS_EAI_AGAIN
#define EAI_FAIL     DNS_EAI_FAIL
#define EAI_MEMORY   DNS_EAI_MEMORY
#define EAI_SERVICE  DNS_EAI_SERVICE
#define EAI_SOCKTYPE DNS_EAI_SOCKTYPE
#define EAI_FAMILY   DNS_EAI_FAMILY

#endif /* NINA_POSIX_NETDB_H_ */
