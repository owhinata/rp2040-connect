/* SPDX-License-Identifier: Apache-2.0 */
/* Name resolution with the module's resolver commands (0x34 starts it with
 * the name, 0x35 reads the address), shared by the Zephyr, NuttX and
 * FreeRTOS builds.  It was the Zephyr build's drivers/nina/nina_dns.c, where
 * resolve() and free_result() were the socket offload's resolver; here they
 * are nina_dns_getaddrinfo() and nina_dns_freeaddrinfo() (<nina_dns.h>), with the
 * same branches and results:
 *
 * - The errors are the OS's EAI_* (Zephyr's DNS_EAI_*, NuttX's and
 *   FreeRTOS's EAI_* through <netdb.h>).
 * - The result comes from malloc(), or from the OS's heap with
 *   NINA_OS_DNS_OFFLOAD (nina_os.h; the Zephyr build: its 2048-byte k_heap).
 * - nina_dns_lock is nina_os_dns_lock().
 * - nina_dns_init() registers the resolver with NINA_OS_DNS_OFFLOAD
 *   (nina_os_dns_register()), and does nothing without it.
 *
 * OS-independent apart from nina_os.h and the POSIX names of the sockets
 * (on Zephyr: common/drivers/nina/port/zephyr/posix); the host test
 * (common/tests/nina_dns.c) runs it on a fake nina_proto_request().
 */

#include <errno.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#include <arpa/inet.h>
#include <netdb.h>
#include <netinet/in.h>
#include <sys/socket.h>

#include "nina_dns.h"
#include "nina_os.h"
#include "nina_proto.h"

#ifdef NINA_OS_DNS_OFFLOAD
#  define dns_alloc nina_os_dns_alloc
#  define dns_free  nina_os_dns_free
#else
#  define dns_alloc malloc
#  define dns_free  free
#endif

/* Protect NINA's single shared DNS result across both protocol commands:
 * nina_os_dns_lock(). */

/* An entry and its address, freed together. */
struct nina_addrinfo {
	struct addrinfo ai;
	struct sockaddr_in addr;
};

int nina_dns_getaddrinfo(const char *host, const char *service,
		     const struct addrinfo *hints,
		     struct addrinfo **res)
{
	struct nina_addrinfo *entry;
	struct sockaddr_in *addr;
	unsigned long port = 0;
	uint8_t ip[4], answer;
	size_t n, host_len;
	int ret, family = AF_UNSPEC, type = SOCK_STREAM;
	int protocol = IPPROTO_TCP, flags = 0;

	if (!res || !host || !*host) {
		return EAI_NONAME;
	}
	*res = NULL;
	if (hints) {
		family = hints->ai_family;
		if (hints->ai_socktype) {
			type = hints->ai_socktype;
		} else if (hints->ai_protocol == IPPROTO_UDP) {
			type = SOCK_DGRAM;
		}
		protocol = hints->ai_protocol       ? hints->ai_protocol
			   : type == SOCK_DGRAM ? IPPROTO_UDP
						    : IPPROTO_TCP;
		flags = hints->ai_flags;
	}
	if (flags & ~(AI_NUMERICHOST | AI_NUMERICSERV |
		      AI_ADDRCONFIG)) {
		return EAI_BADFLAGS;
	}
	if (family != AF_UNSPEC && family != AF_INET) {
		return EAI_FAMILY;
	}
	if (!((type == SOCK_STREAM && protocol == IPPROTO_TCP) ||
	      (type == SOCK_DGRAM && protocol == IPPROTO_UDP))) {
		return EAI_SOCKTYPE;
	}
	if (service) {
		char *end;
		if (!*service ||
		    strspn(service, "0123456789") != strlen(service)) {
			return EAI_SERVICE;
		}
		errno = 0;
		port = strtoul(service, &end, 10);
		if (errno || *end || port > 65535) {
			return EAI_SERVICE;
		}
	}

	if (inet_pton(AF_INET, host, ip) != 1) {
		struct nina_proto_param param;

		if (flags & AI_NUMERICHOST) {
			return EAI_NONAME;
		}
		host_len = strlen(host);
		if (host_len > 253) {
			return EAI_NONAME;
		}
		param = (struct nina_proto_param){(const uint8_t *)host, host_len};
		nina_os_dns_lock();
		ret = nina_proto_request(NINA_CMD_REQ_HOST_BY_NAME, &param, 1, false, false, &answer, 1,
				   &n);
		if (!ret && n == 1 && answer == 1) {
			ret = nina_proto_request(NINA_CMD_GET_HOST_BY_NAME, NULL, 0, false, false, ip,
					   sizeof(ip), &n);
		} else if (!ret) {
			ret = -ENOENT;
		}
		nina_os_dns_unlock();
		if (ret) {
			return ret == -ENOENT ? EAI_NONAME : EAI_AGAIN;
		}
		if (n != sizeof(ip)) {
			return EAI_FAIL;
		}
	}

	entry = dns_alloc(sizeof(*entry));
	if (!entry) {
		return EAI_MEMORY;
	}
	memset(entry, 0, sizeof(*entry));
	addr = &entry->addr;
	addr->sin_family = AF_INET;
	addr->sin_port = htons(port);
	memcpy(&addr->sin_addr, ip, sizeof(ip));
	entry->ai.ai_family = AF_INET;
	entry->ai.ai_socktype = type;
	entry->ai.ai_protocol = protocol;
	entry->ai.ai_addrlen = sizeof(*addr);
	entry->ai.ai_addr = (struct sockaddr *)addr;
	*res = &entry->ai;
	return 0;
}

void nina_dns_freeaddrinfo(struct addrinfo *result)
{
	/* The entry starts with its struct addrinfo. */
	dns_free(result);
}

/* With NINA_OS_DNS_OFFLOAD (the Zephyr build), the OS makes
 * nina_dns_getaddrinfo() the resolver of its sockets (from iface_init() of
 * zephyr/nina_net.c).  Elsewhere the callers reach nina_dns_getaddrinfo() directly
 * (zsock_getaddrinfo() of the compatibility layers), so there is nothing
 * to register.
 */
void nina_dns_init(void)
{
#ifdef NINA_OS_DNS_OFFLOAD
	nina_os_dns_register();
#endif
}
