/* SPDX-License-Identifier: Apache-2.0 */
#include <errno.h>
#include <stdlib.h>
#include <string.h>
#include <zephyr/kernel.h>
#include <zephyr/net/dns_resolve.h>
#include <zephyr/net/socket_offload.h>
#include "nina_proto.h"

/* Protect NINA's single shared DNS result across both protocol commands. */
K_MUTEX_DEFINE(nina_dns_lock);
K_HEAP_DEFINE(nina_dns_heap, 2048);

static int resolve(const char *host, const char *service,
		   const struct zsock_addrinfo *hints,
		   struct zsock_addrinfo **res)
{
	struct zsock_addrinfo *entry;
	struct net_sockaddr_in *addr;
	unsigned long port = 0;
	uint8_t ip[4], answer;
	size_t n, host_len;
	int ret, family = NET_AF_UNSPEC, type = NET_SOCK_STREAM;
	int protocol = NET_IPPROTO_TCP, flags = 0;

	if (!res || !host || !*host) {
		return DNS_EAI_NONAME;
	}
	*res = NULL;
	if (hints) {
		family = hints->ai_family;
		if (hints->ai_socktype) {
			type = hints->ai_socktype;
		} else if (hints->ai_protocol == NET_IPPROTO_UDP) {
			type = NET_SOCK_DGRAM;
		}
		protocol = hints->ai_protocol       ? hints->ai_protocol
			   : type == NET_SOCK_DGRAM ? NET_IPPROTO_UDP
						    : NET_IPPROTO_TCP;
		flags = hints->ai_flags;
	}
	if (flags & ~(ZSOCK_AI_NUMERICHOST | ZSOCK_AI_NUMERICSERV |
		      ZSOCK_AI_ADDRCONFIG)) {
		return DNS_EAI_BADFLAGS;
	}
	if (family != NET_AF_UNSPEC && family != NET_AF_INET) {
		return DNS_EAI_FAMILY;
	}
	if (!((type == NET_SOCK_STREAM && protocol == NET_IPPROTO_TCP) ||
	      (type == NET_SOCK_DGRAM && protocol == NET_IPPROTO_UDP))) {
		return DNS_EAI_SOCKTYPE;
	}
	if (service) {
		char *end;
		if (!*service ||
		    strspn(service, "0123456789") != strlen(service)) {
			return DNS_EAI_SERVICE;
		}
		errno = 0;
		port = strtoul(service, &end, 10);
		if (errno || *end || port > 65535) {
			return DNS_EAI_SERVICE;
		}
	}

	if (zsock_inet_pton(NET_AF_INET, host, ip) != 1) {
		struct nina_param param;

		if (flags & ZSOCK_AI_NUMERICHOST) {
			return DNS_EAI_NONAME;
		}
		host_len = strlen(host);
		if (host_len > 253) {
			return DNS_EAI_NONAME;
		}
		param = (struct nina_param){(const uint8_t *)host, host_len};
		k_mutex_lock(&nina_dns_lock, K_FOREVER);
		ret = nina_request(0x34, &param, 1, false, false, &answer, 1,
				   &n);
		if (!ret && n == 1 && answer == 1) {
			ret = nina_request(0x35, NULL, 0, false, false, ip,
					   sizeof(ip), &n);
		} else if (!ret) {
			ret = -ENOENT;
		}
		k_mutex_unlock(&nina_dns_lock);
		if (ret) {
			return ret == -ENOENT ? DNS_EAI_NONAME : DNS_EAI_AGAIN;
		}
		if (n != sizeof(ip)) {
			return DNS_EAI_FAIL;
		}
	}

	entry = k_heap_alloc(&nina_dns_heap, sizeof(*entry), K_NO_WAIT);
	if (!entry) {
		return DNS_EAI_MEMORY;
	}
	memset(entry, 0, sizeof(*entry));
	addr = (struct net_sockaddr_in *)&entry->_ai_addr;
	addr->sin_family = NET_AF_INET;
	addr->sin_port = net_htons(port);
	memcpy(&addr->sin_addr, ip, sizeof(ip));
	entry->ai_family = NET_AF_INET;
	entry->ai_socktype = type;
	entry->ai_protocol = protocol;
	entry->ai_addrlen = sizeof(*addr);
	entry->ai_addr = (struct net_sockaddr *)addr;
	*res = entry;
	return 0;
}

static void free_result(struct zsock_addrinfo *result)
{
	k_heap_free(&nina_dns_heap, result);
}

static const struct socket_dns_offload dns_ops = {
	.getaddrinfo = resolve,
	.freeaddrinfo = free_result,
};

void nina_dns_init(void)
{
	socket_offload_dns_register(&dns_ops);
}
