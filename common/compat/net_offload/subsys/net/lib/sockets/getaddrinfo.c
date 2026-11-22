/*
 * Copyright (c) 2017 Linaro Limited
 * Copyright (c) 2020 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/* zsock_getaddrinfo() and zsock_freeaddrinfo() of Zephyr's
 * subsys/net/lib/sockets/getaddrinfo.c (Zephyr v4.5.0), with Zephyr's text
 * for the offloaded resolver (socket_offload.c here).  Left out: the native
 * resolver and the literal addresses (no CONFIG_NET_IP, no
 * CONFIG_DNS_RESOLVER: the layer has no network stack), so without an
 * offloaded resolver the result is EAI_FAIL (Zephyr's DNS_EAI_FAIL, -4,
 * <netdb.h> here) and nothing is allocated, and zsock_freeaddrinfo() has
 * nothing to free (Zephyr: k_free()).  Also left out: zsock_gai_strerror()
 * and the user mode.
 * Part of the network part of the Zephyr compatibility layer for an OS
 * without a network stack of its own (common/compat/net_offload), which the
 * FreeRTOS build uses.
 */

#include <zephyr/kernel.h>
#include <zephyr/net/socket.h>
#include <zephyr/net/socket_offload.h>

int zsock_getaddrinfo(const char *host, const char *service,
		      const struct zsock_addrinfo *hints,
		      struct zsock_addrinfo **res)
{
	if (socket_offload_dns_is_enabled()) {
		return socket_offload_getaddrinfo(host, service, hints, res);
	}

	int ret = EAI_FAIL;

	return ret;
}

void zsock_freeaddrinfo(struct zsock_addrinfo *ai)
{
	if (socket_offload_dns_is_enabled()) {
		socket_offload_freeaddrinfo(ai);
		return;
	}
}
