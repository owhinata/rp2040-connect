/* SPDX-License-Identifier: Apache-2.0 */
/* Host test of common/drivers/nina/nina_dns.c with a fake nina_proto_request()
 * and a fake lock: the hints (defaults, refused flags, families, socket
 * types), the service, numeric addresses without the module, a name with
 * the commands 0x34 and 0x35 under the lock, and each failure's EAI_*
 * value.  Built twice: as the NuttX and FreeRTOS builds compile it
 * (malloc()), and with NINA_OS_DNS_OFFLOAD as the Zephyr build does (the
 * results on the OS's heap, here a fake one of two entries, and
 * nina_dns_init() registering the resolver).
 */

#include <assert.h>
#include <errno.h>
#include <stdio.h>
#include <string.h>

#include <arpa/inet.h>
#include <netdb.h>
#include <netinet/in.h>
#include <sys/socket.h>

#include "nina_dns.h"
#include "nina_os.h"
#include "nina_proto.h"

static int locked, lock_count;

void nina_os_dns_lock(void)
{
	assert(!locked);
	locked = 1;
	lock_count++;
}

void nina_os_dns_unlock(void)
{
	assert(locked);
	locked = 0;
}

#ifdef NINA_OS_DNS_OFFLOAD
/* The OS's heap: room for two results. */
static unsigned char heap[2][256];
static bool heap_used[2];
static int allocs, frees, registers;

void *nina_os_dns_alloc(size_t size)
{
	assert(size <= sizeof(heap[0]));
	for (int i = 0; i < 2; i++) {
		if (!heap_used[i]) {
			heap_used[i] = true;
			allocs++;
			return heap[i];
		}
	}
	return NULL;
}

void nina_os_dns_free(void *ptr)
{
	for (int i = 0; i < 2; i++) {
		if (ptr == heap[i]) {
			assert(heap_used[i]);
			heap_used[i] = false;
			frees++;
			return;
		}
	}
	assert(0);
}

void nina_os_dns_register(void)
{
	registers++;
}
#endif

struct step {
	uint8_t cmd;
	int ret;
	uint8_t reply[4];
	size_t reply_len;
};

static struct step script[16];
static size_t nsteps, next_step;
static char host_param[300];
static size_t host_param_len;

static void expect(uint8_t cmd, int ret, const uint8_t *reply, size_t len)
{
	assert(nsteps < sizeof(script) / sizeof(script[0]) && len <= sizeof(script[0].reply));
	script[nsteps].cmd = cmd;
	script[nsteps].ret = ret;
	memcpy(script[nsteps].reply, reply, len);
	script[nsteps].reply_len = len;
	nsteps++;
}

static void reset(void)
{
	nsteps = next_step = 0;
	lock_count = 0;
	host_param_len = 0;
}

static void done(void)
{
	assert(next_step == nsteps && !locked);
}

int nina_proto_request(uint8_t cmd, const struct nina_proto_param *p, size_t nparams, bool wide_params,
		 bool wide_response, uint8_t *out, size_t cap, size_t *out_len)
{
	struct step *s;

	assert(next_step < nsteps && locked);
	s = &script[next_step++];
	assert(s->cmd == cmd && !wide_params && !wide_response);
	if (cmd == 0x34) {
		assert(nparams == 1 && p[0].len < sizeof(host_param));
		memcpy(host_param, p[0].data, p[0].len);
		host_param_len = p[0].len;
		assert(cap == 1);
	} else {
		assert(cmd == 0x35 && nparams == 0 && cap == 4);
	}
	if (s->ret) {
		return s->ret;
	}
	assert(s->reply_len <= cap);
	memcpy(out, s->reply, s->reply_len);
	*out_len = s->reply_len;
	return 0;
}

static const struct sockaddr_in *sin_of(const struct addrinfo *ai)
{
	assert(ai && ai->ai_addr && ai->ai_addrlen == sizeof(struct sockaddr_in));
	return (const struct sockaddr_in *)ai->ai_addr;
}

int main(void)
{
	struct addrinfo hints, *res = (struct addrinfo *)1;
	char longname[256];

	/* No name. */
	reset();
	assert(nina_dns_getaddrinfo(NULL, NULL, NULL, &res) == EAI_NONAME);
	assert(nina_dns_getaddrinfo("", NULL, NULL, &res) == EAI_NONAME);
	assert(nina_dns_getaddrinfo("a", NULL, NULL, NULL) == EAI_NONAME);

	/* A numeric address: no command; TCP by default; the port. */
	res = (struct addrinfo *)1;
	assert(nina_dns_getaddrinfo("10.42.0.1", "1883", NULL, &res) == 0);
	assert(res->ai_family == AF_INET && res->ai_socktype == SOCK_STREAM &&
	       res->ai_protocol == IPPROTO_TCP && res->ai_next == NULL);
	assert(sin_of(res)->sin_family == AF_INET);
	assert(sin_of(res)->sin_addr.s_addr == inet_addr("10.42.0.1"));
	assert(ntohs(sin_of(res)->sin_port) == 1883);
	nina_dns_freeaddrinfo(res);
	done();
	assert(lock_count == 0);

	/* Hints: UDP from the protocol alone or the type; family. */
	memset(&hints, 0, sizeof(hints));
	hints.ai_protocol = IPPROTO_UDP;
	assert(nina_dns_getaddrinfo("10.42.0.1", NULL, &hints, &res) == 0);
	assert(res->ai_socktype == SOCK_DGRAM && res->ai_protocol == IPPROTO_UDP);
	assert(sin_of(res)->sin_port == 0);
	nina_dns_freeaddrinfo(res);
	memset(&hints, 0, sizeof(hints));
	hints.ai_family = AF_INET;
	hints.ai_socktype = SOCK_DGRAM;
	assert(nina_dns_getaddrinfo("10.42.0.1", "123", &hints, &res) == 0);
	assert(res->ai_protocol == IPPROTO_UDP && ntohs(sin_of(res)->sin_port) == 123);
	nina_dns_freeaddrinfo(res);
	hints.ai_family = AF_INET6;
	assert(nina_dns_getaddrinfo("10.42.0.1", NULL, &hints, &res) == EAI_FAMILY);
	hints.ai_family = AF_UNSPEC;
	hints.ai_protocol = IPPROTO_TCP;
	assert(nina_dns_getaddrinfo("10.42.0.1", NULL, &hints, &res) == EAI_SOCKTYPE);
	hints.ai_socktype = SOCK_RAW;
	hints.ai_protocol = 0;
	assert(nina_dns_getaddrinfo("10.42.0.1", NULL, &hints, &res) == EAI_SOCKTYPE);
	memset(&hints, 0, sizeof(hints));
	hints.ai_flags = AI_PASSIVE;
	assert(nina_dns_getaddrinfo("10.42.0.1", NULL, &hints, &res) == EAI_BADFLAGS);
	hints.ai_flags = AI_NUMERICHOST | AI_NUMERICSERV | AI_ADDRCONFIG;
	assert(nina_dns_getaddrinfo("10.42.0.1", NULL, &hints, &res) == 0);
	nina_dns_freeaddrinfo(res);
	/* A name with AI_NUMERICHOST: no command. */
	assert(nina_dns_getaddrinfo("pc.local", NULL, &hints, &res) == EAI_NONAME);

	/* The service: digits only, at most 65535. */
	assert(nina_dns_getaddrinfo("10.42.0.1", "", NULL, &res) == EAI_SERVICE);
	assert(nina_dns_getaddrinfo("10.42.0.1", "ntp", NULL, &res) == EAI_SERVICE);
	assert(nina_dns_getaddrinfo("10.42.0.1", "-1", NULL, &res) == EAI_SERVICE);
	assert(nina_dns_getaddrinfo("10.42.0.1", "65536", NULL, &res) == EAI_SERVICE);
	assert(nina_dns_getaddrinfo("10.42.0.1", "65535", NULL, &res) == 0);
	nina_dns_freeaddrinfo(res);
	done();

	/* A name: 0x34 with the name, then 0x35 for the address, locked. */
	reset();
	expect(0x34, 0, (const uint8_t[]){1}, 1);
	expect(0x35, 0, (const uint8_t[]){10, 42, 0, 7}, 4);
	assert(nina_dns_getaddrinfo("pc.local", "123", NULL, &res) == 0);
	done();
	assert(lock_count == 1);
	assert(host_param_len == 8 && !memcmp(host_param, "pc.local", 8));
	assert(sin_of(res)->sin_addr.s_addr == inet_addr("10.42.0.7"));
	assert(ntohs(sin_of(res)->sin_port) == 123);
	nina_dns_freeaddrinfo(res);

	/* Not resolved: EAI_NONAME; an SPI error: EAI_AGAIN (also on the
	 * second command); a short address: EAI_FAIL. */
	reset();
	expect(0x34, 0, (const uint8_t[]){0}, 1);
	assert(nina_dns_getaddrinfo("nowhere", NULL, NULL, &res) == EAI_NONAME && res == NULL);
	expect(0x34, 0, (const uint8_t[]){2}, 1);
	assert(nina_dns_getaddrinfo("nowhere", NULL, NULL, &res) == EAI_NONAME);
	expect(0x34, -ETIMEDOUT, NULL, 0);
	assert(nina_dns_getaddrinfo("nowhere", NULL, NULL, &res) == EAI_AGAIN);
	expect(0x34, 0, (const uint8_t[]){1}, 1);
	expect(0x35, -EIO, NULL, 0);
	assert(nina_dns_getaddrinfo("nowhere", NULL, NULL, &res) == EAI_AGAIN);
	expect(0x34, 0, (const uint8_t[]){1}, 1);
	expect(0x35, 0, (const uint8_t[]){10, 42, 0}, 3);
	assert(nina_dns_getaddrinfo("nowhere", NULL, NULL, &res) == EAI_FAIL);
	done();
	assert(lock_count == 5);

	/* At most 253 characters reach the module. */
	reset();
	memset(longname, 'a', sizeof(longname));
	longname[254] = '\0';
	assert(nina_dns_getaddrinfo(longname, NULL, NULL, &res) == EAI_NONAME);
	longname[253] = '\0';
	expect(0x34, 0, (const uint8_t[]){0}, 1);
	assert(nina_dns_getaddrinfo(longname, NULL, NULL, &res) == EAI_NONAME);
	done();
	assert(host_param_len == 253);

#ifdef NINA_OS_DNS_OFFLOAD
	/* Every result came from the OS's heap and went back; a full heap is
	 * EAI_MEMORY; nina_dns_init() registers the resolver. */
	{
		struct addrinfo *a, *b;

		assert(allocs > 0 && allocs == frees);
		reset();
		assert(nina_dns_getaddrinfo("10.0.0.1", NULL, NULL, &a) == 0);
		assert(nina_dns_getaddrinfo("10.0.0.2", NULL, NULL, &b) == 0);
		assert(nina_dns_getaddrinfo("10.0.0.3", NULL, NULL, &res) == EAI_MEMORY);
		nina_dns_freeaddrinfo(a);
		nina_dns_freeaddrinfo(b);
		done();
		assert(allocs == frees);
		nina_dns_init();
		assert(registers == 1);
	}
#else
	nina_dns_init();
#endif

	puts("nina_dns: ok");
	return 0;
}
