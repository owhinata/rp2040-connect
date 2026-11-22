/* SPDX-License-Identifier: MIT */
/* The host test of the layer's socket part on the FreeRTOS POSIX port: the
 * reduced copies of Zephyr's socket layer (common/compat/net_offload/subsys/
 * net/lib/sockets: sockets.c, getaddrinfo.c, socket_offload.c) and
 * descriptor table (common/compat/net_offload/lib/os/zvfs: zvfs_fdtable.c,
 * zvfs_poll.c), and k_heap (common/compat/net_offload/zephyr_heap.c), as
 * freertos/drivers/compat/compat.cmake builds them, on fake socket
 * implementations registered here and the host linker's section of them
 * (net_socket_register.ld here):
 *
 * - the descriptor table: reserve, finalize and free, the lowest free
 *   number first, reuse, ENFILE when all 16 are taken, the object of a
 *   descriptor with and without the vtable check, a free that does not go
 *   below zero;
 * - the list of implementations: the priority order (the names sorted by
 *   the linker), a family that does not match (its is_supported() not
 *   asked), is_supported() refusing (the next one), NET_AF_UNSPEC, none
 *   (EAFNOSUPPORT), the handler's result and errno;
 * - the dispatch: every zsock_*() call to its vtable entry with its
 *   arguments and result, EBADF for a descriptor that is not open,
 *   EOPNOTSUPP for a NULL entry (ENOTSUP for shutdown), close through
 *   close2() with the descriptor for a socket, the descriptor's lock held
 *   around each call (a close waits for a call in progress), fcntl to the
 *   ioctl method;
 * - poll: an offloaded descriptor takes the whole call (POLL_PREPARE
 *   -EXDEV, POLL_OFFLOAD with all descriptors and the timeout in ms), also
 *   with descriptors of another kind; the other kinds on k_poll() events
 *   (POLL_PREPARE, POLL_UPDATE), -EALREADY (no wait), a POLL_PREPARE error,
 *   POLLNVAL, negative descriptors, the whole timeout without events;
 * - the resolver: EAI_FAIL without one, the registered one, deregistered;
 * - k_heap: the bound of its buffer, zero and oversized allocations, reuse
 *   and joining of freed blocks, 8-byte alignment.
 *
 * Built and run by freertos/script/host_tests.sh.
 */

#undef NDEBUG
#include <assert.h>
#include <errno.h>
#include <fcntl.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>

#include "FreeRTOS.h"
#include "task.h"

#include <zephyr/kernel.h>
#include <zephyr/net/socket.h>
#include <zephyr/net/socket_offload.h>
#include <zephyr/sys/fdtable.h>

#include "priorities.h"
#include "zephyr_compat.h"

#define RUNNER_STACK 32768
#define HELPER_STACK 32768
#define TABLE_SIZE   16 /* CONFIG_ZVFS_OPEN_MAX of the Zephyr build */

_Thread_local int z_host_isr_depth;

/* tick.c's vApplicationTickHook(), counted as an interrupt handler. */
void app_tick_hook(void);

void vApplicationTickHook(void)
{
	z_host_isr_depth++;
	app_tick_hook();
	z_host_isr_depth--;
}

#define CHECK_FAIL(call, err)                                                                      \
	do {                                                                                       \
		errno = 0;                                                                         \
		if ((call) != -1 || errno != (err)) {                                              \
			fprintf(stderr, "%s:%d: %s: errno %d, expected %d\n", __FILE__, __LINE__,  \
				#call, errno, err);                                                \
			assert(0);                                                                 \
		}                                                                                  \
	} while (0)

/* The fake implementations ------------------------------------------------*/

/* What the calls did, in order: "<impl>:<op>" separated by spaces. */
static char trace[512];

static void note(const char *what)
{
	size_t n = strlen(trace);

	assert(n + strlen(what) + 2 < sizeof(trace));
	snprintf(trace + n, sizeof(trace) - n, "%s%s", n ? " " : "", what);
}

static void trace_expect(const char *expected)
{
	if (strcmp(trace, expected) != 0) {
		fprintf(stderr, "trace:    \"%s\"\nexpected: \"%s\"\n", trace, expected);
		assert(0);
	}
	trace[0] = '\0';
}

struct fake {
	bool used;
	char impl; /* 'f' full (beta), 's' sparse (alpha) */
	int fd;
	int nonblocking;
	int closed_with_fd;
	struct k_mutex *lock; /* from ZFD_IOCTL_SET_LOCK */
	struct k_sem readable;
	bool blocking_recv;
	struct k_sem recv_go;
};

static struct fake fakes[TABLE_SIZE];
static int last_arg[6];
static const void *last_ptr[3];
static struct k_sem recv_started;
static int prepare_result; /* what the full kind's POLL_PREPARE returns, 0 normally */
static int offload_nfds, offload_timeout;
static short offload_revents = ZSOCK_POLLIN;

static struct fake *fake_new(char impl)
{
	for (size_t i = 0; i < ARRAY_SIZE(fakes); i++) {
		if (!fakes[i].used) {
			memset(&fakes[i], 0, sizeof(fakes[i]));
			fakes[i].used = true;
			fakes[i].impl = impl;
			k_sem_init(&fakes[i].readable, 0, 1);
			k_sem_init(&fakes[i].recv_go, 0, 1);
			return &fakes[i];
		}
	}
	return NULL;
}

/* The full kind: every operation, a native socket for poll (k_poll()
 * events on its semaphore).
 */
static int full_close(void *obj, int fd)
{
	struct fake *f = obj;

	note("full:close");
	f->closed_with_fd = fd;
	f->used = false;
	return 0;
}

static int full_ioctl(void *obj, unsigned int request, va_list args)
{
	struct fake *f = obj;

	switch (request) {
	case ZFD_IOCTL_SET_LOCK:
		f->lock = va_arg(args, struct k_mutex *);
		return 0;
	case ZVFS_F_GETFL:
		note("full:getfl");
		return f->nonblocking ? ZVFS_O_NONBLOCK : 0;
	case ZVFS_F_SETFL:
		note("full:setfl");
		f->nonblocking = va_arg(args, int) & ZVFS_O_NONBLOCK;
		return 0;
	case ZFD_IOCTL_POLL_PREPARE: {
		struct zsock_pollfd *pfd = va_arg(args, struct zsock_pollfd *);
		struct k_poll_event **pev = va_arg(args, struct k_poll_event **);
		struct k_poll_event *pev_end = va_arg(args, struct k_poll_event *);

		note("full:prepare");
		if (prepare_result) {
			return prepare_result;
		}
		assert(*pev < pev_end && (pfd->events & ZSOCK_POLLIN));
		k_poll_event_init(*pev, K_POLL_TYPE_SEM_AVAILABLE, K_POLL_MODE_NOTIFY_ONLY,
				  &f->readable);
		(*pev)++;
		return 0;
	}
	case ZFD_IOCTL_POLL_UPDATE: {
		struct zsock_pollfd *pfd = va_arg(args, struct zsock_pollfd *);
		struct k_poll_event **pev = va_arg(args, struct k_poll_event **);

		note("full:update");
		if (k_sem_count_get(&f->readable)) {
			pfd->revents = ZSOCK_POLLIN;
		}
		(*pev)++;
		return 0;
	}
	default:
		errno = EINVAL;
		return -1;
	}
}

static int full_shutdown(void *obj, int how)
{
	ARG_UNUSED(obj);
	note("full:shutdown");
	last_arg[0] = how;
	return 0;
}

static int full_bind(void *obj, const struct net_sockaddr *addr, net_socklen_t addrlen)
{
	ARG_UNUSED(obj);
	note("full:bind");
	last_ptr[0] = addr;
	last_arg[0] = (int)addrlen;
	return 0;
}

static int full_connect(void *obj, const struct net_sockaddr *addr, net_socklen_t addrlen)
{
	ARG_UNUSED(obj);
	note("full:connect");
	last_ptr[0] = addr;
	last_arg[0] = (int)addrlen;
	errno = ECONNREFUSED;
	return -1;
}

static int full_listen(void *obj, int backlog)
{
	ARG_UNUSED(obj);
	note("full:listen");
	last_arg[0] = backlog;
	return 0;
}

static int fake_socket_full(int family, int type, int proto);

static int full_accept(void *obj, struct net_sockaddr *addr, net_socklen_t *addrlen)
{
	ARG_UNUSED(obj);
	note("full:accept");
	last_ptr[0] = addr;
	last_ptr[1] = addrlen;
	return fake_socket_full(NET_AF_INET, NET_SOCK_STREAM, 0);
}

static ssize_t full_sendto(void *obj, const void *buf, size_t len, int flags,
			   const struct net_sockaddr *dest_addr, net_socklen_t addrlen)
{
	ARG_UNUSED(obj);
	note("full:sendto");
	last_ptr[0] = buf;
	last_ptr[1] = dest_addr;
	last_arg[0] = (int)len;
	last_arg[1] = flags;
	last_arg[2] = (int)addrlen;
	return (ssize_t)len;
}

static ssize_t full_recvfrom(void *obj, void *buf, size_t max_len, int flags,
			     struct net_sockaddr *src_addr, net_socklen_t *addrlen)
{
	struct fake *f = obj;

	note("full:recvfrom");
	last_ptr[0] = buf;
	last_ptr[1] = src_addr;
	last_ptr[2] = addrlen;
	last_arg[0] = (int)max_len;
	last_arg[1] = flags;
	if (f->blocking_recv) {
		k_sem_give(&recv_started);
		(void)k_sem_take(&f->recv_go, K_FOREVER);
	}
	return 3;
}

static int full_getsockopt(void *obj, int level, int optname, void *optval, net_socklen_t *optlen)
{
	ARG_UNUSED(obj);
	note("full:getsockopt");
	last_arg[0] = level;
	last_arg[1] = optname;
	last_ptr[0] = optval;
	last_ptr[1] = optlen;
	return 0;
}

static int full_setsockopt(void *obj, int level, int optname, const void *optval,
			   net_socklen_t optlen)
{
	ARG_UNUSED(obj);
	note("full:setsockopt");
	last_arg[0] = level;
	last_arg[1] = optname;
	last_arg[2] = (int)optlen;
	last_ptr[0] = optval;
	return 0;
}

static ssize_t full_sendmsg(void *obj, const struct net_msghdr *msg, int flags)
{
	ARG_UNUSED(obj);
	note("full:sendmsg");
	last_ptr[0] = msg;
	last_arg[0] = flags;
	return 7;
}

static ssize_t full_recvmsg(void *obj, struct net_msghdr *msg, int flags)
{
	ARG_UNUSED(obj);
	note("full:recvmsg");
	last_ptr[0] = msg;
	last_arg[0] = flags;
	return 5;
}

static int full_getpeername(void *obj, struct net_sockaddr *addr, net_socklen_t *addrlen)
{
	ARG_UNUSED(obj);
	note("full:getpeername");
	last_ptr[0] = addr;
	last_ptr[1] = addrlen;
	return 0;
}

static int full_getsockname(void *obj, struct net_sockaddr *addr, net_socklen_t *addrlen)
{
	ARG_UNUSED(obj);
	note("full:getsockname");
	last_ptr[0] = addr;
	last_ptr[1] = addrlen;
	return 0;
}

static const struct socket_op_vtable full_ops = {
	.fd_vtable = {.close2 = full_close, .ioctl = full_ioctl},
	.shutdown = full_shutdown,
	.bind = full_bind,
	.connect = full_connect,
	.listen = full_listen,
	.accept = full_accept,
	.sendto = full_sendto,
	.recvfrom = full_recvfrom,
	.getsockopt = full_getsockopt,
	.setsockopt = full_setsockopt,
	.sendmsg = full_sendmsg,
	.recvmsg = full_recvmsg,
	.getpeername = full_getpeername,
	.getsockname = full_getsockname,
};

/* The sparse kind: an offloaded socket with close and ioctl only (the
 * offload's poll for every descriptor).
 */
static int sparse_close(void *obj)
{
	struct fake *f = obj;

	note("sparse:close");
	f->used = false;
	return 0;
}

static int sparse_ioctl(void *obj, unsigned int request, va_list args)
{
	ARG_UNUSED(obj);

	switch (request) {
	case ZFD_IOCTL_POLL_PREPARE:
		note("sparse:prepare");
		return -EXDEV;
	case ZFD_IOCTL_POLL_OFFLOAD: {
		struct zsock_pollfd *fds = va_arg(args, struct zsock_pollfd *);

		note("sparse:offload");
		offload_nfds = va_arg(args, int);
		offload_timeout = va_arg(args, int);
		for (int i = 0; i < offload_nfds; i++) {
			fds[i].revents = fds[i].fd >= 0 ? offload_revents : 0;
		}
		return offload_nfds;
	}
	default:
		errno = ENOTSUP;
		return -1;
	}
}

static const struct socket_op_vtable sparse_ops = {
	.fd_vtable = {.close = sparse_close, .ioctl = sparse_ioctl},
};

/* The handlers and is_supported() of the registrations. */
static int fake_socket(char impl, const struct socket_op_vtable *ops)
{
	struct fake *f;
	int fd = zvfs_reserve_fd();

	if (fd < 0) {
		return -1;
	}
	f = fake_new(impl);
	assert(f != NULL);
	f->fd = fd;
	zvfs_finalize_typed_fd(fd, f, (const struct fd_op_vtable *)ops, ZVFS_MODE_IFSOCK);
	return fd;
}

static int handler_errno;

static int fake_socket_full(int family, int type, int proto)
{
	ARG_UNUSED(family);
	ARG_UNUSED(type);
	ARG_UNUSED(proto);
	handler_errno = errno;
	note("full:socket");
	return fake_socket('f', &full_ops);
}

static int fake_socket_sparse(int family, int type, int proto)
{
	ARG_UNUSED(family);
	ARG_UNUSED(type);
	ARG_UNUSED(proto);
	note("sparse:socket");
	return fake_socket('s', &sparse_ops);
}

static int refuse_socket(int family, int type, int proto)
{
	ARG_UNUSED(family);
	ARG_UNUSED(type);
	ARG_UNUSED(proto);
	assert(0);
	return -1;
}

static int six_socket(int family, int type, int proto)
{
	ARG_UNUSED(family);
	ARG_UNUSED(type);
	ARG_UNUSED(proto);
	note("six:socket");
	errno = ENOMEM;
	return -1;
}

static bool refuse_supported(int family, int type, int proto)
{
	ARG_UNUSED(family);
	ARG_UNUSED(type);
	ARG_UNUSED(proto);
	note("refuse:supported");
	return false;
}

static bool six_supported(int family, int type, int proto)
{
	ARG_UNUSED(family);
	ARG_UNUSED(type);
	ARG_UNUSED(proto);
	note("six:supported");
	return true;
}

/* The sparse kind takes datagrams only. */
static bool sparse_supported(int family, int type, int proto)
{
	ARG_UNUSED(family);
	ARG_UNUSED(proto);
	note("sparse:supported");
	return type == NET_SOCK_DGRAM;
}

/* The full kind takes every family, but not the type 99. */
static bool full_supported(int family, int type, int proto)
{
	ARG_UNUSED(family);
	ARG_UNUSED(proto);
	note("full:supported");
	return type != 99;
}

/* Defined out of their order: the linker sorts them (10, 20, 30, 50). */
NET_SOCKET_REGISTER(full, 50, NET_AF_UNSPEC, full_supported, fake_socket_full);
NET_SOCKET_OFFLOAD_REGISTER(sparse, 30, NET_AF_INET, sparse_supported, fake_socket_sparse);
NET_SOCKET_REGISTER(six, 20, NET_AF_INET6, six_supported, six_socket);
NET_SOCKET_REGISTER(refuse, 10, NET_AF_INET, refuse_supported, refuse_socket);

extern const struct net_socket_register _net_socket_register_list_start[];
extern const struct net_socket_register _net_socket_register_list_end[];

static void close_all(void)
{
	for (int fd = 0; fd < TABLE_SIZE; fd++) {
		if (zvfs_get_fd_obj(fd, NULL, 0)) {
			assert(zsock_close(fd) == 0);
		}
	}
	trace[0] = '\0';
}

/* The descriptor table ------------------------------------------------------*/

static void test_fdtable(void)
{
	static int objs[TABLE_SIZE + 1];
	static const struct fd_op_vtable other = {0};
	const struct fd_op_vtable *vt;
	struct k_mutex *lock = NULL;
	int fd;

	for (int i = 0; i < TABLE_SIZE; i++) {
		fd = zvfs_reserve_fd();
		assert(fd == i);
		/* Reserved, not finalized: open, without an object. */
		assert(zvfs_get_fd_obj(fd, NULL, EINVAL) == NULL);
		zvfs_finalize_fd(fd, &objs[i], &other);
	}
	errno = 0;
	assert(zvfs_reserve_fd() == -1 && errno == ENFILE);
	assert(zvfs_alloc_fd(&objs[TABLE_SIZE], &other) == -1 && errno == ENFILE);

	assert(zvfs_get_fd_obj(3, NULL, 0) == &objs[3]);
	assert(zvfs_get_fd_obj(3, &other, EINVAL) == &objs[3]);
	errno = 0;
	assert(zvfs_get_fd_obj(3, (const struct fd_op_vtable *)&full_ops, EINVAL) == NULL &&
	       errno == EINVAL);
	assert(zvfs_get_fd_obj_and_vtable(3, &vt, &lock) == &objs[3] && vt == &other && lock);

	/* Freed: EBADF; the lowest free number is used again; a second free
	 * does nothing. */
	zvfs_free_fd(3);
	zvfs_free_fd(9);
	zvfs_free_fd(9);
	errno = 0;
	assert(zvfs_get_fd_obj(3, NULL, 0) == NULL && errno == EBADF);
	errno = 0;
	assert(zvfs_get_fd_obj_and_vtable(9, &vt, NULL) == NULL && errno == EBADF);
	assert(zvfs_alloc_fd(&objs[3], &other) == 3);
	assert(zvfs_reserve_fd() == 9);
	errno = 0;
	assert(zvfs_reserve_fd() == -1 && errno == ENFILE);
	zvfs_free_fd(9);

	/* Out of range. */
	errno = 0;
	assert(zvfs_get_fd_obj(-1, NULL, 0) == NULL && errno == EBADF);
	errno = 0;
	assert(zvfs_get_fd_obj(TABLE_SIZE, NULL, 0) == NULL && errno == EBADF);

	/* A close without a close method frees the entry. */
	for (int i = 0; i < TABLE_SIZE; i++) {
		if (i != 9) {
			assert(zvfs_close(i) == 0);
		}
	}
	CHECK_FAIL(zvfs_close(0), EBADF);
	CHECK_FAIL(zvfs_close(TABLE_SIZE), EBADF);
	assert(zvfs_reserve_fd() == 0);
	zvfs_free_fd(0);
	printf("ok  fdtable\n");
}

/* The list of implementations ---------------------------------------------*/

static void test_registrations(void)
{
	const struct net_socket_register *r = _net_socket_register_list_start;
	int fd;

	/* The linker's order is the priorities'. */
	assert(_net_socket_register_list_end - _net_socket_register_list_start == 4);
	assert(r[0].handler == refuse_socket && r[1].handler == six_socket &&
	       r[2].handler == fake_socket_sparse && r[3].handler == fake_socket_full);
	assert(!r[0].is_offloaded && !r[1].is_offloaded && r[2].is_offloaded && !r[3].is_offloaded);

	/* A datagram: refused at 10, family not matched at 20 (is_supported()
	 * not asked), taken at 30. */
	fd = zsock_socket(NET_AF_INET, NET_SOCK_DGRAM, 0);
	assert(fd == 0);
	trace_expect("refuse:supported sparse:supported sparse:socket");
	assert(zvfs_get_fd_obj(fd, (const struct fd_op_vtable *)&sparse_ops, 0) != NULL);

	/* A stream: refused at 10 and 30, taken by the NET_AF_UNSPEC one. */
	fd = zsock_socket(NET_AF_INET, NET_SOCK_STREAM, 0);
	assert(fd == 1);
	trace_expect("refuse:supported sparse:supported full:supported full:socket");
	assert(zvfs_get_fd_obj(fd, (const struct fd_op_vtable *)&full_ops, 0) != NULL);

	/* IPv6: the one at 20; its handler's failure is the call's (errno
	 * ENOMEM), the later ones are not tried. */
	CHECK_FAIL(zsock_socket(NET_AF_INET6, NET_SOCK_STREAM, 0), ENOMEM);
	trace_expect("six:supported six:socket");

	/* Another family: only the NET_AF_UNSPEC one, which takes it; its
	 * handler sees errno 0 (the layer clears it first). */
	errno = EIO;
	fd = zsock_socket(17, NET_SOCK_DGRAM, 0);
	assert(fd == 2 && handler_errno == 0);
	trace_expect("full:supported full:socket");

	/* None takes it: EAFNOSUPPORT. */
	CHECK_FAIL(zsock_socket(17, 99, 0), EAFNOSUPPORT);
	trace_expect("full:supported");
	CHECK_FAIL(zsock_socket(NET_AF_INET, 99, 0), EAFNOSUPPORT);
	trace_expect("refuse:supported sparse:supported full:supported");
	close_all();
	printf("ok  registrations\n");
}

/* The dispatch -----------------------------------------------------------------*/

static volatile int helper_state;

static void blocked_receiver(void *arg)
{
	int fd = (int)(intptr_t)arg;
	char buf[4];

	helper_state = 1;
	assert(zsock_recv(fd, buf, sizeof(buf), 0) == 3);
	helper_state = 2;
	vTaskDelete(NULL);
}

static void closer(void *arg)
{
	int fd = (int)(intptr_t)arg;

	helper_state = 3;
	assert(zsock_close(fd) == 0);
	helper_state = 4;
	vTaskDelete(NULL);
}

static void test_dispatch(void)
{
	struct net_sockaddr_in a = {.sin_family = NET_AF_INET};
	net_socklen_t len = sizeof(a);
	struct net_iovec iov = {"x", 1};
	struct net_msghdr msg = {.msg_iov = &iov, .msg_iovlen = 1};
	UBaseType_t mine = uxTaskPriorityGet(NULL);
	char buf[8];
	int opt = 1;
	int full = zsock_socket(NET_AF_INET, NET_SOCK_STREAM, 0);
	int sparse = zsock_socket(NET_AF_INET, NET_SOCK_DGRAM, 0);
	struct fake *f = zvfs_get_fd_obj(full, NULL, 0);

	trace[0] = '\0';
	assert(full == 0 && sparse == 1 && f != NULL);
	/* The lock was handed to the implementation (ZFD_IOCTL_SET_LOCK). */
	assert(f->lock != NULL);

	/* Each call to its entry, with its arguments and result. */
	assert(zsock_shutdown(full, ZSOCK_SHUT_RD) == 0 && last_arg[0] == ZSOCK_SHUT_RD);
	assert(zsock_bind(full, (struct net_sockaddr *)&a, len) == 0 && last_ptr[0] == &a &&
	       last_arg[0] == (int)len);
	CHECK_FAIL(zsock_connect(full, (struct net_sockaddr *)&a, len), ECONNREFUSED);
	assert(last_ptr[0] == &a);
	assert(zsock_listen(full, 3) == 0 && last_arg[0] == 3);
	assert(zsock_accept(full, (struct net_sockaddr *)&a, &len) == 2 && last_ptr[0] == &a &&
	       last_ptr[1] == &len);
	assert(zsock_sendto(full, "abc", 3, ZSOCK_MSG_DONTWAIT, (struct net_sockaddr *)&a, len) ==
		       3 &&
	       last_arg[1] == ZSOCK_MSG_DONTWAIT && last_ptr[1] == &a && last_arg[2] == (int)len);
	assert(zsock_send(full, "ab", 2, 0) == 2 && last_ptr[1] == NULL && last_arg[2] == 0);
	assert(zsock_recvfrom(full, buf, sizeof(buf), ZSOCK_MSG_PEEK, (struct net_sockaddr *)&a,
			      &len) == 3 &&
	       last_ptr[0] == buf && last_ptr[1] == &a && last_ptr[2] == &len &&
	       last_arg[0] == (int)sizeof(buf) && last_arg[1] == ZSOCK_MSG_PEEK);
	assert(zsock_recv(full, buf, 2, 0) == 3 && last_ptr[1] == NULL && last_ptr[2] == NULL);
	assert(zsock_sendmsg(full, &msg, 0) == 7 && last_ptr[0] == &msg);
	assert(zsock_recvmsg(full, &msg, ZSOCK_MSG_DONTWAIT) == 5 && last_ptr[0] == &msg &&
	       last_arg[0] == ZSOCK_MSG_DONTWAIT);
	assert(zsock_getsockopt(full, ZSOCK_SOL_SOCKET, ZSOCK_SO_ERROR, &opt, &len) == 0 &&
	       last_arg[0] == ZSOCK_SOL_SOCKET && last_arg[1] == ZSOCK_SO_ERROR &&
	       last_ptr[0] == &opt && last_ptr[1] == &len);
	assert(zsock_setsockopt(full, NET_IPPROTO_TCP, ZSOCK_TCP_NODELAY, &opt, sizeof(opt)) == 0 &&
	       last_arg[0] == NET_IPPROTO_TCP && last_arg[2] == (int)sizeof(opt));
	assert(zsock_getpeername(full, (struct net_sockaddr *)&a, &len) == 0 && last_ptr[0] == &a);
	assert(zsock_getsockname(full, (struct net_sockaddr *)&a, &len) == 0 && last_ptr[0] == &a);
	trace_expect("full:shutdown full:bind full:connect full:listen full:accept full:socket "
		     "full:sendto full:sendto full:recvfrom full:recvfrom full:sendmsg "
		     "full:recvmsg full:getsockopt full:setsockopt full:getpeername "
		     "full:getsockname");

	/* fcntl: to the ioctl method under the lock, the flags passed. */
	assert(zsock_fcntl(full, F_GETFL) == 0);
	assert(zsock_fcntl(full, F_SETFL, ZVFS_O_NONBLOCK) == 0 && f->nonblocking);
	assert(zsock_fcntl(full, F_GETFL) == ZVFS_O_NONBLOCK);
	trace_expect("full:getfl full:setfl full:getfl");
	CHECK_FAIL(zsock_fcntl(sparse, F_GETFL), ENOTSUP);

	/* NULL entries: EOPNOTSUPP, shutdown ENOTSUP. */
	CHECK_FAIL(zsock_shutdown(sparse, ZSOCK_SHUT_RD), ENOTSUP);
	CHECK_FAIL(zsock_bind(sparse, (struct net_sockaddr *)&a, len), EOPNOTSUPP);
	CHECK_FAIL(zsock_connect(sparse, (struct net_sockaddr *)&a, len), EOPNOTSUPP);
	CHECK_FAIL(zsock_listen(sparse, 1), EOPNOTSUPP);
	CHECK_FAIL(zsock_accept(sparse, NULL, NULL), EOPNOTSUPP);
	CHECK_FAIL(zsock_send(sparse, "a", 1, 0), EOPNOTSUPP);
	CHECK_FAIL(zsock_sendmsg(sparse, &msg, 0), EOPNOTSUPP);
	CHECK_FAIL(zsock_recv(sparse, buf, 1, 0), EOPNOTSUPP);
	CHECK_FAIL(zsock_recvmsg(sparse, &msg, 0), EOPNOTSUPP);
	CHECK_FAIL(zsock_getsockopt(sparse, ZSOCK_SOL_SOCKET, ZSOCK_SO_ERROR, &opt, &len),
		   EOPNOTSUPP);
	CHECK_FAIL(zsock_setsockopt(sparse, ZSOCK_SOL_SOCKET, ZSOCK_SO_RCVTIMEO, &opt, sizeof(opt)),
		   EOPNOTSUPP);
	CHECK_FAIL(zsock_getpeername(sparse, (struct net_sockaddr *)&a, &len), EOPNOTSUPP);
	CHECK_FAIL(zsock_getsockname(sparse, (struct net_sockaddr *)&a, &len), EOPNOTSUPP);
	trace_expect("");

	/* A descriptor that is not open: EBADF for every call. */
	for (int i = 0; i < 3; i++) {
		static const int bad[] = {-1, 5, TABLE_SIZE};
		int fd = bad[i];

		CHECK_FAIL(zsock_close(fd), EBADF);
		CHECK_FAIL(zsock_shutdown(fd, 0), EBADF);
		CHECK_FAIL(zsock_bind(fd, (struct net_sockaddr *)&a, len), EBADF);
		CHECK_FAIL(zsock_connect(fd, (struct net_sockaddr *)&a, len), EBADF);
		CHECK_FAIL(zsock_listen(fd, 1), EBADF);
		CHECK_FAIL(zsock_accept(fd, NULL, NULL), EBADF);
		CHECK_FAIL(zsock_send(fd, "a", 1, 0), EBADF);
		CHECK_FAIL(zsock_sendmsg(fd, &msg, 0), EBADF);
		CHECK_FAIL(zsock_recv(fd, buf, 1, 0), EBADF);
		CHECK_FAIL(zsock_recvmsg(fd, &msg, 0), EBADF);
		CHECK_FAIL(zsock_getsockopt(fd, 0, 0, &opt, &len), EBADF);
		CHECK_FAIL(zsock_setsockopt(fd, 0, 0, &opt, sizeof(opt)), EBADF);
		CHECK_FAIL(zsock_getpeername(fd, (struct net_sockaddr *)&a, &len), EBADF);
		CHECK_FAIL(zsock_getsockname(fd, (struct net_sockaddr *)&a, &len), EBADF);
		CHECK_FAIL(zsock_fcntl(fd, F_GETFL), EBADF);
	}
	trace_expect("");

	/* close: a socket through close2() with its descriptor, then free. */
	assert(zsock_close(2) == 0);
	trace_expect("full:close");
	assert(fakes[2].closed_with_fd == 2);
	CHECK_FAIL(zsock_close(2), EBADF);

	/* The descriptor's lock: a close waits for a receive in progress. */
	k_sem_init(&recv_started, 0, 1);
	f->blocking_recv = true;
	helper_state = 0;
	assert(xTaskCreate(blocked_receiver, "receiver", HELPER_STACK / sizeof(StackType_t),
			   (void *)(intptr_t)full, mine + 1, NULL) == pdPASS);
	assert(k_sem_take(&recv_started, K_SECONDS(1)) == 0 && helper_state == 1);
	assert(xTaskCreate(closer, "closer", HELPER_STACK / sizeof(StackType_t),
			   (void *)(intptr_t)full, mine + 1, NULL) == pdPASS);
	k_msleep(50);
	assert(helper_state == 3); /* the closer waits */
	assert(zvfs_get_fd_obj(full, NULL, 0) == f);
	k_sem_give(&f->recv_go);
	k_msleep(50);
	assert(helper_state == 4);
	errno = 0;
	assert(zvfs_get_fd_obj(full, NULL, 0) == NULL && errno == EBADF);
	trace_expect("full:recvfrom full:close");
	close_all();
	printf("ok  dispatch\n");
}

/* poll -------------------------------------------------------------------------*/

static void giver(void *arg)
{
	struct fake *f = arg;

	k_msleep(30);
	k_sem_give(&f->readable);
	vTaskDelete(NULL);
}

static void test_poll(void)
{
	struct zsock_pollfd p[4];
	UBaseType_t mine = uxTaskPriorityGet(NULL);
	int full = zsock_socket(NET_AF_INET, NET_SOCK_STREAM, 0);
	int sparse = zsock_socket(NET_AF_INET, NET_SOCK_DGRAM, 0);
	struct fake *f = zvfs_get_fd_obj(full, NULL, 0);
	int64_t t0;

	trace[0] = '\0';

	/* An offloaded descriptor: the whole call to its POLL_OFFLOAD, with
	 * every descriptor (also those of another kind, a closed one and a
	 * negative one) and the timeout in ms. */
	p[0] = (struct zsock_pollfd){.fd = full, .events = ZSOCK_POLLIN};
	p[1] = (struct zsock_pollfd){.fd = sparse, .events = ZSOCK_POLLIN};
	p[2] = (struct zsock_pollfd){.fd = 7, .events = ZSOCK_POLLIN};
	p[3] = (struct zsock_pollfd){.fd = -1, .events = ZSOCK_POLLIN};
	assert(zsock_poll(p, 4, 1234) == 4 && offload_nfds == 4 && offload_timeout == 1234);
	assert(p[0].revents == ZSOCK_POLLIN && p[3].revents == 0);
	trace_expect("full:prepare sparse:prepare sparse:offload");
	assert(zsock_poll(&p[1], 1, -1) == 1 && offload_timeout == -1);
	assert(zsock_poll(&p[1], 1, 0) == 1 && offload_timeout == 0);
	trace_expect("sparse:prepare sparse:offload sparse:prepare sparse:offload");

	/* Another kind alone: its events in k_poll(), then POLL_UPDATE. */
	t0 = k_uptime_get();
	assert(zsock_poll(p, 1, 60) == 0 && p[0].revents == 0);
	assert(k_uptime_get() - t0 >= 59);
	trace_expect("full:prepare full:update");
	assert(xTaskCreate(giver, "giver", HELPER_STACK / sizeof(StackType_t), f, mine + 1, NULL) ==
	       pdPASS);
	t0 = k_uptime_get();
	assert(zsock_poll(p, 1, 1000) == 1 && p[0].revents == ZSOCK_POLLIN);
	assert(k_uptime_get() - t0 < 500);
	trace_expect("full:prepare full:update");
	k_sem_reset(&f->readable);

	/* With a closed and a negative descriptor: POLLNVAL counted, the
	 * negative one skipped. */
	p[1] = (struct zsock_pollfd){.fd = 7, .events = ZSOCK_POLLIN, .revents = 9};
	p[2] = (struct zsock_pollfd){.fd = -1, .events = ZSOCK_POLLIN, .revents = 9};
	assert(zsock_poll(p, 3, 0) == 1 && p[0].revents == 0 && p[1].revents == ZSOCK_POLLNVAL &&
	       p[2].revents == 0);
	trace_expect("full:prepare full:update");

	/* No event at all: the whole timeout (Zephyr's k_poll() with none). */
	t0 = k_uptime_get();
	assert(zsock_poll(&p[1], 2, 40) == 1 && p[1].revents == ZSOCK_POLLNVAL);
	assert(k_uptime_get() - t0 >= 39);

	/* -EALREADY: no wait; an error of POLL_PREPARE fails the call. */
	prepare_result = -EALREADY;
	t0 = k_uptime_get();
	assert(zsock_poll(p, 1, 1000) == 0);
	assert(k_uptime_get() - t0 < 500);
	trace_expect("full:prepare full:update");
	prepare_result = -ENOMEM;
	CHECK_FAIL(zsock_poll(p, 1, 1000), ENOMEM);
	trace_expect("full:prepare");
	prepare_result = 0;
	close_all();
	printf("ok  poll\n");
}

/* The resolver -------------------------------------------------------------------*/

static int dns_calls;
static struct zsock_addrinfo fake_result;

static int fake_getaddrinfo(const char *node, const char *service,
			    const struct zsock_addrinfo *hints, struct zsock_addrinfo **res)
{
	ARG_UNUSED(node);
	ARG_UNUSED(service);
	ARG_UNUSED(hints);
	dns_calls++;
	*res = &fake_result;
	return 0;
}

static void fake_freeaddrinfo(struct zsock_addrinfo *res)
{
	ARG_UNUSED(res);
	dns_calls += 100;
}

static void test_dns(void)
{
	static const struct socket_dns_offload ops = {
		.getaddrinfo = fake_getaddrinfo,
		.freeaddrinfo = fake_freeaddrinfo,
	};
	static const struct socket_dns_offload other = {0};
	struct zsock_addrinfo *res = NULL;

	assert(!socket_offload_dns_is_enabled());
	assert(zsock_getaddrinfo("example.org", NULL, NULL, &res) == EAI_FAIL && res == NULL);
	zsock_freeaddrinfo(NULL);
	socket_offload_dns_register(&ops);
	assert(socket_offload_dns_is_enabled());
	assert(zsock_getaddrinfo("example.org", NULL, NULL, &res) == 0 && res == &fake_result);
	zsock_freeaddrinfo(res);
	assert(dns_calls == 101);
	socket_offload_dns_enable(false);
	assert(zsock_getaddrinfo("x", NULL, NULL, &res) == EAI_FAIL);
	socket_offload_dns_enable(true);
	assert(socket_offload_dns_deregister(&other) == -EINVAL);
	assert(socket_offload_dns_deregister(&ops) == 0);
	assert(!socket_offload_dns_is_enabled());
	assert(zsock_getaddrinfo("x", NULL, NULL, &res) == EAI_FAIL && dns_calls == 101);
	printf("ok  dns\n");
}

/* k_heap -------------------------------------------------------------------------*/

K_HEAP_DEFINE(test_heap, 256);
K_HEAP_DEFINE(odd_heap, 100);

static void test_heap_bounds(void)
{
	void *a, *b, *c, *d, *e;
	void *all[64];
	int n;

	assert(odd_heap.size == 96);
	assert(k_heap_alloc(&test_heap, 0, K_NO_WAIT) == NULL);
	assert(k_heap_alloc(&test_heap, 257, K_NO_WAIT) == NULL);

	/* The whole buffer less one header, not a byte more. */
	assert(k_heap_alloc(&test_heap, 249, K_NO_WAIT) == NULL);
	a = k_heap_alloc(&test_heap, 248, K_NO_WAIT);
	assert(a != NULL && ((uintptr_t)a & 7) == 0);
	assert(k_heap_alloc(&test_heap, 1, K_FOREVER) == NULL); /* never waits */
	k_heap_free(&test_heap, a);
	k_heap_free(&test_heap, NULL);

	/* 8-byte blocks of 16 bytes: 16 of them. */
	for (n = 0; n < (int)ARRAY_SIZE(all); n++) {
		all[n] = k_heap_alloc(&test_heap, 5, K_NO_WAIT);
		if (all[n] == NULL) {
			break;
		}
		assert(((uintptr_t)all[n] & 7) == 0);
		memset(all[n], 0xa5, 5);
	}
	assert(n == 16);
	for (int i = 0; i < n; i++) {
		k_heap_free(&test_heap, all[i]);
	}

	/* Freed blocks are joined again: 72 + 72 + 72 + 40 bytes, a hole in
	 * the middle is too small for 100, the hole with the first one is not.
	 */
	a = k_heap_alloc(&test_heap, 64, K_NO_WAIT);
	b = k_heap_alloc(&test_heap, 64, K_NO_WAIT);
	c = k_heap_alloc(&test_heap, 64, K_NO_WAIT);
	assert(a && b && c && (uint8_t *)b - (uint8_t *)a == 72 &&
	       (uint8_t *)c - (uint8_t *)b == 72);
	assert(k_heap_alloc(&test_heap, 40, K_NO_WAIT) == NULL);
	d = k_heap_alloc(&test_heap, 32, K_NO_WAIT);
	assert(d != NULL);
	k_heap_free(&test_heap, b);
	assert(k_heap_alloc(&test_heap, 100, K_NO_WAIT) == NULL);
	k_heap_free(&test_heap, a);
	e = k_heap_alloc(&test_heap, 100, K_NO_WAIT);
	assert(e == a);
	k_heap_free(&test_heap, e);
	k_heap_free(&test_heap, d);
	k_heap_free(&test_heap, c);
	assert((a = k_heap_alloc(&test_heap, 248, K_NO_WAIT)) != NULL);
	k_heap_free(&test_heap, a);
	printf("ok  k_heap\n");
}

static void runner(void *arg)
{
	(void)arg;
	test_fdtable();
	test_registrations();
	test_dispatch();
	test_poll();
	test_dns();
	test_heap_bounds();
	fflush(stdout);
	_exit(0);
}

int main(void)
{
	/* printf() must not allocate in a task (the port can switch threads
	 * while the C library holds its locks).
	 */
	static char out[1 << 12];

	setvbuf(stdout, out, _IOLBF, sizeof(out));
	if (zephyr_compat_initialize() != 0) {
		return 2;
	}
	if (xTaskCreate(runner, "runner", RUNNER_STACK / sizeof(StackType_t), NULL,
			NANO_PRIO_FROM_ZEPHYR(9), NULL) != pdPASS) {
		return 2;
	}
	vTaskStartScheduler();
	return 3;
}
