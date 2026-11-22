/*
 * Copyright (c) 2017 Linaro Limited
 * Copyright (c) 2021 Nordic Semiconductor
 * Copyright (c) 2023 Arm Limited (or its affiliates). All rights reserved.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/* Zephyr's subsys/net/lib/sockets/sockets.c (Zephyr v4.5.0), reduced to the
 * socket layer above the list of socket implementations (see
 * <zephyr/net/socket.h> here): VTABLE_CALL(), get_sock_vtable(), and the
 * z_impl_zsock_*() functions of socket, close, shutdown, bind, connect,
 * listen, accept, sendto, sendmsg, recvfrom, recvmsg, fcntl, getsockopt,
 * setsockopt, getpeername and getsockname, with Zephyr's text.  Without user
 * mode, Zephyr's zsock_*() is a syscall wrapper that only calls
 * z_impl_zsock_*(); here they are one function under the zsock_*() name.
 * zsock_poll() is the inline function of Zephyr's <zephyr/net/socket.h>
 * (zvfs_poll()), here a function, so that a host test can replace the
 * calls one by one.
 *
 * Left out: the user mode (the z_vrfy_*() functions, the object checks of
 * get_sock_vtable()), tracing (SYS_PORT_TRACING_*(), empty here), the
 * object core statistics (sock_obj_core_*(), empty as in Zephyr without
 * CONFIG_NET_SOCKETS_OBJ_CORE), the offloaded port tracking (Zephyr's
 * net_socket_offloaded_port_*(), empty as without
 * CONFIG_NET_SOCKETS_OFFLOAD_PORT_TRACKING), the debug messages,
 * zsock_get_context_object(), zsock_ioctl_impl(), zsock_inet_pton() (the C
 * library's inet_pton()), net_socket_port_in_use(), msghdr_non_empty_iov_count()
 * and zsock_send_all()/zsock_sendmsg_all() (nothing uses them).
 * Part of the network part of the Zephyr compatibility layer for an OS
 * without a network stack of its own (common/compat/net_offload), which the
 * FreeRTOS build uses.
 */

#include <errno.h>

#include <zephyr/kernel.h>
#include <zephyr/net/socket.h>
#include <zephyr/sys/__assert.h>

#define SYS_PORT_TRACING_OBJ_INIT(...)
#define SYS_PORT_TRACING_OBJ_FUNC_ENTER(...)
#define SYS_PORT_TRACING_OBJ_FUNC_EXIT(...)
#define NET_ASSERT(cond)                            __ASSERT_NO_MSG(cond)
#define sock_obj_core_alloc(...)                    0
#define sock_obj_core_alloc_find(...)               0
#define sock_obj_core_update_send_stats(sock, n)    ARG_UNUSED(n)
#define sock_obj_core_update_recv_stats(sock, n)    ARG_UNUSED(n)
#define net_socket_offloaded_port_bind(sock, addr)  ((void)0)
#define net_socket_offloaded_port_untrack(fd)       ARG_UNUSED(fd)

/* STRUCT_SECTION_FOREACH(net_socket_register, ...): the entries the linker
 * script collected, in the order of their names (see <zephyr/net/socket.h>).
 */
extern const struct net_socket_register _net_socket_register_list_start[];
extern const struct net_socket_register _net_socket_register_list_end[];

#define VTABLE_CALL(fn, sock, ...)			     \
	({						     \
		const struct socket_op_vtable *vtable;	     \
		struct k_mutex *lock;			     \
		void *obj;				     \
		int retval;				     \
							     \
		obj = get_sock_vtable(sock, &vtable, &lock); \
		if (obj == NULL) {			     \
			errno = EBADF;			     \
			return -1;			     \
		}					     \
							     \
		if (vtable->fn == NULL) {		     \
			errno = EOPNOTSUPP;		     \
			return -1;			     \
		}					     \
							     \
		(void)k_mutex_lock(lock, K_FOREVER);         \
							     \
		retval = vtable->fn(obj, __VA_ARGS__);	     \
							     \
		k_mutex_unlock(lock);                        \
							     \
		retval;					     \
	})

static inline void *get_sock_vtable(int sock,
				    const struct socket_op_vtable **vtable,
				    struct k_mutex **lock)
{
	void *ctx;

	ctx = zvfs_get_fd_obj_and_vtable(sock,
				      (const struct fd_op_vtable **)vtable,
				      lock);

	return ctx;
}

int zsock_socket(int family, int type, int proto)
{
	for (const struct net_socket_register *sock_family = _net_socket_register_list_start;
	     sock_family < _net_socket_register_list_end; sock_family++) {
		int ret;

		if (sock_family->family != family &&
		    sock_family->family != NET_AF_UNSPEC) {
			continue;
		}

		NET_ASSERT(sock_family->is_supported);

		if (!sock_family->is_supported(family, type, proto)) {
			continue;
		}

		errno = 0;
		ret = sock_family->handler(family, type, proto);

		SYS_PORT_TRACING_OBJ_INIT(socket, ret < 0 ? -errno : ret,
					  family, type, proto);

		(void)sock_obj_core_alloc(ret, sock_family, family, type, proto);

		return ret;
	}

	errno = EAFNOSUPPORT;
	SYS_PORT_TRACING_OBJ_INIT(socket, -errno, family, type, proto);
	return -1;
}

int zsock_close(int sock)
{
	return zvfs_close(sock);
}

/*
 * Runs on every close path, even when zsock_close() is bypassed.
 */
void zvfs_socket_close_hook(int fd)
{
	net_socket_offloaded_port_untrack(fd);
}

int zsock_shutdown(int sock, int how)
{
	const struct socket_op_vtable *vtable;
	struct k_mutex *lock;
	void *ctx;
	int ret;

	SYS_PORT_TRACING_OBJ_FUNC_ENTER(socket, shutdown, sock, how);

	ctx = get_sock_vtable(sock, &vtable, &lock);
	if (ctx == NULL) {
		errno = EBADF;
		SYS_PORT_TRACING_OBJ_FUNC_EXIT(socket, shutdown, sock, -errno);
		return -1;
	}

	if (!vtable->shutdown) {
		errno = ENOTSUP;
		SYS_PORT_TRACING_OBJ_FUNC_EXIT(socket, shutdown, sock, -errno);
		return -1;
	}

	(void)k_mutex_lock(lock, K_FOREVER);

	ret = vtable->shutdown(ctx, how);

	k_mutex_unlock(lock);

	SYS_PORT_TRACING_OBJ_FUNC_EXIT(socket, shutdown, sock, ret < 0 ? -errno : ret);

	return ret;
}

int zsock_bind(int sock, const struct net_sockaddr *addr, net_socklen_t addrlen)
{
	int ret;

	SYS_PORT_TRACING_OBJ_FUNC_ENTER(socket, bind, sock, addr, addrlen);

	ret = VTABLE_CALL(bind, sock, addr, addrlen);
	if (ret == 0) {
		net_socket_offloaded_port_bind(sock, addr);
	}

	SYS_PORT_TRACING_OBJ_FUNC_EXIT(socket, bind, sock, ret < 0 ? -errno : ret);

	return ret;
}

int zsock_connect(int sock, const struct net_sockaddr *addr,
			net_socklen_t addrlen)
{
	int ret;

	SYS_PORT_TRACING_OBJ_FUNC_ENTER(socket, connect, sock, addr, addrlen);

	ret = VTABLE_CALL(connect, sock, addr, addrlen);

	SYS_PORT_TRACING_OBJ_FUNC_EXIT(socket, connect, sock,
				       ret < 0 ? -errno : ret);
	return ret;
}

int zsock_listen(int sock, int backlog)
{
	int ret;

	SYS_PORT_TRACING_OBJ_FUNC_ENTER(socket, listen, sock, backlog);

	ret = VTABLE_CALL(listen, sock, backlog);

	SYS_PORT_TRACING_OBJ_FUNC_EXIT(socket, listen, sock,
				       ret < 0 ? -errno : ret);
	return ret;
}

int zsock_accept(int sock, struct net_sockaddr *addr, net_socklen_t *addrlen)
{
	int new_sock;

	SYS_PORT_TRACING_OBJ_FUNC_ENTER(socket, accept, sock);

	new_sock = VTABLE_CALL(accept, sock, addr, addrlen);

	SYS_PORT_TRACING_OBJ_FUNC_EXIT(socket, accept, new_sock, addr, addrlen,
				       new_sock < 0 ? -errno : 0);

	(void)sock_obj_core_alloc_find(sock, new_sock, NET_SOCK_STREAM);

	return new_sock;
}

ssize_t zsock_sendto(int sock, const void *buf, size_t len, int flags,
			   const struct net_sockaddr *dest_addr, net_socklen_t addrlen)
{
	int bytes_sent;

	SYS_PORT_TRACING_OBJ_FUNC_ENTER(socket, sendto, sock, len, flags,
					dest_addr, addrlen);

	bytes_sent = VTABLE_CALL(sendto, sock, buf, len, flags, dest_addr, addrlen);

	SYS_PORT_TRACING_OBJ_FUNC_EXIT(socket, sendto, sock,
				       bytes_sent < 0 ? -errno : bytes_sent);

	sock_obj_core_update_send_stats(sock, bytes_sent);

	return bytes_sent;
}

ssize_t zsock_sendmsg(int sock, const struct net_msghdr *msg, int flags)
{
	int bytes_sent;

	SYS_PORT_TRACING_OBJ_FUNC_ENTER(socket, sendmsg, sock, msg, flags);

	bytes_sent = VTABLE_CALL(sendmsg, sock, msg, flags);

	SYS_PORT_TRACING_OBJ_FUNC_EXIT(socket, sendmsg, sock,
				       bytes_sent < 0 ? -errno : bytes_sent);

	sock_obj_core_update_send_stats(sock, bytes_sent);

	return bytes_sent;
}

ssize_t zsock_recvfrom(int sock, void *buf, size_t max_len, int flags,
			     struct net_sockaddr *src_addr, net_socklen_t *addrlen)
{
	int bytes_received;

	SYS_PORT_TRACING_OBJ_FUNC_ENTER(socket, recvfrom, sock, max_len, flags, src_addr, addrlen);

	bytes_received = VTABLE_CALL(recvfrom, sock, buf, max_len, flags, src_addr, addrlen);

	SYS_PORT_TRACING_OBJ_FUNC_EXIT(socket, recvfrom, sock,
				       src_addr, addrlen,
				       bytes_received < 0 ? -errno : bytes_received);

	sock_obj_core_update_recv_stats(sock, bytes_received);

	return bytes_received;
}

ssize_t zsock_recvmsg(int sock, struct net_msghdr *msg, int flags)
{
	int bytes_received;

	SYS_PORT_TRACING_OBJ_FUNC_ENTER(socket, recvmsg, sock, msg, flags);

	bytes_received = VTABLE_CALL(recvmsg, sock, msg, flags);

	SYS_PORT_TRACING_OBJ_FUNC_EXIT(socket, recvmsg, sock, msg,
				       bytes_received < 0 ? -errno : bytes_received);

	sock_obj_core_update_recv_stats(sock, bytes_received);

	return bytes_received;
}

/* As this is limited function, we don't follow POSIX signature, with
 * "..." instead of last arg.
 */
int zsock_fcntl_impl(int sock, int cmd, int flags)
{
	const struct socket_op_vtable *vtable;
	struct k_mutex *lock;
	void *obj;
	int ret;

	SYS_PORT_TRACING_OBJ_FUNC_ENTER(socket, fcntl, sock, cmd, flags);

	obj = get_sock_vtable(sock, &vtable, &lock);
	if (obj == NULL) {
		errno = EBADF;
		SYS_PORT_TRACING_OBJ_FUNC_EXIT(socket, fcntl, sock, -errno);
		return -1;
	}

	(void)k_mutex_lock(lock, K_FOREVER);

	ret = zvfs_fdtable_call_ioctl((const struct fd_op_vtable *)vtable,
				   obj, cmd, flags);

	k_mutex_unlock(lock);

	SYS_PORT_TRACING_OBJ_FUNC_EXIT(socket, fcntl, sock,
				       ret < 0 ? -errno : ret);
	return ret;
}

int zsock_poll(struct zsock_pollfd *fds, int nfds, int timeout)
{
	return zvfs_poll(fds, nfds, timeout);
}

int zsock_getsockopt(int sock, int level, int optname,
			    void *optval, net_socklen_t *optlen)
{
	int ret;

	SYS_PORT_TRACING_OBJ_FUNC_ENTER(socket, getsockopt, sock, level, optname);

	ret = VTABLE_CALL(getsockopt, sock, level, optname, optval, optlen);

	SYS_PORT_TRACING_OBJ_FUNC_EXIT(socket, getsockopt, sock, level, optname,
				       optval, *optlen, ret < 0 ? -errno : ret);
	return ret;
}

int zsock_setsockopt(int sock, int level, int optname,
			    const void *optval, net_socklen_t optlen)
{
	int ret;

	SYS_PORT_TRACING_OBJ_FUNC_ENTER(socket, setsockopt, sock,
					level, optname, optval, optlen);

	ret = VTABLE_CALL(setsockopt, sock, level, optname, optval, optlen);

	SYS_PORT_TRACING_OBJ_FUNC_EXIT(socket, setsockopt, sock,
				       ret < 0 ? -errno : ret);
	return ret;
}

int zsock_getpeername(int sock, struct net_sockaddr *addr,
			     net_socklen_t *addrlen)
{
	int ret;

	SYS_PORT_TRACING_OBJ_FUNC_ENTER(socket, getpeername, sock);

	ret = VTABLE_CALL(getpeername, sock, addr, addrlen);

	SYS_PORT_TRACING_OBJ_FUNC_EXIT(socket, getpeername, sock,
				       addr, addrlen,
				       ret < 0 ? -errno : ret);
	return ret;
}

int zsock_getsockname(int sock, struct net_sockaddr *addr,
			     net_socklen_t *addrlen)
{
	int ret;

	SYS_PORT_TRACING_OBJ_FUNC_ENTER(socket, getsockname, sock);

	ret = VTABLE_CALL(getsockname, sock, addr, addrlen);

	SYS_PORT_TRACING_OBJ_FUNC_EXIT(socket, getsockname, sock,
				       addr, addrlen,
				       ret < 0 ? -errno : ret);
	return ret;
}
