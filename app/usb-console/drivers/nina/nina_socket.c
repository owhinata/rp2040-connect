/* SPDX-License-Identifier: Apache-2.0 */
#include <errno.h>
#include <stdarg.h>
#include <string.h>
#include <zephyr/kernel.h>
#include <zephyr/net/socket.h>
#include <zephyr/sys/fdtable.h>
#include "nina_proto.h"

#define MAX_NINA_SOCKETS 4
#define NINA_MAX_IO      512
struct nina_sock {
	bool used;
	bool connected;
	uint8_t id;
};
static struct nina_sock sockets[MAX_NINA_SOCKETS];
K_MUTEX_DEFINE(socket_lock);
static const struct socket_op_vtable socket_ops;

static int cmd1(uint8_t cmd, uint8_t id, uint8_t *data, size_t cap, size_t *n)
{
	struct nina_param p = {&id, 1};

	return nina_request(cmd, &p, 1, false, false, data, cap, n);
}

static int sock_state(struct nina_sock *s, uint8_t *state)
{
	size_t n;
	int ret = cmd1(0x2f, s->id, state, 1, &n);

	return ret ? ret : (n == 1 ? 0 : -EPROTO);
}

static int available(struct nina_sock *s, unsigned int *count)
{
	uint8_t data[2];
	size_t n;
	int ret = cmd1(0x2b, s->id, data, sizeof(data), &n);

	if (ret) {
		return ret;
	}
	if (n != 2) {
		return -EPROTO;
	}
	*count = data[0] | ((unsigned int)data[1] << 8);
	return 0;
}

static int fail(int err)
{
	errno = err > 0 ? err : -err;
	return -1;
}

static int do_connect(void *obj, const struct net_sockaddr *addr,
		      net_socklen_t addrlen)
{
	struct nina_sock *s = obj;
	const struct net_sockaddr_in *in = (const struct net_sockaddr_in *)addr;
	uint8_t result, mode = 0;
	size_t n;
	struct nina_param p[4];
	int ret;

	if (!addr || addrlen < sizeof(*in) || addr->sa_family != NET_AF_INET) {
		return fail(EINVAL);
	}
	if (s->connected) {
		return fail(EISCONN);
	}

	p[0] = (struct nina_param){(const uint8_t *)&in->sin_addr, 4};
	p[1] = (struct nina_param){(const uint8_t *)&in->sin_port, 2};
	p[2] = (struct nina_param){&s->id, 1};
	p[3] = (struct nina_param){&mode, 1};
	ret = nina_request(0x2d, p, 4, false, false, &result, 1, &n);
	if (ret) {
		return fail(ret);
	}
	if (n != 1 || result != 1) {
		return fail(ECONNREFUSED);
	}
	for (int i = 0; i < 100; i++) {
		ret = sock_state(s, &result);
		if (ret) {
			return fail(ret);
		}
		if (result == 4) {
			s->connected = true;
			return 0;
		}
		if (result == 0) {
			return fail(ECONNREFUSED);
		}
		k_sleep(K_MSEC(100));
	}
	return fail(ETIMEDOUT);
}

static ssize_t do_send(void *obj, const void *buf, size_t len, int flags,
		       const struct net_sockaddr *to, net_socklen_t tolen)
{
	struct nina_sock *s = obj;
	uint8_t answer[2];
	size_t n;
	int ret;
	struct nina_param p[2];

	ARG_UNUSED(tolen);
	if (to || flags) {
		return fail(EOPNOTSUPP);
	}
	if (!s->connected) {
		return fail(ENOTCONN);
	}
	if (!len) {
		return 0;
	}
	if (!buf) {
		return fail(EINVAL);
	}

	len = MIN(len, NINA_MAX_IO);
	p[0] = (struct nina_param){&s->id, 1};
	p[1] = (struct nina_param){buf, len};
	ret = nina_request(0x44, p, 2, true, false, answer, sizeof(answer), &n);
	if (ret) {
		return fail(ret);
	}
	if (n != 2) {
		return fail(EPROTO);
	}
	unsigned int written = answer[0] | ((unsigned int)answer[1] << 8);
	if (written > len) {
		return fail(EPROTO);
	}
	return written ? written : fail(EIO);
}

static ssize_t do_sendmsg(void *obj, const struct net_msghdr *msg, int flags)
{
	if (!msg || (msg->msg_iovlen && !msg->msg_iov)) {
		return fail(EINVAL);
	}
	if (msg->msg_name || msg->msg_controllen) {
		return fail(EOPNOTSUPP);
	}
	/* Returning one vector at a time is a valid partial TCP write. */
	for (size_t i = 0; i < msg->msg_iovlen; i++) {
		if (msg->msg_iov[i].iov_len) {
			return do_send(obj, msg->msg_iov[i].iov_base,
				       msg->msg_iov[i].iov_len, flags, NULL, 0);
		}
	}
	return 0;
}

static ssize_t do_recv(void *obj, void *buf, size_t len, int flags,
		       struct net_sockaddr *from, net_socklen_t *fromlen)
{
	struct nina_sock *s = obj;
	uint8_t state, bytes[2];
	size_t n;
	unsigned int count;
	int ret;
	struct nina_param p[2];

	if (from || fromlen || (flags & ~ZSOCK_MSG_DONTWAIT)) {
		return fail(EOPNOTSUPP);
	}
	if (!s->connected) {
		return fail(ENOTCONN);
	}
	if (!len) {
		return 0;
	}
	if (!buf) {
		return fail(EINVAL);
	}

	for (;;) {
		ret = available(s, &count);
		if (ret) {
			return fail(ret);
		}
		if (count) {
			break;
		}
		ret = sock_state(s, &state);
		if (ret) {
			return fail(ret);
		}
		if (state != 4) {
			s->connected = false;
			return 0;
		}
		if (flags & ZSOCK_MSG_DONTWAIT) {
			return fail(EAGAIN);
		}
		k_sleep(K_MSEC(20));
	}

	len = MIN(MIN(len, count), NINA_MAX_IO);
	bytes[0] = len & 0xff;
	bytes[1] = len >> 8;
	p[0] = (struct nina_param){&s->id, 1};
	p[1] = (struct nina_param){bytes, 2};
	ret = nina_request(0x45, p, 2, true, true, buf, len, &n);
	if (ret) {
		return fail(ret);
	}
	return n;
}

static int do_close(void *obj)
{
	struct nina_sock *s = obj;
	uint8_t result;
	size_t n;
	int ret = 0;

	if (s->connected) {
		ret = cmd1(0x2e, s->id, &result, 1, &n);
	}
	k_mutex_lock(&socket_lock, K_FOREVER);
	s->used = false;
	s->connected = false;
	k_mutex_unlock(&socket_lock);
	return ret ? fail(ret) : 0;
}

static ssize_t do_read(void *obj, void *buf, size_t count)
{
	return do_recv(obj, buf, count, 0, NULL, NULL);
}

static ssize_t do_write(void *obj, const void *buf, size_t count)
{
	return do_send(obj, buf, count, 0, NULL, 0);
}

static int do_poll(struct zsock_pollfd *fds, int nfds, int timeout)
{
	int64_t deadline = timeout < 0 ? INT64_MAX : k_uptime_get() + timeout;

	if (nfds != 1) {
		return fail(ENOTSUP);
	}
	struct nina_sock *s = zvfs_get_fd_obj(
		fds[0].fd, (const struct fd_op_vtable *)&socket_ops, 0);
	if (!s) {
		return fail(EBADF);
	}

	for (;;) {
		unsigned int count;
		uint8_t state;
		int ret;
		fds[0].revents = 0;
		if ((fds[0].events & ZSOCK_POLLOUT) && s->connected) {
			fds[0].revents |= ZSOCK_POLLOUT;
		}
		if (fds[0].events & ZSOCK_POLLIN) {
			ret = available(s, &count);
			if (ret) {
				return fail(ret);
			}
			if (count) {
				fds[0].revents |= ZSOCK_POLLIN;
			} else {
				ret = sock_state(s, &state);
				if (ret) {
					return fail(ret);
				}
				if (state != 4) {
					fds[0].revents |= ZSOCK_POLLHUP;
				}
			}
		}
		if (fds[0].revents) {
			return 1;
		}
		if (k_uptime_get() >= deadline) {
			return 0;
		}
		k_sleep(K_MSEC(20));
	}
}

static int do_ioctl(void *obj, unsigned int request, va_list args)
{
	ARG_UNUSED(obj);
	switch (request) {
	case ZFD_IOCTL_POLL_PREPARE:
		return -EXDEV;
	case ZFD_IOCTL_POLL_OFFLOAD: {
		struct zsock_pollfd *fds = va_arg(args, struct zsock_pollfd *);
		int nfds = va_arg(args, int);
		int timeout = va_arg(args, int);
		return do_poll(fds, nfds, timeout);
	}
	default:
		return fail(ENOTSUP);
	}
}
static const struct socket_op_vtable socket_ops = {
	.fd_vtable = {.read = do_read,
		      .write = do_write,
		      .close = do_close,
		      .ioctl = do_ioctl},
	.connect = do_connect,
	.sendto = do_send,
	.sendmsg = do_sendmsg,
	.recvfrom = do_recv,
};
static bool supported(int family, int type, int proto)
{
	return family == NET_AF_INET && type == NET_SOCK_STREAM &&
	       (proto == 0 || proto == NET_IPPROTO_TCP);
}

int nina_socket_create(int family, int type, int proto)
{
	uint8_t id;
	size_t n;
	int ret, fd;
	struct nina_sock *s = NULL;

	if (!supported(family, type, proto)) {
		return fail(EAFNOSUPPORT);
	}
	fd = zvfs_reserve_fd();
	if (fd < 0) {
		return -1;
	}
	k_mutex_lock(&socket_lock, K_FOREVER);
	for (int i = 0; i < MAX_NINA_SOCKETS; i++) {
		if (!sockets[i].used) {
			s = &sockets[i];
			break;
		}
	}
	if (!s) {
		k_mutex_unlock(&socket_lock);
		zvfs_free_fd(fd);
		return fail(EMFILE);
	}
	ret = nina_request(0x3f, NULL, 0, false, false, &id, 1, &n);
	if (ret || n != 1 || id == 255) {
		k_mutex_unlock(&socket_lock);
		zvfs_free_fd(fd);
		return fail(ret ? ret : ENFILE);
	}
	/* Firmware does not reserve an ID until connect: reject reuse. */
	for (int i = 0; i < MAX_NINA_SOCKETS; i++) {
		if (sockets[i].used && sockets[i].id == id) {
			k_mutex_unlock(&socket_lock);
			zvfs_free_fd(fd);
			return fail(EBUSY);
		}
	}
	s->id = id;
	s->used = true;
	s->connected = false;
	k_mutex_unlock(&socket_lock);
	zvfs_finalize_typed_fd(fd, s, (const struct fd_op_vtable *)&socket_ops,
			       ZVFS_MODE_IFSOCK);
	return fd;
}
NET_SOCKET_OFFLOAD_REGISTER(nina, CONFIG_NET_SOCKETS_OFFLOAD_PRIORITY,
			    NET_AF_INET, supported, nina_socket_create);
