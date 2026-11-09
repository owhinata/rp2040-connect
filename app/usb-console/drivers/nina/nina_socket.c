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
#define NINA_NO_SOCKET   UINT8_MAX
struct nina_sock {
	bool used;
	bool connected;
	bool udp;
	bool bound;
	uint8_t id;
	struct net_sockaddr_in peer;
};
static struct nina_sock sockets[MAX_NINA_SOCKETS];
K_MUTEX_DEFINE(socket_lock);
K_MUTEX_DEFINE(udp_lock);
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

/* Caller holds socket_lock until the firmware has reserved the ID. */
static int reserve_id(struct nina_sock *s)
{
	size_t n;
	int ret;

	/* NINA 1.4.8 can starve UDP while another socket has unread data. */
	for (int i = 0; i < MAX_NINA_SOCKETS; i++) {
		if (&sockets[i] != s && sockets[i].used &&
		    sockets[i].id != NINA_NO_SOCKET &&
		    (s->udp || sockets[i].udp)) {
			return -EOPNOTSUPP;
		}
	}
	ret = nina_request(0x3f, NULL, 0, false, false, &s->id, 1, &n);

	if (ret || n != 1 || s->id == NINA_NO_SOCKET) {
		s->id = NINA_NO_SOCKET;
		return ret ? ret : -ENFILE;
	}
	for (int i = 0; i < MAX_NINA_SOCKETS; i++) {
		if (&sockets[i] != s && sockets[i].used &&
		    sockets[i].id == s->id) {
			s->id = NINA_NO_SOCKET;
			return -EBUSY;
		}
	}
	return 0;
}

static int udp_bind_locked(struct nina_sock *s, uint16_t port)
{
	uint8_t mode = 1, result;
	size_t n;
	struct nina_param p[3];
	int ret;

	if (s->bound) {
		return -EINVAL;
	}
	ret = reserve_id(s);
	if (ret) {
		return ret;
	}
	p[0] = (struct nina_param){(const uint8_t *)&port, 2};
	p[1] = (struct nina_param){&s->id, 1};
	p[2] = (struct nina_param){&mode, 1};
	ret = nina_request(0x28, p, 3, false, false, &result, 1, &n);
	if (ret || n != 1 || result != 1) {
		s->id = NINA_NO_SOCKET;
		return ret ? ret : -EADDRINUSE;
	}
	s->bound = true;
	return 0;
}

static int do_bind(void *obj, const struct net_sockaddr *addr,
		   net_socklen_t addrlen)
{
	struct nina_sock *s = obj;
	const struct net_sockaddr_in *in = (const struct net_sockaddr_in *)addr;
	int ret;

	if (!s->udp) {
		return fail(EOPNOTSUPP);
	}
	if (!addr || addrlen < sizeof(*in) || addr->sa_family != NET_AF_INET) {
		return fail(EINVAL);
	}
	if (in->sin_addr.s_addr != 0) {
		return fail(EADDRNOTAVAIL);
	}
	k_mutex_lock(&udp_lock, K_FOREVER);
	k_mutex_lock(&socket_lock, K_FOREVER);
	ret = udp_bind_locked(s, in->sin_port);
	k_mutex_unlock(&socket_lock);
	k_mutex_unlock(&udp_lock);
	return ret ? fail(ret) : 0;
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
	if (s->udp) {
		k_mutex_lock(&udp_lock, K_FOREVER);
		k_mutex_lock(&socket_lock, K_FOREVER);
		ret = s->bound ? 0 : udp_bind_locked(s, 0);
		if (!ret) {
			s->peer = *in;
			s->connected = true;
		}
		k_mutex_unlock(&socket_lock);
		k_mutex_unlock(&udp_lock);
		return ret ? fail(ret) : 0;
	}
	if (s->connected) {
		return fail(EISCONN);
	}

	/* The firmware reserves an ID only when startClient succeeds. */
	k_mutex_lock(&socket_lock, K_FOREVER);
	if (s->id != NINA_NO_SOCKET) {
		k_mutex_unlock(&socket_lock);
		return fail(EALREADY);
	}
	ret = reserve_id(s);
	if (ret) {
		k_mutex_unlock(&socket_lock);
		return fail(ret);
	}
	p[0] = (struct nina_param){(const uint8_t *)&in->sin_addr, 4};
	p[1] = (struct nina_param){(const uint8_t *)&in->sin_port, 2};
	p[2] = (struct nina_param){&s->id, 1};
	p[3] = (struct nina_param){&mode, 1};
	ret = nina_request(0x2d, p, 4, false, false, &result, 1, &n);
	if (ret) {
		s->id = NINA_NO_SOCKET;
		k_mutex_unlock(&socket_lock);
		return fail(ret);
	}
	if (n != 1 || result != 1) {
		s->id = NINA_NO_SOCKET;
		k_mutex_unlock(&socket_lock);
		return fail(ECONNREFUSED);
	}
	s->connected = true;
	k_mutex_unlock(&socket_lock);
	for (int i = 0; i < 100; i++) {
		ret = sock_state(s, &result);
		if (ret) {
			return fail(ret);
		}
		if (result == 4) {
			return 0;
		}
		if (result == 0) {
			return fail(ECONNREFUSED);
		}
		k_sleep(K_MSEC(100));
	}
	return fail(ETIMEDOUT);
}

static ssize_t udp_send(struct nina_sock *s, const void *buf, size_t len,
			int flags, const struct net_sockaddr *to,
			net_socklen_t tolen)
{
	const struct net_sockaddr_in *peer = (const struct net_sockaddr_in *)to;
	struct nina_param p[4];
	uint8_t mode = 1, answer;
	size_t n;
	int ret;

	if (flags || len > NINA_MAX_IO) {
		return fail(flags ? EOPNOTSUPP : EMSGSIZE);
	}
	if (len && !buf) {
		return fail(EINVAL);
	}
	if (to && (tolen < sizeof(*peer) || to->sa_family != NET_AF_INET)) {
		return fail(EINVAL);
	}
	if (!to) {
		if (!s->connected) {
			return fail(EDESTADDRREQ);
		}
		peer = &s->peer;
	}
	k_mutex_lock(&udp_lock, K_FOREVER);
	k_mutex_lock(&socket_lock, K_FOREVER);
	ret = s->bound ? 0 : udp_bind_locked(s, 0);
	k_mutex_unlock(&socket_lock);
	if (ret) {
		goto done;
	}
	p[0] = (struct nina_param){(const uint8_t *)&peer->sin_addr, 4};
	p[1] = (struct nina_param){(const uint8_t *)&peer->sin_port, 2};
	p[2] = (struct nina_param){&s->id, 1};
	p[3] = (struct nina_param){&mode, 1};
	ret = nina_request(0x2d, p, 4, false, false, &answer, 1, &n);
	if (ret || n != 1 || answer != 1) {
		ret = ret ? ret : -EIO;
		goto done;
	}
	p[0] = (struct nina_param){&s->id, 1};
	p[1] = (struct nina_param){buf, len};
	if (len) {
		ret = nina_request(0x46, p, 2, true, false, &answer, 1, &n);
		if (ret || n != 1 || answer != 1) {
			ret = ret ? ret : -EIO;
			goto done;
		}
	}
	ret = cmd1(0x39, s->id, &answer, 1, &n);
	if (!ret && (n != 1 || answer != 1)) {
		ret = -EIO;
	}
done:
	k_mutex_unlock(&udp_lock);
	return ret ? fail(ret) : (ssize_t)len;
}

static ssize_t do_send(void *obj, const void *buf, size_t len, int flags,
		       const struct net_sockaddr *to, net_socklen_t tolen)
{
	struct nina_sock *s = obj;
	uint8_t answer[2];
	size_t n;
	int ret;
	struct nina_param p[2];

	if (s->udp) {
		return udp_send(s, buf, len, flags, to, tolen);
	}
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
	struct nina_sock *s = obj;

	if (!msg || (msg->msg_iovlen && !msg->msg_iov)) {
		return fail(EINVAL);
	}
	if (msg->msg_name || msg->msg_controllen) {
		return fail(EOPNOTSUPP);
	}
	if (s->udp) {
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

static ssize_t udp_recv(struct nina_sock *s, void *buf, size_t len, int flags,
			struct net_sockaddr *from, net_socklen_t *fromlen)
{
	uint8_t remote[6], length[2], discard[NINA_MAX_IO];
	struct nina_param p[2];
	struct net_sockaddr_in sender = {.sin_family = NET_AF_INET};
	unsigned int count;
	size_t n, copied;
	int ret;

	if ((flags & ~ZSOCK_MSG_DONTWAIT) || (!!from != !!fromlen)) {
		return fail(EOPNOTSUPP);
	}
	if (len && !buf) {
		return fail(EINVAL);
	}
	k_mutex_lock(&socket_lock, K_FOREVER);
	ret = s->bound ? 0 : udp_bind_locked(s, 0);
	k_mutex_unlock(&socket_lock);
	if (ret) {
		return fail(ret);
	}
	for (;;) {
		bool matches;

		k_mutex_lock(&udp_lock, K_FOREVER);
		ret = available(s, &count);
		if (ret) {
			goto error;
		}
		if (!count) {
			k_mutex_unlock(&udp_lock);
			if (flags & ZSOCK_MSG_DONTWAIT) {
				return fail(EAGAIN);
			}
			k_sleep(K_MSEC(20));
			continue;
		}
		ret = cmd1(0x3a, s->id, remote, sizeof(remote), &n);
		if (ret || n != sizeof(remote)) {
			ret = ret ? ret : -EPROTO;
			goto error;
		}
		memcpy(&sender.sin_addr, remote, 4);
		memcpy(&sender.sin_port, remote + 4, 2);
		matches = !s->connected ||
			  (sender.sin_addr.s_addr == s->peer.sin_addr.s_addr &&
			   sender.sin_port == s->peer.sin_port);
		copied = 0;
		p[0] = (struct nina_param){&s->id, 1};
		p[1] = (struct nina_param){length, 2};
		while (count) {
			size_t chunk = MIN(count, NINA_MAX_IO);
			size_t keep = matches ? MIN(chunk, len - copied) : 0;

			/* Avoid consuming a discarded suffix into the caller
			 * buffer. */
			if (keep) {
				chunk = keep;
			}
			length[0] = chunk & 255;
			length[1] = chunk >> 8;
			ret = nina_request(0x45, p, 2, true, true,
					   keep ? (uint8_t *)buf + copied
						: discard,
					   chunk, &n);
			if (ret || !n || n > chunk) {
				ret = ret ? ret : -EPROTO;
				goto error;
			}
			count -= n;
			if (keep) {
				copied += n;
			}
		}
		k_mutex_unlock(&udp_lock);
		if (!matches) {
			continue;
		}
		if (from) {
			net_socklen_t capacity = *fromlen;

			memcpy(from, &sender, MIN(capacity, sizeof(sender)));
			*fromlen = sizeof(sender);
		}
		return copied;
	}
error:
	k_mutex_unlock(&udp_lock);
	return fail(ret);
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

	if (s->udp) {
		return udp_recv(s, buf, len, flags, from, fromlen);
	}
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
	bool udp = s->udp;
	uint8_t result;
	size_t n;
	int ret = 0;

	if (udp) {
		k_mutex_lock(&udp_lock, K_FOREVER);
	}
	k_mutex_lock(&socket_lock, K_FOREVER);
	if (s->id != NINA_NO_SOCKET) {
		ret = cmd1(0x2e, s->id, &result, 1, &n);
	}
	s->used = false;
	s->connected = false;
	s->bound = false;
	s->id = NINA_NO_SOCKET;
	k_mutex_unlock(&socket_lock);
	if (udp) {
		k_mutex_unlock(&udp_lock);
	}
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

	for (;;) {
		int ready = 0;

		for (int i = 0; i < nfds; i++) {
			struct nina_sock *s;
			const struct fd_op_vtable *vtable;
			unsigned int count;
			uint8_t state;
			int ret;

			fds[i].revents = 0;
			if (fds[i].fd < 0) {
				continue;
			}
			s = zvfs_get_fd_obj_and_vtable(fds[i].fd, &vtable,
						       NULL);
			if (!s) {
				fds[i].revents = ZSOCK_POLLNVAL;
			} else if (vtable !=
				   (const struct fd_op_vtable *)&socket_ops) {
				return fail(ENOTSUP);
			} else if (!s->udp && !s->connected) {
				fds[i].revents = ZSOCK_POLLHUP;
			} else {
				if (fds[i].events & ZSOCK_POLLOUT) {
					fds[i].revents |= ZSOCK_POLLOUT;
				}
				if ((fds[i].events & ZSOCK_POLLIN) &&
				    (!s->udp || s->bound)) {
					if (s->udp) {
						k_mutex_lock(&udp_lock,
							     K_FOREVER);
					}
					ret = available(s, &count);
					if (s->udp) {
						k_mutex_unlock(&udp_lock);
					}
					if (ret) {
						return fail(ret);
					}
					if (count) {
						fds[i].revents |= ZSOCK_POLLIN;
					} else if (!s->udp) {
						ret = sock_state(s, &state);
						if (ret) {
							return fail(ret);
						}
						if (state != 4) {
							fds[i].revents |=
								ZSOCK_POLLHUP;
						}
					}
				}
			}
			if (fds[i].revents) {
				ready++;
			}
		}
		if (ready) {
			return ready;
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
	.bind = do_bind,
	.connect = do_connect,
	.sendto = do_send,
	.sendmsg = do_sendmsg,
	.recvfrom = do_recv,
};
static bool supported(int family, int type, int proto)
{
	return family == NET_AF_INET &&
	       ((type == NET_SOCK_STREAM &&
		 (proto == 0 || proto == NET_IPPROTO_TCP)) ||
		(type == NET_SOCK_DGRAM &&
		 (proto == 0 || proto == NET_IPPROTO_UDP)));
}

int nina_socket_create(int family, int type, int proto)
{
	int fd;
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
	s->id = NINA_NO_SOCKET;
	s->used = true;
	s->connected = false;
	s->udp = type == NET_SOCK_DGRAM;
	s->bound = false;
	k_mutex_unlock(&socket_lock);
	zvfs_finalize_typed_fd(fd, s, (const struct fd_op_vtable *)&socket_ops,
			       ZVFS_MODE_IFSOCK);
	return fd;
}
NET_SOCKET_OFFLOAD_REGISTER(nina, CONFIG_NET_SOCKETS_OFFLOAD_PRIORITY,
			    NET_AF_INET, supported, nina_socket_create);
