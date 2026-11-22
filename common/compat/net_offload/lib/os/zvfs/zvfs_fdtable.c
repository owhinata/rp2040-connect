/*
 * Copyright (c) 2018 Linaro Limited
 * Copyright (c) 2024 Tenstorrent AI ULC
 * Copyright (c) 2025 Antmicro
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file
 * @brief File descriptor table
 *
 * This file provides generic file descriptor table implementation, suitable
 * for any I/O object implementing POSIX I/O semantics (i.e. read/write +
 * aux operations).
 */

/* Zephyr's lib/os/zvfs/zvfs_fdtable.c (Zephyr v4.5.0), reduced to the
 * descriptor table of the layer's sockets (<zephyr/sys/fdtable.h>
 * here): the functions below are Zephyr's text except where marked.  The
 * table has ZVFS_OPEN_SIZE entries, CONFIG_ZVFS_OPEN_MAX of the Zephyr build
 * (16), numbered from 0 with the lowest free one first; 0, 1 and 2 are not
 * kept for stdin, stdout and stderr (the Zephyr build has no
 * CONFIG_POSIX_DEVICE_IO either).  The numbers are only the zsock_*()
 * calls': newlib's descriptors (0, 1 and 2 are the console's) are another
 * table.
 *
 * Left out: the files (zvfs_open() and its slab, the offset, read, write,
 * pread/pwrite, lseek, fstat, fsync, ftruncate, ioctl, fdopen/fileno,
 * unlink, rename), the stdin/stdout/stderr entries, the condition variable
 * of each entry (only Zephyr's native sockets wait on it), the user mode
 * and zvfs_fcntl() (the socket layer's fcntl calls the ioctl method itself).
 *
 * One change: zvfs_reserve_fd() does not initialize the entry's mutex again.
 * On FreeRTOS the layer's k_mutex is a FreeRTOS mutex, and creating it again
 * while a task waits for it (a call that looked the descriptor up before a
 * close freed it) would break FreeRTOS's lists; a zero k_mutex is an
 * unlocked one (as the layer requires on every OS) that its
 * first use creates, and the mutex is unlocked whenever the entry is free
 * (zvfs_close() unlocks it before it frees the entry), so the entry starts
 * as in Zephyr.
 *
 * Part of the network part of the Zephyr compatibility layer for an OS
 * without a network stack of its own (common/compat/net_offload), which the
 * FreeRTOS build uses.
 */

#include <errno.h>
#include <string.h>
#include <stdio.h>

#include <zephyr/kernel.h>
#include <zephyr/sys/fdtable.h>
#include <zephyr/sys/atomic.h>

/* The Zephyr build's CONFIG_ZVFS_OPEN_MAX (zephyr/build/usb-console/zephyr/
 * .config; lib/os/CMakeLists.txt makes it ZVFS_OPEN_SIZE).
 */
#ifndef ZVFS_OPEN_SIZE
#define ZVFS_OPEN_SIZE 16
#endif

struct fd_entry {
	void *obj;
	const struct fd_op_vtable *vtable;
	atomic_t refcount;
	struct k_mutex lock;
	uint32_t mode;
};

static struct fd_entry fdtable[ZVFS_OPEN_SIZE];

static K_MUTEX_DEFINE(fdtable_lock);

static int z_fd_ref(int fd)
{
	return atomic_inc(&fdtable[fd].refcount) + 1;
}

static int z_fd_unref(int fd)
{
	atomic_val_t old_rc;

	/* Reference counter must be checked to avoid decrement refcount below
	 * zero causing file descriptor leak. Loop statement below executes
	 * atomic decrement if refcount value is grater than zero. Otherwise,
	 * refcount is not going to be written.
	 */
	do {
		old_rc = atomic_get(&fdtable[fd].refcount);
		if (!old_rc) {
			return 0;
		}
	} while (!atomic_cas(&fdtable[fd].refcount, old_rc, old_rc - 1));

	if (old_rc != 1) {
		return old_rc - 1;
	}

	fdtable[fd].obj = NULL;
	fdtable[fd].vtable = NULL;

	return 0;
}

static int _find_fd_entry(void)
{
	int fd;

	for (fd = 0; fd < ARRAY_SIZE(fdtable); fd++) {
		if (!atomic_get(&fdtable[fd].refcount)) {
			return fd;
		}
	}

	errno = ENFILE;
	return -1;
}

/* Zephyr's also sanitizes the index against speculation
 * (k_array_index_sanitize(), a no-op on this CPU).
 */
static int _check_fd(int fd)
{
	if ((fd < 0) || (fd >= ARRAY_SIZE(fdtable))) {
		errno = EBADF;
		return -1;
	}

	if (!atomic_get(&fdtable[fd].refcount)) {
		errno = EBADF;
		return -1;
	}

	return 0;
}

void *zvfs_get_fd_obj(int fd, const struct fd_op_vtable *vtable, int err)
{
	struct fd_entry *entry;

	if (_check_fd(fd) < 0) {
		return NULL;
	}

	entry = &fdtable[fd];

	if ((vtable != NULL) && (entry->vtable != vtable)) {
		errno = err;
		return NULL;
	}

	return entry->obj;
}

void *zvfs_get_fd_obj_and_vtable(int fd, const struct fd_op_vtable **vtable, struct k_mutex **lock)
{
	struct fd_entry *entry;

	if (_check_fd(fd) < 0) {
		return NULL;
	}

	entry = &fdtable[fd];
	*vtable = entry->vtable;

	if (lock != NULL) {
		*lock = &entry->lock;
	}

	return entry->obj;
}

int zvfs_reserve_fd(void)
{
	int fd;

	(void)k_mutex_lock(&fdtable_lock, K_FOREVER);

	fd = _find_fd_entry();
	if (fd >= 0) {
		/* Mark entry as used, zvfs_finalize_fd() will fill it in. */
		(void)z_fd_ref(fd);
		fdtable[fd].obj = NULL;
		fdtable[fd].vtable = NULL;
		/* Zephyr: k_mutex_init() and k_condvar_init() (see above). */
	}

	k_mutex_unlock(&fdtable_lock);

	return fd;
}

void zvfs_finalize_typed_fd(int fd, void *obj, const struct fd_op_vtable *vtable, uint32_t mode)
{
	/* Assumes fd was already bounds-checked. */
	fdtable[fd].obj = obj;
	fdtable[fd].vtable = vtable;
	fdtable[fd].mode = mode;

	/* Let the object know about the lock just in case it needs it
	 * for something. For BSD sockets, the lock is used with condition
	 * variables to avoid keeping the lock for a long period of time.
	 */
	if (vtable && vtable->ioctl) {
		int prev_errno = errno;

		(void)zvfs_fdtable_call_ioctl(vtable, obj, ZFD_IOCTL_SET_LOCK, &fdtable[fd].lock);
		if ((prev_errno != EOPNOTSUPP) && (errno == EOPNOTSUPP)) {
			/* restore backed-up errno value if the backend does not support locking */
			errno = prev_errno;
		}
	}
}

void zvfs_free_fd(int fd)
{
	/* Assumes fd was already bounds-checked. */
	(void)z_fd_unref(fd);
}

int zvfs_alloc_fd(void *obj, const struct fd_op_vtable *vtable)
{
	int fd;

	fd = zvfs_reserve_fd();
	if (fd >= 0) {
		zvfs_finalize_fd(fd, obj, vtable);
	}

	return fd;
}

int zvfs_close(int fd)
{
	int res = 0;

	if (_check_fd(fd) < 0) {
		return -1;
	}

	(void)k_mutex_lock(&fdtable[fd].lock, K_FOREVER);
	if (fdtable[fd].vtable->close != NULL) {
		/* close() is optional - e.g. stdinout_fd_op_vtable */
		if (fdtable[fd].mode & ZVFS_MODE_IFSOCK) {
			/* Network socket needs to know socket number so pass
			 * it via close2() call.
			 */
			res = fdtable[fd].vtable->close2(fdtable[fd].obj, fd);
			zvfs_socket_close_hook(fd);
		} else {
			res = fdtable[fd].vtable->close(fdtable[fd].obj);
		}
	}
	k_mutex_unlock(&fdtable[fd].lock);

	zvfs_free_fd(fd);

	return res;
}
