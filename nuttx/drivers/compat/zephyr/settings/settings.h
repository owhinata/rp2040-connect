/* SPDX-License-Identifier: Apache-2.0 */
/* The part of Zephyr's <zephyr/settings/settings.h> (Zephyr v4.5.0) that
 * the stored Wi-Fi credentials and the Wi-Fi profile (nuttx/apps/wifi) use:
 * one item by its full name, in NuttX's NVS (/dev/config, the store of the
 * "kv" command) under the same name.  Zephyr's settings on NVS keep each
 * item the same way, so only the subtree reads with callbacks
 * (settings_load_subtree_direct()) and the handlers are not here; the
 * credentials read their few names one by one instead.
 *
 * Every call opens /dev/config, does one ioctl and closes it, as
 * nuttx/apps/imu/kv_store.c does (the driver serializes the calls, as
 * Zephyr's settings lock does).  Threads only.  The implementation is
 * nuttx/apps/wifi/settings_nvs.c.
 *
 * Part of the small Zephyr compatibility layer of the port
 * (nuttx/drivers/compat, see nuttx/drivers/README.md).
 */

#ifndef __NUTTX_DRIVERS_COMPAT_ZEPHYR_SETTINGS_SETTINGS_H
#define __NUTTX_DRIVERS_COMPAT_ZEPHYR_SETTINGS_SETTINGS_H

#include <stddef.h>
#include <sys/types.h>

/* 0 if the store is there, -ENODEV if not (Zephyr: initializes the
 * settings once; here the board has mounted the store at boot).
 */

int settings_subsys_init(void);

/* Stores VALUE (VAL_LEN bytes) as NAME, replacing what NAME had; storing
 * the value already stored writes nothing.  VAL_LEN 0 deletes NAME.  0 or a
 * negative errno (-ENOSPC when the store is full).
 */

int settings_save_one(const char *name, const void *value, size_t val_len);

/* Deletes NAME; 0 also when NAME was not stored. */

int settings_delete(const char *name);

/* Reads NAME into BUF (at most BUF_LEN bytes).  The stored length (which
 * may be more than BUF_LEN), 0 if NAME is not stored, or a negative errno.
 */

ssize_t settings_load_one(const char *name, void *buf, size_t buf_len);

#endif /* __NUTTX_DRIVERS_COMPAT_ZEPHYR_SETTINGS_SETTINGS_H */
