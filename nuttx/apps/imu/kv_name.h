/* SPDX-License-Identifier: Apache-2.0 */
/* The key checks of the key-value store, without the store: the
 * OS-independent part of zephyr/app/usb-console/src/kv_store.c
 * (make_name() and shell_key_allowed()), shared by kv_store.c and the "kv"
 * command and tested on the host (nuttx/tests/kv_name.c).
 */

#ifndef APP_KV_NAME_H_
#define APP_KV_NAME_H_

#include <stdbool.h>
#include <stddef.h>

#include "kv_store.h"

/* Stored names are KV_PREFIX followed by the key, as in the Zephyr build,
 * which keeps the shell's keys apart from other users of the store.
 */

#define KV_PREFIX        "app_kv/"
#define KV_SYSTEM_PREFIX "sys."

/* Room for the longest name and its terminating NUL. */

#define KV_NAME_SIZE     (sizeof(KV_PREFIX) + KV_STORE_KEY_MAX)

/* Writes KV_PREFIX + key to name.  -EINVAL unless key is 1 to
 * KV_STORE_KEY_MAX characters of ASCII letters, digits, '_', '-' or '.';
 * -ENAMETOOLONG if name has less than KV_NAME_SIZE bytes.
 */
int kv_make_name(const char *key, char *name, size_t capacity);

/* False for the keys that the shell must not touch ("sys." ...). */
bool kv_shell_key_allowed(const char *key);

#endif
