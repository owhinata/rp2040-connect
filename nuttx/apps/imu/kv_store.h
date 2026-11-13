/* SPDX-License-Identifier: Apache-2.0 */
#ifndef APP_KV_STORE_H_
#define APP_KV_STORE_H_

#include <stddef.h>
#include <sys/types.h>

#define KV_STORE_KEY_MAX   32
#define KV_STORE_VALUE_MAX 128

/* Keys contain ASCII letters, digits, '.', '-' or '_'. Values are binary.
 * A zero-length value is not supported. Data is stored without encryption.
 * All calls must run in thread context and return negative errno on failure.
 */
int kv_store_init(void);
int kv_store_set(const char *key, const void *value, size_t len);
/* Returns the value length; -ENOENT if absent, -ENOBUFS if too small. */
ssize_t kv_store_get(const char *key, void *value, size_t capacity);
int kv_store_delete(const char *key);

#endif
