/* SPDX-License-Identifier: Apache-2.0 */
/* hex2bin() and bin2hex() of Zephyr's lib/utils/hex.c, which the Zephyr
 * build's "kv sethex" and "kv get" use, with the same results: upper- and
 * lower-case digits are accepted, an odd number of digits gets a leading
 * zero nibble, any other character or a buffer that is too small gives 0,
 * and the output is lower case.  Tested on the host (nuttx/tests/kv_hex.c).
 */

#ifndef APP_KV_HEX_H_
#define APP_KV_HEX_H_

#include <stddef.h>
#include <stdint.h>

size_t kv_bin2hex(const uint8_t *buf, size_t buflen, char *hex, size_t hexlen);
size_t kv_hex2bin(const char *hex, size_t hexlen, uint8_t *buf, size_t buflen);

#endif
