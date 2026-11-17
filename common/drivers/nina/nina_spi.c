/* SPDX-License-Identifier: Apache-2.0 */
/* NINA Arduino firmware SPI protocol: WiFiNINA / Arduino_SpiNINA.
 *
 * A line-for-line port of the Zephyr build's drivers/nina/nina_wifi.c:
 * the same branches, order, constants, timeouts and error values.  Only
 * the calls into the system are replaced:
 *
 *   Zephyr                              here
 *   device_is_ready(), gpio_is_ready_dt nano_nina_bus_ready()
 *   gpio_pin_configure_dt()             nano_nina_pin_configure()
 *   gpio_pin_set_dt(), gpio_pin_get_dt  nano_nina_pin_set(), _get()
 *   spi_transceive(), spi_write()       nano_nina_transceive()
 *   K_MUTEX_DEFINE(nina_lock)           nina_os_lock(), nina_os_unlock()
 *   k_sleep(), k_uptime_get()           nina_os_sleep_us/_ms(), _uptime_ms()
 *
 * nano_nina_*() are the board's (<nano_nina.h>,
 * boards/arduino-nano-rp2040-connect/src/nano_nina.c), nina_os_*() are in
 * nina_os.c.  The host test (nuttx/tests/nina_spi.c) builds this file with
 * fakes of both.
 *
 * One difference: nina_os_lock() can fail where k_mutex_lock(K_FOREVER)
 * cannot (only with pthread cancellation, which this build does not use);
 * nina_request() and nina_ip_config() then return its error untouched.
 */

#include <errno.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

#include "nano_nina.h"
#include "nina_os.h"

#define NINA_START 0xe0
#define NINA_END   0xee
#define NINA_DUMMY 0xff
#define NINA_REPLY 0x80

#ifndef MIN
#  define MIN(a, b) ((a) < (b) ? (a) : (b))
#endif

static bool initialized;

/* How often the module's ready line is sampled while waiting for it. */
#define NINA_READY_POLL_US 100

static int wait_ready(int desired, int timeout_ms)
{
	int64_t deadline = nina_os_uptime_ms() + timeout_ms;
	int value;

	do {
		value = nano_nina_pin_get(NANO_NINA_READY);
		if (value < 0) {
			return value;
		}
		if (value == desired) {
			return 0;
		}
		nina_os_sleep_us(NINA_READY_POLL_US);
	} while (nina_os_uptime_ms() < deadline);
	return -ETIMEDOUT;
}

static int init_nina(void)
{
	int ret;

	if (initialized) {
		return 0;
	}
	if (!nano_nina_bus_ready()) {
		return -ENODEV;
	}
	if ((ret = nano_nina_pin_configure(NANO_NINA_CS, NANO_NINA_OUTPUT_INACTIVE)) ||
	    (ret = nano_nina_pin_configure(NANO_NINA_READY, NANO_NINA_INPUT)) ||
	    (ret = nano_nina_pin_configure(NANO_NINA_IRQ, NANO_NINA_OUTPUT_HIGH)) ||
	    (ret = nano_nina_pin_configure(NANO_NINA_RESET, NANO_NINA_OUTPUT_ACTIVE))) {
		return ret;
	}
	nina_os_sleep_ms(10);
	if ((ret = nano_nina_pin_set(NANO_NINA_RESET, 0))) {
		return ret;
	}
	nina_os_sleep_ms(750);
	if ((ret = nano_nina_pin_set(NANO_NINA_IRQ, 0)) ||
	    (ret = nano_nina_pin_configure(NANO_NINA_IRQ, NANO_NINA_INPUT))) {
		return ret;
	}
	initialized = true;
	return 0;
}

static int transfer(uint8_t tx, uint8_t *rx)
{
	return nano_nina_transceive(&tx, rx, 1);
}

/* The caller keeps NINA's chip select asserted across these transfers. */
static int transfer_write(const uint8_t *data, size_t len)
{
	return len ? nano_nina_transceive(data, NULL, len) : 0;
}

static int transfer_read(uint8_t *data, size_t len)
{
	uint8_t dummy[128];

	memset(dummy, NINA_DUMMY, sizeof(dummy));
	while (len) {
		size_t chunk = MIN(len, sizeof(dummy));
		int ret = nano_nina_transceive(dummy, data, chunk);

		if (ret) {
			return ret;
		}
		data += chunk;
		len -= chunk;
	}
	return 0;
}

static int command(uint8_t cmd, const uint8_t *a, size_t alen, const uint8_t *b,
		   size_t blen, uint8_t *out, size_t cap, size_t *outlen)
{
	uint8_t c, tx[108];
	size_t n = 0, len = 0, i, p, count;
	int ret;

	if (alen > 32 || blen > 63) {
		return -EINVAL;
	}
	if ((ret = init_nina())) {
		return ret;
	}
	if ((ret = wait_ready(1, 2000))) {
		return ret; /* active-low ready */
	}

	tx[n++] = NINA_START;
	tx[n++] = cmd;
	tx[n++] = a ? (b ? 2 : 1) : 0;
	if (a) {
		tx[n++] = alen;
		memcpy(tx + n, a, alen);
		n += alen;
	}
	if (b) {
		tx[n++] = blen;
		memcpy(tx + n, b, blen);
		n += blen;
	}
	tx[n++] = NINA_END;
	while (n % 4) {
		tx[n++] = NINA_DUMMY;
	}
	if ((ret = nano_nina_pin_set(NANO_NINA_CS, 1))) {
		return ret;
	}

	ret = wait_ready(0, 20);
	if (!ret) {
		ret = transfer_write(tx, n);
	}
	(void)nano_nina_pin_set(NANO_NINA_CS, 0);
	memset(tx, 0, sizeof(tx));

	if (ret) {
		return ret;
	}
	if ((ret = wait_ready(1, 2000))) {
		return ret;
	}
	if ((ret = nano_nina_pin_set(NANO_NINA_CS, 1))) {
		return ret;
	}

	ret = wait_ready(0, 20);
	for (i = 0; !ret && i < 1000; i++) {
		ret = transfer(NINA_DUMMY, &c);
		if (ret || c == NINA_START) {
			break;
		}
	}
	if (!ret && c != NINA_START) {
		ret = -EPROTO;
	}
	if (!ret) {
		ret = transfer(NINA_DUMMY, &c);
		if (!ret && c != (cmd | NINA_REPLY)) {
			ret = -EPROTO;
		}
	}
	if (!ret) {
		ret = transfer(NINA_DUMMY, &c);
		if (!ret && c != (cmd == 0x21 ? 3 : 1)) {
			ret = -EPROTO;
		}
	}
	count = cmd == 0x21 ? 3 : 1;
	for (p = 0; !ret && p < count; p++) {
		ret = transfer(NINA_DUMMY, &c);
		if (!ret && (len + c > cap || (cmd == 0x21 && c != 4))) {
			ret = -EMSGSIZE;
		}
		if (!ret) {
			ret = transfer_read(out + len, c);
			len += c;
		}
	}
	if (!ret) {
		ret = transfer(NINA_DUMMY, &c);
		if (!ret && c != NINA_END) {
			ret = -EPROTO;
		}
	}
	(void)nano_nina_pin_set(NANO_NINA_CS, 0);
	if (!ret) {
		*outlen = len;
	}
	return ret;
}

/* Socket commands use 16-bit parameter lengths; regular commands use 8-bit. */
#include "nina_proto.h"
int nina_request(uint8_t cmd, const struct nina_param *params, size_t nparams,
		 bool wide_params, bool wide_response, uint8_t *out, size_t cap,
		 size_t *out_len)
{
	uint8_t c = 0;
	size_t i, j, sent = 0, len = 0, total = 0;
	/* accept and recvfrom answer with three parameters. */
	uint8_t response_count = cmd == 0x75 || cmd == 0x7a ? 3 : 1;
	int ret;
#define SEND_BYTE(v)                                                           \
	do {                                                                   \
		if (!ret) {                                                    \
			ret = transfer((v), &c);                               \
			sent++;                                                \
		}                                                              \
	} while (0)
	if (nparams > 6 || !out || !out_len) {
		return -EINVAL;
	}
	for (i = 0; i < nparams; i++) {
		if ((!params[i].data && params[i].len) ||
		    params[i].len > (wide_params ? 65535 : 255)) {
			return -EINVAL;
		}
	}
	if ((ret = nina_os_lock())) {
		return ret;
	}
	ret = init_nina();
	if (!ret) {
		ret = wait_ready(1, 2000);
	}
	if (!ret) {
		ret = nano_nina_pin_set(NANO_NINA_CS, 1);
	}
	if (!ret) {
		ret = wait_ready(0, 20);
		SEND_BYTE(NINA_START);
		SEND_BYTE(cmd);
		SEND_BYTE(nparams);
		for (i = 0; !ret && i < nparams; i++) {
			if (wide_params) {
				SEND_BYTE(params[i].len >> 8);
			}
			SEND_BYTE(params[i].len & 0xff);
			if (!ret) {
				ret = transfer_write(params[i].data, params[i].len);
				sent += params[i].len;
			}
		}
		SEND_BYTE(NINA_END);
		while (!ret && sent % 4) {
			SEND_BYTE(NINA_DUMMY);
		}
		(void)nano_nina_pin_set(NANO_NINA_CS, 0);
	}
	if (!ret) {
		ret = wait_ready(1, 10000);
	}
	if (!ret) {
		ret = nano_nina_pin_set(NANO_NINA_CS, 1);
	}
	if (!ret) {
		ret = wait_ready(0, 20);
		for (i = 0; !ret && i < 1000; i++) {
			ret = transfer(NINA_DUMMY, &c);
			if (c == NINA_START) {
				break;
			}
		}
		if (!ret && c != NINA_START) {
			ret = -EPROTO;
		}
		if (!ret) {
			ret = transfer(NINA_DUMMY, &c);
			if (!ret && c != (cmd | NINA_REPLY)) {
				ret = -EPROTO;
			}
		}
		if (!ret) {
			ret = transfer(NINA_DUMMY, &c);
			if (!ret && cmd == 0x27) {
				response_count = c;
			} else if (!ret && c != response_count) {
				ret = -EPROTO;
			}
		}
		for (j = 0; !ret && j < response_count; j++) {
			len = 0;
			if (wide_response) {
				ret = transfer(NINA_DUMMY, &c);
				len = (size_t)c << 8;
			}
			if (!ret) {
				ret = transfer(NINA_DUMMY, &c);
				len |= c;
				if (cmd == 0x27 && (len > 32 || total == cap)) {
					ret = -EMSGSIZE;
				}
				if (!ret && cmd == 0x27) {
					out[total++] = len;
				}
				if (len > cap - total ||
				    (cmd == 0x75 && len != (j == 0 ? 1 : (j == 1 ? 4 : 2))) ||
				    (cmd == 0x7a && j < 2 && len != (j == 0 ? 4 : 2))) {
					ret = -EMSGSIZE;
				}
			}
			if (!ret) {
				ret = transfer_read(out + total, len);
				total += len;
			}
		}
		if (!ret) {
			ret = transfer(NINA_DUMMY, &c);
			if (!ret && c != NINA_END) {
				ret = -EPROTO;
			}
		}
		(void)nano_nina_pin_set(NANO_NINA_CS, 0);
	}
	if (!ret) {
		*out_len = total;
	}
	nina_os_unlock();
	return ret;
#undef SEND_BYTE
}

int nina_ip_config(uint8_t data[12])
{
	const uint8_t dummy = NINA_DUMMY;
	size_t len = 0;
	int ret;

	if ((ret = nina_os_lock())) {
		return ret;
	}
	ret = command(0x21, &dummy, 1, NULL, 0, data, 12, &len);
	nina_os_unlock();
	return ret ? ret : (len == 12 ? 0 : -EPROTO);
}
