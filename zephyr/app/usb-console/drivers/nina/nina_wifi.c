/* SPDX-License-Identifier: Apache-2.0 */
/* NINA Arduino firmware SPI protocol: WiFiNINA / Arduino_SpiNINA. */
#include <errno.h>
#include <string.h>
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/kernel.h>

#define NINA_START 0xe0
#define NINA_END   0xee
#define NINA_DUMMY 0xff
#define NINA_REPLY 0x80
#define NINA_NODE  DT_NODELABEL(nina_w102)

static const struct device *const spi = DEVICE_DT_GET(DT_NODELABEL(spi1));
static const struct gpio_dt_spec ready =
	GPIO_DT_SPEC_GET(NINA_NODE, ready_gpios);
static const struct gpio_dt_spec reset =
	GPIO_DT_SPEC_GET(NINA_NODE, reset_gpios);
static const struct gpio_dt_spec irq = GPIO_DT_SPEC_GET(NINA_NODE, irq_gpios);
static const struct gpio_dt_spec cs =
	GPIO_DT_SPEC_GET_BY_IDX(DT_NODELABEL(spi1), cs_gpios, 0);
static const struct spi_config config = {
	.frequency = CONFIG_NINA_SPI_FREQUENCY, .operation = SPI_WORD_SET(8) | SPI_TRANSFER_MSB};
static bool initialized;
K_MUTEX_DEFINE(nina_lock);

/* How often the module's ready line is sampled while waiting for it. */
#define NINA_READY_POLL_US 100

static int wait_ready(int desired, int timeout_ms)
{
	int64_t deadline = k_uptime_get() + timeout_ms;
	int value;

	do {
		value = gpio_pin_get_dt(&ready);
		if (value < 0) {
			return value;
		}
		if (value == desired) {
			return 0;
		}
		k_sleep(K_USEC(NINA_READY_POLL_US));
	} while (k_uptime_get() < deadline);
	return -ETIMEDOUT;
}

static int init_nina(void)
{
	int ret;

	if (initialized) {
		return 0;
	}
	if (!device_is_ready(spi) || !gpio_is_ready_dt(&ready) ||
	    !gpio_is_ready_dt(&reset) || !gpio_is_ready_dt(&irq) ||
	    !gpio_is_ready_dt(&cs)) {
		return -ENODEV;
	}
	if ((ret = gpio_pin_configure_dt(&cs, GPIO_OUTPUT_INACTIVE)) ||
	    (ret = gpio_pin_configure_dt(&ready, GPIO_INPUT)) ||
	    (ret = gpio_pin_configure_dt(&irq, GPIO_OUTPUT_HIGH)) ||
	    (ret = gpio_pin_configure_dt(&reset, GPIO_OUTPUT_ACTIVE))) {
		return ret;
	}
	k_sleep(K_MSEC(10));
	if ((ret = gpio_pin_set_dt(&reset, 0))) {
		return ret;
	}
	k_sleep(K_MSEC(750));
	if ((ret = gpio_pin_set_dt(&irq, 0)) ||
	    (ret = gpio_pin_configure_dt(&irq, GPIO_INPUT))) {
		return ret;
	}
	initialized = true;
	return 0;
}

static int transfer(uint8_t tx, uint8_t *rx)
{
	struct spi_buf tb = {.buf = &tx, .len = 1}, rb = {.buf = rx, .len = 1};
	struct spi_buf_set ts = {.buffers = &tb, .count = 1},
			   rs = {.buffers = &rb, .count = 1};

	return spi_transceive(spi, &config, &ts, &rs);
}

/* The caller keeps NINA's chip select asserted across these transfers. */
static int transfer_write(const uint8_t *data, size_t len)
{
	struct spi_buf buf = {.buf = (void *)data, .len = len};
	struct spi_buf_set set = {.buffers = &buf, .count = 1};

	return len ? spi_write(spi, &config, &set) : 0;
}

static int transfer_read(uint8_t *data, size_t len)
{
	uint8_t dummy[128];

	memset(dummy, NINA_DUMMY, sizeof(dummy));
	while (len) {
		size_t chunk = MIN(len, sizeof(dummy));
		struct spi_buf tb = {.buf = dummy, .len = chunk};
		struct spi_buf rb = {.buf = data, .len = chunk};
		struct spi_buf_set ts = {.buffers = &tb, .count = 1};
		struct spi_buf_set rs = {.buffers = &rb, .count = 1};
		int ret = spi_transceive(spi, &config, &ts, &rs);

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
	if ((ret = gpio_pin_set_dt(&cs, 1))) {
		return ret;
	}

	ret = wait_ready(0, 20);
	if (!ret) {
		ret = transfer_write(tx, n);
	}
	(void)gpio_pin_set_dt(&cs, 0);
	memset(tx, 0, sizeof(tx));

	if (ret) {
		return ret;
	}
	if ((ret = wait_ready(1, 2000))) {
		return ret;
	}
	if ((ret = gpio_pin_set_dt(&cs, 1))) {
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
	(void)gpio_pin_set_dt(&cs, 0);
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
	k_mutex_lock(&nina_lock, K_FOREVER);
	ret = init_nina();
	if (!ret) {
		ret = wait_ready(1, 2000);
	}
	if (!ret) {
		ret = gpio_pin_set_dt(&cs, 1);
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
		(void)gpio_pin_set_dt(&cs, 0);
	}
	if (!ret) {
		ret = wait_ready(1, 10000);
	}
	if (!ret) {
		ret = gpio_pin_set_dt(&cs, 1);
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
		(void)gpio_pin_set_dt(&cs, 0);
	}
	if (!ret) {
		*out_len = total;
	}
	k_mutex_unlock(&nina_lock);
	return ret;
#undef SEND_BYTE
}

int nina_ip_config(uint8_t data[12])
{
	const uint8_t dummy = NINA_DUMMY;
	size_t len = 0;
	int ret;

	k_mutex_lock(&nina_lock, K_FOREVER);
	ret = command(0x21, &dummy, 1, NULL, 0, data, 12, &len);
	k_mutex_unlock(&nina_lock);
	return ret ? ret : (len == 12 ? 0 : -EPROTO);
}
