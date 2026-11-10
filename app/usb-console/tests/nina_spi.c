/* SPDX-License-Identifier: Apache-2.0 */
#include <assert.h>
#include <errno.h>
#include <stdio.h>
#include <string.h>

#include "../drivers/nina/nina_wifi.c"

struct device fake_device;
static uint8_t request[2048], response[2048];
static size_t request_len, response_len, response_offset;
static unsigned int phase, calls, bulk_calls;
static int cs_active, fail_bulk;

bool device_is_ready(const struct device *dev) { return true; }
bool gpio_is_ready_dt(const struct gpio_dt_spec *spec) { return true; }
int gpio_pin_configure_dt(const struct gpio_dt_spec *spec, int flags) { return 0; }
int gpio_pin_set_dt(const struct gpio_dt_spec *spec, int value)
{
	if (spec == &cs) {
		cs_active = value;
		if (value) {
			phase++;
		}
	}
	return 0;
}
int gpio_pin_get_dt(const struct gpio_dt_spec *spec) { return !cs_active; }
void k_mutex_lock(struct k_mutex *lock, int timeout) {}
void k_mutex_unlock(struct k_mutex *lock) {}
void k_sleep(int ms) {}
int64_t k_uptime_get(void) { return 0; }

int spi_transceive(const struct device *dev, const struct spi_config *cfg,
		   const struct spi_buf_set *tx, const struct spi_buf_set *rx)
{
	const uint8_t *src = tx->buffers[0].buf;
	uint8_t *dst = rx->buffers[0].buf;
	size_t len = tx->buffers[0].len;

	assert(cs_active && tx->count == 1 && rx->count == 1);
	assert(len == rx->buffers[0].len);
	calls++;
	if (len > 1) {
		bulk_calls++;
		if (fail_bulk) {
			return -EIO;
		}
	}
	for (size_t i = 0; i < len; i++) {
		if (phase == 1) {
			assert(request_len < sizeof(request));
			request[request_len++] = src[i];
			dst[i] = 0xff;
		} else {
			assert(phase == 2 && src[i] == 0xff);
			assert(response_offset < response_len);
			dst[i] = response[response_offset++];
		}
	}
	return 0;
}

int spi_write(const struct device *dev, const struct spi_config *cfg,
	      const struct spi_buf_set *tx)
{
	const struct spi_buf *buf = &tx->buffers[0];

	assert(cs_active && phase == 1 && tx->count == 1);
	calls++;
	bulk_calls++;
	if (fail_bulk) {
		return -EIO;
	}
	assert(buf->len <= sizeof(request) - request_len);
	memcpy(request + request_len, buf->buf, buf->len);
	request_len += buf->len;
	return 0;
}

static void reset_fake(const uint8_t *reply, size_t len)
{
	memcpy(response, reply, len);
	response_len = len;
	response_offset = request_len = phase = calls = bulk_calls = 0;
	cs_active = fail_bulk = 0;
	initialized = true;
}

int main(void)
{
	uint8_t payload[512], output[512], id = 7;
	struct nina_param params[] = {{&id, 1}, {payload, sizeof(payload)}};
	size_t n;
	const uint8_t send_reply[] = {0xe0, 0xc4, 1, 2, 0x00, 0x02, 0xee};

	for (size_t i = 0; i < sizeof(payload); i++) {
		payload[i] = (uint8_t)i;
	}
	reset_fake(send_reply, sizeof(send_reply));
	assert(nina_request(0x44, params, 2, true, false, output, sizeof(output), &n) == 0);
	assert(n == 2 && output[0] == 0 && output[1] == 2);
	assert(request_len == 524 && (request_len % 4) == 0);
	assert(memcmp(request, "\xe0\x44\x02\x00\x01\x07\x02\x00", 8) == 0);
	assert(memcmp(request + 8, payload, sizeof(payload)) == 0);
	assert(request[520] == 0xee && request[521] == 0xff &&
	       request[522] == 0xff && request[523] == 0xff);
	assert(bulk_calls >= 1 && calls < 30 && !cs_active);

	uint8_t large[1460];
	for (size_t i = 0; i < sizeof(large); i++) {
		large[i] = (uint8_t)(i * 7);
	}
	params[1].data = large;
	params[1].len = sizeof(large);
	reset_fake(send_reply, sizeof(send_reply));
	assert(nina_request(0x44, params, 2, true, false, output, sizeof(output), &n) == 0);
	assert(request_len == 1472 && request[6] == 0x05 && request[7] == 0xb4);
	assert(memcmp(request + 8, large, sizeof(large)) == 0);
	assert(request[1468] == 0xee && request[1471] == 0xff && !cs_active);
	params[1].data = payload;
	params[1].len = sizeof(payload);

	uint8_t read_reply[519] = {0xe0, 0xc5, 1, 2, 0};
	memcpy(read_reply + 5, payload, sizeof(payload));
	read_reply[517] = 0xee;
	reset_fake(read_reply, 518);
	assert(nina_request(0x45, params, 1, true, true, output, sizeof(output), &n) == 0);
	assert(n == sizeof(payload) && memcmp(output, payload, n) == 0);
	assert(bulk_calls >= 4 && calls < 30 && !cs_active);

	const uint8_t accept_reply[] = {0xe0, 0xf5, 3, 1, 3, 4, 192, 168, 1, 2,
				       2, 0x34, 0x12, 0xee};
	reset_fake(accept_reply, sizeof(accept_reply));
	assert(nina_request(0x75, params, 1, false, false, output, sizeof(output), &n) == 0);
	assert(n == 7 && output[0] == 3 && output[1] == 192 && output[6] == 0x12);

	reset_fake(read_reply, 518);
	assert(nina_request(0x45, params, 1, true, true, output, 511, &n) == -EMSGSIZE);
	assert(!cs_active);

	const uint8_t malformed_accept[] = {0xe0, 0xf5, 3, 2, 3, 4, 0xee};
	reset_fake(malformed_accept, sizeof(malformed_accept));
	assert(nina_request(0x75, params, 1, false, false, output, sizeof(output), &n) ==
	       -EMSGSIZE);
	assert(!cs_active);

	reset_fake(send_reply, sizeof(send_reply));
	fail_bulk = 1;
	assert(nina_request(0x44, params, 2, true, false, output, sizeof(output), &n) == -EIO);
	assert(!cs_active);
	puts("NINA SPI framing and bulk transfer tests passed");
	return 0;
}
