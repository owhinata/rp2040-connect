/* SPDX-License-Identifier: Apache-2.0 */
#ifndef NINA_PROTO_H_
#define NINA_PROTO_H_
#include <stddef.h>
#include <stdint.h>
#include <stdbool.h>
struct nina_param {
	const uint8_t *data;
	size_t len;
};
int nina_request(uint8_t cmd, const struct nina_param *params, size_t nparams,
		 bool wide_params, bool wide_response, uint8_t *out, size_t cap,
		 size_t *out_len);
int nina_ip_config(uint8_t data[12]);
#endif
