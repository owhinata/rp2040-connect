/* SPDX-License-Identifier: Apache-2.0 */
#ifndef NINA_SEND_PROFILE_H_
#define NINA_SEND_PROFILE_H_

#include <stdint.h>

struct nina_send_sample {
	uint32_t lock_cycles, pre_cycles, tx_cycles, ready_cycles, reply_cycles;
	uint32_t requested, accepted;
	int error, phase;
};

void nina_send_profile_record(const struct nina_send_sample *sample);
void nina_send_profile_retry(uint32_t cycles);
#endif
