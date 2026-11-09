/* SPDX-License-Identifier: Apache-2.0 */
#ifndef TIME_SYNC_H_
#define TIME_SYNC_H_

#include <stdbool.h>

void time_sync_init(void);
bool time_sync_is_ready(void);

#endif
