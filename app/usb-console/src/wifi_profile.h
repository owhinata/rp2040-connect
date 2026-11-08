/* SPDX-License-Identifier: Apache-2.0 */
#ifndef WIFI_PROFILE_H_
#define WIFI_PROFILE_H_
#include <zephyr/net/wifi_mgmt.h>
int wifi_profile_init(void);
void wifi_profile_prepare(const struct wifi_connect_req_params *params);
void wifi_profile_commit(void);
void wifi_profile_discard(void);
#endif
