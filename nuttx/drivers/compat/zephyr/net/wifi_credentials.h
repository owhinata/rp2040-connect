/* SPDX-License-Identifier: Apache-2.0 */
/* Zephyr's <zephyr/net/wifi_credentials.h> (Zephyr v4.5.0): the stored
 * Wi-Fi credentials, with Zephyr's names, types, flags and functions.  The
 * library is the copy of Zephyr's subsys/net/lib/wifi_credentials/ in
 * nuttx/apps/wifi (CONFIG_NANO_WIFI_CREDENTIALS), which keeps the entries
 * in NuttX's NVS (/dev/config) under the names "wifi_cred/<index>", as the
 * Zephyr build keeps them in its settings.  The entry is this header's
 * struct wifi_credentials_personal, stored as it is in memory, as Zephyr's
 * settings backend stores it; the store itself is NuttX's NVS, which the
 * Zephyr build cannot read (and the other way round).
 *
 * The configuration of the Zephyr build (CONFIG_WIFI_CREDENTIALS_*) comes
 * from the NANO_WIFI_CREDENTIALS_* options of nuttx/apps/wifi/Kconfig.
 *
 * NuttX only: wifi_credentials_init(), which the init task calls where
 * Zephyr runs the library's SYS_INIT() (before the threads and main()).
 *
 * Part of the small Zephyr compatibility layer of the port
 * (nuttx/drivers/compat, see nuttx/drivers/README.md).
 */

/*
 * Copyright (c) 2024 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_INCLUDE_NET_WIFI_CREDENTIALS_H_
#define ZEPHYR_INCLUDE_NET_WIFI_CREDENTIALS_H_

#ifdef __NuttX__
#  include <nuttx/config.h>
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include <zephyr/kernel.h>
#include <zephyr/net/wifi_mgmt.h>

/* The Zephyr build's configuration (zephyr/build/usb-console/zephyr/.config). */

#ifdef CONFIG_NANO_WIFI_CREDENTIALS_MAX_ENTRIES
#  define CONFIG_WIFI_CREDENTIALS_MAX_ENTRIES \
     CONFIG_NANO_WIFI_CREDENTIALS_MAX_ENTRIES
#else
#  define CONFIG_WIFI_CREDENTIALS_MAX_ENTRIES 2
#endif

#ifdef CONFIG_NANO_WIFI_CREDENTIALS_SAE_PASSWORD_LENGTH
#  define CONFIG_WIFI_CREDENTIALS_SAE_PASSWORD_LENGTH \
     CONFIG_NANO_WIFI_CREDENTIALS_SAE_PASSWORD_LENGTH
#else
#  define CONFIG_WIFI_CREDENTIALS_SAE_PASSWORD_LENGTH 128
#endif

#ifdef CONFIG_NANO_WIFI_CREDENTIALS_CONNECTION_TIMEOUT
#  define CONFIG_WIFI_CREDENTIALS_CONNECT_STORED_CONNECTION_TIMEOUT \
     CONFIG_NANO_WIFI_CREDENTIALS_CONNECTION_TIMEOUT
#else
#  define CONFIG_WIFI_CREDENTIALS_CONNECT_STORED_CONNECTION_TIMEOUT 30
#endif

/* CONFIG_WIFI_CREDENTIALS_CONNECT_STORED: NET_REQUEST_WIFI_CONNECT_STORED
 * and "wifi cred auto_connect" (<zephyr/net/wifi_mgmt.h>).
 */

#define CONFIG_WIFI_CREDENTIALS_CONNECT_STORED 1

/* CONFIG_WIFI_CREDENTIALS_LOG_LEVEL: 3 (LOG_LEVEL_INF of app_log.h). */

#define CONFIG_WIFI_CREDENTIALS_LOG_LEVEL 3

#ifdef __cplusplus
extern "C" {
#endif

#define WIFI_CREDENTIALS_FLAG_BSSID       BIT(0)
#define WIFI_CREDENTIALS_FLAG_FAVORITE     BIT(1)
#define WIFI_CREDENTIALS_FLAG_2_4GHz       BIT(2)
#define WIFI_CREDENTIALS_FLAG_5GHz         BIT(3)
#define WIFI_CREDENTIALS_FLAG_6GHz         BIT(4)
#define WIFI_CREDENTIALS_FLAG_MFP_REQUIRED BIT(5)
#define WIFI_CREDENTIALS_FLAG_MFP_DISABLED BIT(6)
#define WIFI_CREDENTIALS_FLAG_ANONYMOUS_IDENTITY  BIT(7)
#define WIFI_CREDENTIALS_FLAG_KEY_PASSWORD  BIT(8)

#define WIFI_CREDENTIALS_MAX_PASSWORD_LEN                                                          \
	MAX(WIFI_PSK_MAX_LEN, CONFIG_WIFI_CREDENTIALS_SAE_PASSWORD_LENGTH)

struct wifi_credentials_header {
	/** Wi-Fi security type */
	enum wifi_security_type type;

	/** SSID (Service Set Identifier) */
	char ssid[WIFI_SSID_MAX_LEN];

	/** Length of the SSID */
	size_t ssid_len;

	/** Flags for controlling detail settings */
	uint32_t flags;

	/** Timeout for connecting to the network */
	uint32_t timeout;

	/** BSSID (Basic Service Set Identifier) */
	uint8_t bssid[WIFI_MAC_ADDR_LEN];

	/** Channel on which the network operates */
	uint8_t channel;

	/** Anonymous identifier (Limited to 16 bytes due to settings subsystem overflow) */
	char anon_id[16];

	/** Length of the Anonymous identifier */
	uint8_t aid_length;

	/** Password/PSK (Limited to 16 bytes due to settings subsystem overflow) */
	char key_passwd[16];

	/** Length of the Password */
	uint8_t key_passwd_length;
};

struct wifi_credentials_personal {
	/** Header */
	struct wifi_credentials_header header;

	/** Password/PSK */
	char password[WIFI_CREDENTIALS_MAX_PASSWORD_LEN];

	/** Length of the password */
	size_t password_len;
};

struct wifi_credentials_enterprise {
	/** Header */
	struct wifi_credentials_header header;

	/** Length of the identity */
	size_t identity_len;

	/** Length of the anonymous identity */
	size_t anonymous_identity_len;

	/** Length of the password */
	size_t password_len;

	/** Length of the CA certificate */
	size_t ca_cert_len;

	/** Length of the client certificate */
	size_t client_cert_len;

	/** Length of the private key */
	size_t private_key_len;

	/** Length of the private key password */
	size_t private_key_pw_len;
};

int wifi_credentials_get_by_ssid_personal(const char *ssid, size_t ssid_len,
					  enum wifi_security_type *type, uint8_t *bssid_buf,
					  size_t bssid_buf_len, char *password_buf,
					  size_t password_buf_len, size_t *password_len,
					  uint32_t *flags, uint8_t *channel, uint32_t *timeout);

int wifi_credentials_set_personal(const char *ssid, size_t ssid_len, enum wifi_security_type type,
				  const uint8_t *bssid, size_t bssid_len, const char *password,
				  size_t password_len, uint32_t flags, uint8_t channel,
				  uint32_t timeout);

int wifi_credentials_get_by_ssid_personal_struct(const char *ssid, size_t ssid_len,
						 struct wifi_credentials_personal *buf);

int wifi_credentials_set_personal_struct(const struct wifi_credentials_personal *creds);

int wifi_credentials_delete_by_ssid(const char *ssid, size_t ssid_len);

bool wifi_credentials_is_empty(void);

int wifi_credentials_delete_all(void);

typedef void (*wifi_credentials_ssid_cb)(void *cb_arg, const char *ssid, size_t ssid_len);

void wifi_credentials_for_each_ssid(wifi_credentials_ssid_cb cb, void *cb_arg);

/* NuttX: the library's initialization (Zephyr: SYS_INIT(init, POST_KERNEL,
 * CONFIG_APPLICATION_INIT_PRIORITY)), which reads the stored SSIDs into the
 * library's cache.  Always 0; a failure is logged, as in Zephyr.
 */

int wifi_credentials_init(void);

#ifdef __cplusplus
}
#endif

#endif /* ZEPHYR_INCLUDE_NET_WIFI_CREDENTIALS_H_ */
