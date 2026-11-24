/* SPDX-License-Identifier: Apache-2.0 */
/* Zephyr's <zephyr/net/wifi_credentials.h> (Zephyr v4.5.0): the stored
 * Wi-Fi credentials, with Zephyr's names, types, flags and functions.  The
 * library is the copy of Zephyr's subsys/net/lib/wifi_credentials/ in
 * common/compat (wifi_credentials.c, wifi_credentials_backend_settings.c;
 * CONFIG_WIFI_CREDENTIALS), which keeps the entries in the copy of
 * Zephyr's settings (common/compat/subsys/settings, on NVS in the key-value
 * area) under the names "wifi_cred/<index>", as the Zephyr build keeps them
 * in its settings.  The entry is this header's struct
 * wifi_credentials_personal, stored as it is in memory, as Zephyr's
 * settings backend stores it, in the same bytes as the Zephyr build (see
 * the layout check below).
 *
 * The configuration (CONFIG_WIFI_CREDENTIALS_*, the Zephyr build's names)
 * is the OS's where it sets one (NuttX: the options of
 * nuttx/apps/wifi/Kconfig), defined before this header, else the Zephyr
 * build's value below.
 *
 * Not in Zephyr: wifi_credentials_init(), which the port's init task calls
 * where Zephyr runs the library's SYS_INIT() (before the threads and
 * main()).
 *
 * Part of the small Zephyr compatibility layer of the port
 * (common/compat, see common/compat/README.md).
 */

/*
 * Copyright (c) 2024 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_INCLUDE_NET_WIFI_CREDENTIALS_H_
#define ZEPHYR_INCLUDE_NET_WIFI_CREDENTIALS_H_

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include <zephyr/kernel.h>
#include <zephyr/net/wifi_mgmt.h>

/* The Zephyr build's configuration (zephyr/build/usb-console/zephyr/.config)
 * where the OS does not set it.
 */

#ifndef CONFIG_WIFI_CREDENTIALS_MAX_ENTRIES
#  define CONFIG_WIFI_CREDENTIALS_MAX_ENTRIES 2
#endif

#ifndef CONFIG_WIFI_CREDENTIALS_SAE_PASSWORD_LENGTH
#  define CONFIG_WIFI_CREDENTIALS_SAE_PASSWORD_LENGTH 128
#endif

#ifndef CONFIG_WIFI_CREDENTIALS_CONNECT_STORED_CONNECTION_TIMEOUT
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

/* Port: the entry is stored as it is in memory, so its layout must be the
 * Zephyr build's (from its zephyr.elf, "ptype /o" of GDB): 224 bytes, the
 * enum of one byte (arm-none-eabi's short enums, which every build for the
 * board has), ssid at 1, ssid_len at 36, password at 92, password_len at
 * 220.  Host builds have 4-byte enums and are not checked.
 */
#if defined(__arm__) && CONFIG_WIFI_CREDENTIALS_SAE_PASSWORD_LENGTH == 128
BUILD_ASSERT(sizeof(enum wifi_security_type) == 1, "enum size differs from Zephyr's");
BUILD_ASSERT(sizeof(struct wifi_credentials_personal) == 224, "entry size differs");
BUILD_ASSERT(offsetof(struct wifi_credentials_header, ssid) == 1, "layout differs");
BUILD_ASSERT(offsetof(struct wifi_credentials_header, ssid_len) == 36, "layout differs");
BUILD_ASSERT(offsetof(struct wifi_credentials_header, flags) == 40, "layout differs");
BUILD_ASSERT(offsetof(struct wifi_credentials_header, timeout) == 44, "layout differs");
BUILD_ASSERT(offsetof(struct wifi_credentials_header, bssid) == 48, "layout differs");
BUILD_ASSERT(offsetof(struct wifi_credentials_header, channel) == 54, "layout differs");
BUILD_ASSERT(offsetof(struct wifi_credentials_header, anon_id) == 55, "layout differs");
BUILD_ASSERT(offsetof(struct wifi_credentials_header, aid_length) == 71, "layout differs");
BUILD_ASSERT(offsetof(struct wifi_credentials_header, key_passwd) == 72, "layout differs");
BUILD_ASSERT(offsetof(struct wifi_credentials_header, key_passwd_length) == 88,
	     "layout differs");
BUILD_ASSERT(offsetof(struct wifi_credentials_personal, password) == 92, "layout differs");
BUILD_ASSERT(offsetof(struct wifi_credentials_personal, password_len) == 220,
	     "layout differs");
#endif

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

/* Port: the library's initialization (Zephyr: SYS_INIT(init, POST_KERNEL,
 * CONFIG_APPLICATION_INIT_PRIORITY)), which reads the stored SSIDs into the
 * library's cache.  Always 0; a failure is logged, as in Zephyr.
 */

int wifi_credentials_init(void);

#ifdef __cplusplus
}
#endif

#endif /* ZEPHYR_INCLUDE_NET_WIFI_CREDENTIALS_H_ */
