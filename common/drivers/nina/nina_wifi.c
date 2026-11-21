/* SPDX-License-Identifier: Apache-2.0 */
/* The NINA-W102's Wi-Fi operations (nina_wifi.h), on the module's commands
 * and the kernel services of nina_os.h.  The state of the connection is
 * under nina_os_wifi_lock(); the values that the threads also read
 * without it (the generation, the generation of the last request, whether
 * a scan runs) are C11 atomics, of which only loads and stores are used:
 * every change that reads the value first is made under the lock.
 */

#include <errno.h>
#include <stdatomic.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

#include "nina_os.h"
#include "nina_proto.h"
#include "nina_wifi.h"

/* The asynchronous scan of the patch 0005 (NINA_CAP_ASYNC_SCAN,
 * firmware/nina-fw/README.md): 0x36 (NINA_CMD_START_SCAN_NETWORKS) starts
 * it, 0x6B (NINA_CMD_GET_SCAN_STATE) is asked every NINA_SCAN_POLL_MS
 * until it reports the scan done, then 0x27 (NINA_CMD_SCAN_NETWORKS) takes the
 * results.  The NINA lock is held for each command only, so the sockets'
 * commands go on during the scan, where the 0x27 of 3.0.2 holds it until
 * the scan ends.  A scan still running after NINA_SCAN_WAIT_MS ends with
 * -ETIMEDOUT, without 0x27 (which would wait for it).
 */
#define NINA_SCAN_POLL_MS 100
#ifndef NINA_SCAN_WAIT_MS
#define NINA_SCAN_WAIT_MS 10000
#endif

/* The connect thread asks for the connection state up to
 * NINA_CONNECT_CHECKS times, NINA_CONNECT_POLL_MS apart; the monitor asks
 * every NINA_MONITOR_MS while connected, and a link reported down
 * NINA_MONITOR_LOST_CHECKS times in a row is lost.
 */
#define NINA_CONNECT_CHECKS      60
#define NINA_CONNECT_POLL_MS     500
#define NINA_MONITOR_MS          2000
#define NINA_MONITOR_LOST_CHECKS 2

static const struct nina_wifi_ops *ops;
/* Read by the threads while the application may register: atomic. */
static _Atomic(const struct nina_wifi_reconnect_ops *) reconnect_ops;

void nina_wifi_set_reconnect_ops(const struct nina_wifi_reconnect_ops *hooks)
{
	atomic_store(&reconnect_ops, hooks);
}

static void reconnect_request(void)
{
	const struct nina_wifi_reconnect_ops *hooks = atomic_load(&reconnect_ops);

	if (hooks && hooks->request) {
		hooks->request();
	}
}
static uint8_t assigned_ip[4];
static bool connected;
static bool connecting;
static atomic_long generation;
static atomic_long request_generation;
static atomic_bool scanning;
static nina_wifi_scan_cb_t scan_cb;
static void *scan_arg;
static bool scan_reconnect;
static unsigned int disconnected_checks;

static int request_one(uint8_t cmd, const uint8_t *data, size_t length, uint8_t *result,
		       size_t cap, size_t *n)
{
	const struct nina_proto_param p = {data, length};

	return nina_proto_request(cmd, data ? &p : NULL, data ? 1 : 0, false, false, result, cap, n);
}

static int32_t get_le32(const uint8_t *p)
{
	return (int32_t)((uint32_t)p[0] | (uint32_t)p[1] << 8 | (uint32_t)p[2] << 16 |
			 (uint32_t)p[3] << 24);
}

/* Under the lock (the only writers). */
static void next_generation(void)
{
	atomic_store(&generation, atomic_load(&generation) + 1);
}

/* 0x36, then 0x6B until done (see NINA_SCAN_POLL_MS): 0 when 0x27 may
 * take the results; -EIO when the scan could not start or failed (0x36
 * answered 0, or 0x6B 3), -ECANCELED when it was stopped (0x6B 0: a
 * connection, a disconnection or an AP stops a running scan),
 * -ETIMEDOUT when it still runs after NINA_SCAN_WAIT_MS, or the error of
 * the command.
 */
static int scan_async(void)
{
	int64_t deadline = nina_os_uptime_ms() + NINA_SCAN_WAIT_MS;
	uint8_t state;
	size_t n;
	int ret;

	ret = request_one(NINA_CMD_START_SCAN_NETWORKS, NULL, 0, &state, 1, &n);
	if (ret || n != 1) {
		return ret ? ret : -EPROTO;
	}
	if (state != 1) { /* not started */
		return -EIO;
	}
	for (;;) {
		nina_os_sleep_ms(NINA_SCAN_POLL_MS);
		ret = request_one(NINA_CMD_GET_SCAN_STATE, NULL, 0, &state, 1, &n);
		if (ret || n != 1) {
			return ret ? ret : -EPROTO;
		}
		switch (state) {
		case NINA_SCAN_RUNNING:
			break;
		case NINA_SCAN_DONE:
			return 0;
		case NINA_SCAN_IDLE: /* stopped */
			return -ECANCELED;
		default: /* NINA_SCAN_FAILED */
			return -EIO;
		}
		if (nina_os_uptime_ms() >= deadline) {
			return -ETIMEDOUT;
		}
	}
}

/* One request of the scan thread; RECORDS takes the answer of 0x27. */
static void scan_once(uint8_t *records, size_t cap)
{
	size_t len, offset = 0, n;
	uint8_t index = 0;
	nina_wifi_scan_cb_t cb;
	void *arg;
	bool reconnect;
	int ret;

	nina_os_wifi_wait(NINA_OS_WIFI_SCAN);
	cb = scan_cb;
	arg = scan_arg;
	reconnect = scan_reconnect;
	ret = (nina_proto_capabilities() & NINA_CAP_ASYNC_SCAN) ? scan_async() : 0;
	if (!ret) {
		ret = nina_proto_request(NINA_CMD_SCAN_NETWORKS, NULL, 0, false, false, records, cap,
				   &len);
	}
	while (!ret && offset < len) {
		struct nina_wifi_scan_result result = {0};
		uint8_t value[6];
		size_t ssid_len = records[offset++];

		if (ssid_len > NINA_WIFI_SSID_MAX_LEN || ssid_len > len - offset ||
		    index >= NINA_WIFI_SCAN_MAX) {
			ret = -EPROTO;
			break;
		}
		memcpy(result.ssid, records + offset, ssid_len);
		result.ssid_len = ssid_len;
		offset += ssid_len;
		ret = request_one(NINA_CMD_GET_IDX_RSSI, &index, 1, value, sizeof(value), &n);
		if (ret || n != 4) {
			ret = ret ? ret : -EPROTO;
			break;
		}
		result.rssi = get_le32(value);
		ret = request_one(NINA_CMD_GET_IDX_ENCT, &index, 1, value, sizeof(value), &n);
		if (ret || n != 1) {
			ret = ret ? ret : -EPROTO;
			break;
		}
		result.enc_type = value[0];
		ret = request_one(NINA_CMD_GET_IDX_BSSID, &index, 1, value, sizeof(value), &n);
		if (ret || n != NINA_WIFI_MAC_LEN) {
			ret = ret ? ret : -EPROTO;
			break;
		}
		/* NINA's Arduino firmware reports this address in reverse byte order. */
		for (size_t i = 0; i < NINA_WIFI_MAC_LEN; i++) {
			result.bssid[i] = value[NINA_WIFI_MAC_LEN - 1 - i];
		}
		ret = request_one(NINA_CMD_GET_IDX_CHANNEL, &index, 1, value, sizeof(value), &n);
		if (ret || n != 1) {
			ret = ret ? ret : -EPROTO;
			break;
		}
		result.channel = value[0];
		cb(arg, 0, &result);
		index++;
	}
	atomic_store(&scanning, false);
	cb(arg, ret, NULL);
	if (reconnect) {
		reconnect_request();
	}
}

void nina_wifi_scan_thread(void)
{
	uint8_t records[NINA_WIFI_SCAN_MAX * (NINA_WIFI_SSID_MAX_LEN + 1)];

	for (;;) {
		scan_once(records, sizeof(records));
	}
}

int nina_wifi_scan(nina_wifi_scan_cb_t cb, void *arg)
{
	bool was_connected;

	if (!cb) {
		return -EINVAL;
	}
	nina_os_wifi_lock();
	if (connecting || atomic_load(&scanning)) {
		nina_os_wifi_unlock();
		return -EBUSY;
	}
	atomic_store(&scanning, true);
	/* 3.0.2 stops Wi-Fi for a scan, so the link is given up here and taken
	 * up again after it.  A module that keeps the link through a scan
	 * (NINA_CAP_SCAN_KEEPS_LINK) stays connected: the IP configuration
	 * stays, and the monitor sees a loss during the scan as any other.
	 */
	was_connected = connected && !(nina_proto_capabilities() & NINA_CAP_SCAN_KEEPS_LINK);
	if (was_connected) {
		next_generation();
		connected = false;
		ops->link_down(assigned_ip);
	}
	scan_cb = cb;
	scan_arg = arg;
	scan_reconnect = was_connected;
	nina_os_wifi_unlock();
	if (was_connected) {
		ops->disconnected(NINA_WIFI_DISCONNECT_SCAN);
	}
	nina_os_wifi_post(NINA_OS_WIFI_SCAN);
	return 0;
}

/* One request of the connect thread. */
static void connect_once(void)
{
	uint8_t value, ip[12];
	size_t n;
	int ret = -ETIMEDOUT;
	long id;

	nina_os_wifi_wait(NINA_OS_WIFI_CONNECT);
	id = atomic_load(&request_generation);

	for (int i = 0; i < NINA_CONNECT_CHECKS && id == atomic_load(&generation); i++) {
		ret = request_one(NINA_CMD_GET_CONN_STATUS, NULL, 0, &value, 1, &n);
		if (ret || n != 1) {
			if (!ret) {
				ret = -EPROTO;
			}
			break;
		}
		if (value == NINA_WL_CONNECTED) {
			uint8_t no_power_save = 0, power_reply;

			/* Reapply after every association, including reconnects. */
			ret = request_one(NINA_CMD_SET_POWER_MODE, &no_power_save, 1, &power_reply,
					  1, &n);
			if (ret || n != 1 || power_reply != 1) {
				ret = ret ? ret : -EIO;
				break;
			}
			ret = nina_proto_ip_config(ip);
			if (ret) {
				break;
			}
			if (ip[0] || ip[1] || ip[2] || ip[3]) {
				nina_os_wifi_lock();
				if (id != atomic_load(&generation)) {
					nina_os_wifi_unlock();
					ret = -ECANCELED;
					break;
				}
				memcpy(assigned_ip, ip, 4);
				ret = ops->ip_set(assigned_ip, ip + 4, ip + 8);
				if (ret) {
					nina_os_wifi_unlock();
					break;
				}
				connected = true;
				ops->link_up();
				nina_os_wifi_unlock();
				ret = 0;
				break;
			}
		}
		/* NINA can report the previous failure or NO_SSID while
		 * the new asynchronous connection attempt is still
		 * starting. */
		ret = -ETIMEDOUT;
		nina_os_sleep_ms(NINA_CONNECT_POLL_MS);
	}

	nina_os_wifi_lock();
	if (id == atomic_load(&generation)) {
		ops->connect_result(ret);
	}
	connecting = false;
	nina_os_wifi_unlock();
}

void nina_wifi_connect_thread(void)
{
	for (;;) {
		connect_once();
	}
}

int nina_wifi_connect(const struct nina_wifi_connect_params *params)
{
	const struct nina_wifi_reconnect_ops *hooks;
	bool open;
	uint8_t answer;
	size_t n;
	struct nina_proto_param args[2];
	int ret;

	if (!params || !params->ssid || !params->ssid_len ||
	    params->ssid_len > NINA_WIFI_SSID_MAX_LEN) {
		return -EINVAL;
	}
	if (params->security != NINA_WIFI_SECURITY_NONE &&
	    params->security != NINA_WIFI_SECURITY_PSK) {
		return -ENOTSUP;
	}
	open = params->security == NINA_WIFI_SECURITY_NONE;
	if (!open && (!params->psk || params->psk_len < NINA_WIFI_PSK_MIN_LEN ||
		      params->psk_len > NINA_WIFI_PSK_MAX_LEN || params->psk_is_pbkdf2)) {
		return -EINVAL;
	}
	nina_os_wifi_lock();
	hooks = atomic_load(&reconnect_ops);
	if (hooks && hooks->connection_allowed && !hooks->connection_allowed()) {
		nina_os_wifi_unlock();
		return -ECANCELED;
	}
	if (connected) {
		nina_os_wifi_unlock();
		return -EALREADY;
	}
	if (atomic_load(&scanning) || connecting) {
		nina_os_wifi_unlock();
		return -EBUSY;
	}
	connecting = true;

	args[0] = (struct nina_proto_param){params->ssid, params->ssid_len};
	args[1] = (struct nina_proto_param){params->psk, params->psk_len};
	next_generation();
	atomic_store(&request_generation, atomic_load(&generation));
	ret = nina_proto_request(open ? NINA_CMD_SET_NET : NINA_CMD_SET_PASSPHRASE, args, open ? 1 : 2,
			   false, false, &answer, 1, &n);
	if (ret || n != 1 || answer != 1) {
		connecting = false;
		nina_os_wifi_unlock();
		return ret ? ret : -EIO;
	}

	nina_os_wifi_post(NINA_OS_WIFI_CONNECT);
	nina_os_wifi_unlock();
	return 0;
}

int nina_wifi_disconnect(void)
{
	const struct nina_wifi_reconnect_ops *hooks;
	uint8_t dummy = NINA_DUMMY, answer;
	size_t n;
	int ret;

	nina_os_wifi_lock();
	hooks = atomic_load(&reconnect_ops);
	if (hooks && hooks->pause) {
		hooks->pause();
	}
	next_generation();
	ret = request_one(NINA_CMD_DISCONNECT, &dummy, 1, &answer, 1, &n);
	if (ret || n != 1 || answer != 1) {
		nina_os_wifi_unlock();
		return ret ? ret : -EIO;
	}
	if (connected) {
		ops->link_down(assigned_ip);
	}

	connected = false;
	nina_os_wifi_unlock();
	ops->disconnected(NINA_WIFI_DISCONNECT_REQUEST);
	return 0;
}

/* One pass of the monitor. */
static void monitor_once(void)
{
	uint8_t state;
	size_t n;
	int ret;
	bool lost = false;

	nina_os_sleep_ms(NINA_MONITOR_MS);
	nina_os_wifi_lock();
	if (!connected) {
		disconnected_checks = 0;
		nina_os_wifi_unlock();
		return;
	}

	ret = request_one(NINA_CMD_GET_CONN_STATUS, NULL, 0, &state, 1, &n);
	if (!ret && n == 1) {
		if (state == NINA_WL_CONNECTED) {
			disconnected_checks = 0;
		} else if (++disconnected_checks >= NINA_MONITOR_LOST_CHECKS) {
			next_generation();
			connected = false;
			ops->link_down(assigned_ip);
			lost = true;
			disconnected_checks = 0;
		}
	}
	nina_os_wifi_unlock();

	if (lost) {
		ops->disconnected(NINA_WIFI_DISCONNECT_LINK_LOST);
		reconnect_request();
	}
}

void nina_wifi_monitor_thread(void)
{
	for (;;) {
		monitor_once();
	}
}

int nina_wifi_status(struct nina_wifi_status *status)
{
	uint8_t state, dummy = NINA_DUMMY;
	uint8_t rssi[4], bssid[NINA_WIFI_MAC_LEN];
	size_t n;
	int ret;

	ret = request_one(NINA_CMD_GET_CONN_STATUS, NULL, 0, &state, 1, &n);
	if (ret || n != 1) {
		return ret ? ret : -EPROTO;
	}
	status->connected = state == NINA_WL_CONNECTED;
	if (!status->connected) {
		return 0;
	}

	ret = request_one(NINA_CMD_GET_CURR_SSID, &dummy, 1, status->ssid, NINA_WIFI_SSID_MAX_LEN,
			  &n);
	if (ret) {
		return ret;
	}
	status->ssid_len = n;
	status->ssid[n] = '\0';

	ret = request_one(NINA_CMD_GET_CURR_BSSID, &dummy, 1, bssid, sizeof(bssid), &n);
	if (ret || n != sizeof(bssid)) {
		return ret ? ret : -EPROTO;
	}
	/* NINA's Arduino firmware reports this address in reverse byte order.
	 */
	for (size_t i = 0; i < sizeof(bssid); i++) {
		status->bssid[i] = bssid[sizeof(bssid) - 1 - i];
	}

	ret = request_one(NINA_CMD_GET_CURR_RSSI, &dummy, 1, rssi, sizeof(rssi), &n);
	if (ret || n != sizeof(rssi)) {
		return ret ? ret : -EPROTO;
	}
	status->rssi = get_le32(rssi);
	return 0;
}

int nina_wifi_fw_version(char version[NINA_WIFI_FW_VERSION_SIZE])
{
	size_t len;
	int ret;

	ret = nina_proto_request(NINA_CMD_GET_FW_VERSION, NULL, 0, false, false, (uint8_t *)version,
				 NINA_WIFI_FW_VERSION_SIZE - 1, &len);
	if (ret) {
		return ret;
	}
	version[len] = '\0';
	return 0;
}

void nina_wifi_capabilities(uint32_t *module, uint32_t *in_use)
{
	*module = nina_proto_module_capabilities();
	*in_use = nina_proto_capabilities();
}

int nina_wifi_init(const struct nina_wifi_ops *wifi_ops, uint8_t mac[NINA_WIFI_MAC_LEN])
{
	uint8_t dummy = NINA_DUMMY;
	size_t n = 0;
	int ret;

	ops = wifi_ops;
	ret = request_one(NINA_CMD_GET_MACADDR, &dummy, 1, mac, NINA_WIFI_MAC_LEN, &n);
	if (!ret && n != NINA_WIFI_MAC_LEN) {
		ret = -EPROTO;
	}
	/* Once, after the module's reset (the first request): what the
	 * firmware adds to 3.0.2 (nina_proto.h). */
	(void)nina_proto_probe_capabilities();
	return ret;
}
