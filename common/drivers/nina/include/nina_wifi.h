/* SPDX-License-Identifier: Apache-2.0 */
/* The NINA-W102's Wi-Fi operations (nina_wifi.c): scan, connect,
 * disconnect, the link monitor and the status, on the module's commands
 * (nina_proto_request(), nina_proto.h) and the kernel services of nina_os.h.
 * Each OS's binding makes them its own interface:
 *
 *   common/drivers/nina/port/zephyr/nina_net.c    Zephyr's Wi-Fi management and
 *                                            network interface (also the
 *                                            FreeRTOS build's, on the
 *                                            compatibility layer)
 *   common/drivers/nina/port/nuttx/nina_netdev.c  NuttX's network device wlan0
 *                                            and its wireless ioctls (wapi)
 *
 * A binding calls nina_wifi_init() once, before any other function here,
 * and starts three threads with its OS's calls, at the priorities and
 * with the stacks of its build's table (common/compat/priorities.h):
 * nina_scan_tid runs nina_wifi_scan_thread(), nina_connect_tid
 * nina_wifi_connect_thread() and nina_monitor_tid
 * nina_wifi_monitor_thread(); none of them returns.  The threads may start
 * before nina_wifi_init(): they wait for a request (the scan and connect
 * threads) or for a connection (the monitor) first.
 *
 * What happens comes back through the binding's struct nina_wifi_ops and
 * the scan's callback.  "Under the lock" below: called with
 * nina_os_wifi_lock() held, so the callback must not call the functions
 * here.
 */

#ifndef NINA_WIFI_H_
#define NINA_WIFI_H_

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define NINA_WIFI_SSID_MAX_LEN 32
#define NINA_WIFI_PSK_MIN_LEN  8
#define NINA_WIFI_PSK_MAX_LEN  63
#define NINA_WIFI_MAC_LEN      6
#define NINA_WIFI_SCAN_MAX     10 /* the networks of one scan, at most */

/* One network of a scan, in the module's order. */
struct nina_wifi_scan_result {
	uint8_t ssid[NINA_WIFI_SSID_MAX_LEN];
	uint8_t ssid_len;
	int32_t rssi;                     /* dBm */
	uint8_t enc_type;                 /* NINA_ENC_TYPE_* (nina_proto.h), as reported */
	uint8_t bssid[NINA_WIFI_MAC_LEN]; /* in transmission order */
	uint8_t channel;
};

/* The scan's callback, in the scan thread: once with STATUS 0 for each
 * network, then once with RESULT NULL and the scan's status (0, or a
 * negative errno: -EIO the scan failed or did not start, -ECANCELED it was
 * stopped, -ETIMEDOUT it ran past NINA_SCAN_WAIT_MS, -EPROTO a bad answer,
 * or a command's error).  ARG: nina_wifi_scan()'s.  Not under the lock.
 */
typedef void (*nina_wifi_scan_cb_t)(void *arg, int status,
				    const struct nina_wifi_scan_result *result);

/* The security of a connection: an open network, WPA2-PSK, or one the
 * module cannot join (connect returns -ENOTSUP).
 */
enum nina_wifi_security {
	NINA_WIFI_SECURITY_NONE,
	NINA_WIFI_SECURITY_PSK,
	NINA_WIFI_SECURITY_OTHER,
};

struct nina_wifi_connect_params {
	const uint8_t *ssid;
	size_t ssid_len;
	enum nina_wifi_security security;
	const uint8_t *psk; /* the passphrase, for NINA_WIFI_SECURITY_PSK */
	size_t psk_len;
	bool psk_is_pbkdf2; /* a derived key instead: refused */
};

struct nina_wifi_status {
	bool connected;
	/* Only when connected: */
	uint8_t ssid[NINA_WIFI_SSID_MAX_LEN + 1]; /* NUL-terminated */
	size_t ssid_len;
	uint8_t bssid[NINA_WIFI_MAC_LEN];         /* in transmission order */
	int32_t rssi;                             /* dBm */
};

/* Why the connection ended (nina_wifi_ops.disconnected). */
enum nina_wifi_disconnect_reason {
	NINA_WIFI_DISCONNECT_SCAN,      /* a scan gave the link up (3.0.2) */
	NINA_WIFI_DISCONNECT_REQUEST,   /* nina_wifi_disconnect() */
	NINA_WIFI_DISCONNECT_LINK_LOST, /* the monitor saw the link go */
};

struct nina_wifi_ops {
	/* The connection's IPv4 configuration from the module (4 bytes each,
	 * network order; ADDR is never 0.0.0.0), when the module reports the
	 * connection: 0, or a negative errno (the connection then fails with
	 * it).  Under the lock, in the connect thread.
	 */
	int (*ip_set)(const uint8_t *addr, const uint8_t *netmask, const uint8_t *gateway);
	/* After ip_set(): the interface is on the network.  Under the lock. */
	void (*link_up)(void);
	/* The connection is gone: remove ADDR (the last ip_set()'s), the
	 * gateway, and take the interface off the network.  Under the lock,
	 * in the thread that ends the connection.
	 */
	void (*link_down)(const uint8_t *addr);
	/* The end of a connection attempt that nina_wifi_connect() started:
	 * 0 connected, or a negative errno (-ETIMEDOUT, -EIO, -EPROTO, a
	 * command's or ip_set()'s error).  Not called when a disconnection or
	 * a scan cancelled the attempt.  Under the lock, in the connect thread.
	 */
	void (*connect_result)(int status);
	/* The connection ended (after link_down(), or after a disconnection
	 * asked for while not connected).  Not under the lock.
	 */
	void (*disconnected)(enum nina_wifi_disconnect_reason reason);
};

/* Keeps OPS, reads the module's MAC address into MAC and asks the module
 * for its capabilities (nina_proto_probe_capabilities(), nina_proto.h): the
 * module's first commands, after its reset, so this waits for it (10 ms +
 * 750 ms in nina_proto.c).  0, or the MAC request's negative errno (-EPROTO
 * for a short answer); the capabilities are asked either way.
 */
int nina_wifi_init(const struct nina_wifi_ops *ops, uint8_t mac[NINA_WIFI_MAC_LEN]);

/* The application's reconnection, if it has one (none: every connection is
 * allowed and nothing more is done).  connection_allowed(): in
 * nina_wifi_connect()'s caller, under the lock; false refuses the
 * connection (-ECANCELED).  pause(): nina_wifi_disconnect(), under the
 * lock.  request(): after a scan that gave the link up, and after a lost
 * link, not under the lock.  Any of them may be NULL.
 */
struct nina_wifi_reconnect_ops {
	bool (*connection_allowed)(void);
	void (*pause)(void);
	void (*request)(void);
};

/* Keeps OPS (NULL: none) from now on.  At any time, from a thread: the
 * driver's threads read the registration atomically, each call of a hook
 * through the one it read.  A hook of an earlier registration may
 * therefore still be running, or start, a moment after this returns, so
 * every OPS and its functions must stay valid for good (static).
 */
void nina_wifi_set_reconnect_ops(const struct nina_wifi_reconnect_ops *ops);

/* Starts a scan in the scan thread; the results come to CB with ARG.
 * Without NINA_CAP_SCAN_KEEPS_LINK a connection is given up first:
 * link_down() and disconnected(NINA_WIFI_DISCONNECT_SCAN) here, and
 * reconnect_request() after the scan.  0, -EINVAL (CB NULL), -EBUSY (a
 * scan or a connection attempt is in progress).
 */
int nina_wifi_scan(nina_wifi_scan_cb_t cb, void *arg);

/* Starts a connection: checks PARAMS (-EINVAL: no SSID, more than 32
 * bytes, a passphrase of other than 8 to 63 bytes or a derived key;
 * -ENOTSUP: neither open nor WPA2-PSK), asks connection_allowed()
 * (-ECANCELED), refuses while connected (-EALREADY) or during a scan or
 * another attempt (-EBUSY), then gives the module the network (-EIO when
 * it refuses, or the command's error).  0: the connect thread waits for
 * the connection (up to 60 checks 500 ms apart) and reports it with
 * connect_result().
 */
int nina_wifi_connect(const struct nina_wifi_connect_params *params);

/* Ends the connection or the attempt: reconnect_pause(), the module's
 * disconnection (-EIO when it refuses, or the command's error), then
 * link_down() if connected and disconnected(NINA_WIFI_DISCONNECT_REQUEST).
 * 0 or a negative errno.
 */
int nina_wifi_disconnect(void);

/* Asks the module (its connection state, and when connected the SSID,
 * BSSID and RSSI).  0, -EPROTO for a bad answer, or a command's error.
 */
int nina_wifi_status(struct nina_wifi_status *status);

/* The firmware's version, a string ("3.0.2"), into VERSION.  0, or the
 * request's negative errno.
 */
#define NINA_WIFI_FW_VERSION_SIZE 16

int nina_wifi_fw_version(char version[NINA_WIFI_FW_VERSION_SIZE]);

/* The module's capability bits into *MODULE and those the driver uses into
 * *IN_USE (internal/nina_proto.h: NINA_CAP_*); both 0 before
 * nina_wifi_init().
 */
void nina_wifi_capabilities(uint32_t *module, uint32_t *in_use);

/* The threads' bodies (see above). */
void nina_wifi_scan_thread(void);
void nina_wifi_connect_thread(void);
void nina_wifi_monitor_thread(void);

#endif /* NINA_WIFI_H_ */
