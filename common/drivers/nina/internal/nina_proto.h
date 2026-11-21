/* SPDX-License-Identifier: Apache-2.0 */
/* The bytes of the NINA-W102 SPI protocol that the driver uses: the
 * framing, the commands it sends and the values of their replies.
 *
 * The names are the firmware's: the Arduino WiFiNINA library's wifi_spi.h
 * and wl_definitions.h (SET_NET_CMD, ENC_TYPE_WEP, ...) where it names
 * them, else the handler in the firmware's command table
 * (main/CommandHandler.cpp of Arduino nina-fw 3.0.2: socket_connect,
 * getCapabilities, ...) and its WiFi.h (NINA_SCAN_*, added by
 * firmware/nina-fw/patches/0005).  The commands 0x6A and 0x6B and the
 * changed 0x36 are those of the patches 0003 to 0006
 * (firmware/nina-fw/README.md, "Host interface of 0003 to 0006").
 *
 * After them, the functions of nina_proto.c that send a command and read its
 * reply, and the module's capabilities.
 */

#ifndef NINA_PROTO_H_
#define NINA_PROTO_H_

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/* Framing (the firmware's CommandHandlerClass::handle() has them as bare
 * numbers): a command is NINA_START, the command, the number of
 * parameters, each parameter with its length, NINA_END, padded with
 * NINA_DUMMY to 4 bytes; its reply starts with NINA_START and the command
 * | NINA_REPLY.  A firmware without the command replies NINA_ERR 0x00
 * NINA_END instead.
 */
#define NINA_START 0xe0
#define NINA_END   0xee
#define NINA_ERR   0xef
#define NINA_DUMMY 0xff /* padding, and the bytes clocked out to read */
#define NINA_REPLY 0x80

/* Wi-Fi. */
#define NINA_CMD_SET_NET         0x10 /* join an open network */
#define NINA_CMD_SET_PASSPHRASE  0x11 /* join a WPA2-PSK network */
#define NINA_CMD_SET_POWER_MODE  0x17
#define NINA_CMD_GET_CONN_STATUS 0x20 /* NINA_WL_* */
#define NINA_CMD_GET_IPADDR      0x21 /* address, netmask and gateway */
#define NINA_CMD_GET_MACADDR     0x22
#define NINA_CMD_GET_CURR_SSID   0x23
#define NINA_CMD_GET_CURR_BSSID  0x24
#define NINA_CMD_GET_CURR_RSSI   0x25
#define NINA_CMD_DISCONNECT      0x30

/* Scans: the networks of the last scan by index. */
#define NINA_CMD_SCAN_NETWORKS       0x27 /* one parameter per network: its SSID */
#define NINA_CMD_GET_IDX_RSSI        0x32
#define NINA_CMD_GET_IDX_ENCT        0x33 /* NINA_ENC_TYPE_* */
#define NINA_CMD_START_SCAN_NETWORKS 0x36 /* 0005: starts a scan, returns at once */
#define NINA_CMD_GET_IDX_BSSID       0x3c
#define NINA_CMD_GET_IDX_CHANNEL     0x3d
#define NINA_CMD_GET_SCAN_STATE      0x6b /* 0005: NINA_SCAN_* */

/* The resolver. */
#define NINA_CMD_REQ_HOST_BY_NAME 0x34 /* starts it with the name */
#define NINA_CMD_GET_HOST_BY_NAME 0x35 /* reads the address */

/* The module. */
#define NINA_CMD_GET_FW_VERSION   0x37
#define NINA_CMD_GET_CAPABILITIES 0x6a /* 0003: NINA_CAP_* */

/* The BSD-style sockets (0x70 to 0x7F). */
#define NINA_CMD_SOCKET_SOCKET     0x70
#define NINA_CMD_SOCKET_CLOSE      0x71
#define NINA_CMD_SOCKET_BIND       0x73
#define NINA_CMD_SOCKET_LISTEN     0x74
#define NINA_CMD_SOCKET_ACCEPT     0x75
#define NINA_CMD_SOCKET_CONNECT    0x76
#define NINA_CMD_SOCKET_SEND       0x77
#define NINA_CMD_SOCKET_RECV       0x78
#define NINA_CMD_SOCKET_SENDTO     0x79
#define NINA_CMD_SOCKET_RECVFROM   0x7a
#define NINA_CMD_SOCKET_IOCTL      0x7b
#define NINA_CMD_SOCKET_POLL       0x7c /* NINA_POLL_* (nina_sock.h) */
#define NINA_CMD_SOCKET_GETSOCKOPT 0x7e

/* NINA_CMD_GET_CONN_STATUS: wl_status_t (the driver tells only this one). */
#define NINA_WL_CONNECTED 3

/* NINA_CMD_GET_IDX_ENCT: wl_enc_type. */
#define NINA_ENC_TYPE_TKIP 2 /* WPA */
#define NINA_ENC_TYPE_CCMP 4 /* WPA2 */
#define NINA_ENC_TYPE_WEP  5
#define NINA_ENC_TYPE_NONE 7

/* NINA_CMD_GET_SCAN_STATE. */
#define NINA_SCAN_IDLE    0 /* none started, or its results were taken */
#define NINA_SCAN_RUNNING 1
#define NINA_SCAN_DONE    2
#define NINA_SCAN_FAILED  3 /* could not start */

/* The requests (nina_proto.c) ----------------------------------------------- */

struct nina_proto_param {
	const uint8_t *data;
	size_t len;
};
int nina_proto_request(uint8_t cmd, const struct nina_proto_param *params, size_t nparams,
		 bool wide_params, bool wide_response, uint8_t *out, size_t cap,
		 size_t *out_len);
int nina_proto_ip_config(uint8_t data[12]);

/* The module's capabilities (command 0x6A of firmware/nina-fw's patches
 * 0003 to 0006; firmware/nina-fw/README.md): bits of what it adds to the
 * Arduino firmware 3.0.2.
 */
#define NINA_CAP_SCAN_KEEPS_LINK (1u << 0) /* a scan while connected keeps the link */
#define NINA_CAP_ASYNC_SCAN      (1u << 1) /* 0x36 starts a scan, 0x6B reports it */
#define NINA_CAP_SOCKET_EVENTS   (1u << 2) /* socket events on the IRQ line */
#define NINA_CAP_KNOWN           0x7u

/* nina_proto_probe_capabilities() asks the module once (nina_wifi_init() of
 * nina_wifi.c, after the module's reset), starts the OS's waits for the socket events
 * when the module has them (nina_wait.c), and returns the bits in use.  A
 * firmware without the command (it answers 0xEF: -ENOTSUP of
 * nina_proto_request()) or a failed request gives 0: the driver then works as
 * with 3.0.2.  Later calls return the first answer.
 * nina_proto_capabilities() returns the bits in use: the module's known bits,
 * without NINA_CAP_SOCKET_EVENTS when the OS does not wait for the events
 * (nina_os_event_start() failed); 0 before the probe.
 * nina_proto_module_capabilities() returns the module's own bits (0 before).
 */
uint32_t nina_proto_probe_capabilities(void);
uint32_t nina_proto_capabilities(void);
uint32_t nina_proto_module_capabilities(void);

#endif /* NINA_PROTO_H_ */
