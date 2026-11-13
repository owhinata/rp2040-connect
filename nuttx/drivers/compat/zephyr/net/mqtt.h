/* SPDX-License-Identifier: Apache-2.0 */
/* Zephyr's MQTT client API (<zephyr/net/mqtt.h>, Zephyr v4.5.0) for the
 * port's minimal MQTT client (nuttx/apps/imu/mqtt_min.c), with Zephyr's
 * names, types, values and meanings, as the Zephyr build configures it:
 * CONFIG_MQTT_VERSION_3_1_1=y (no MQTT 5.0), no TLS, no WebSocket, no
 * custom transport, no SOCKS, CONFIG_MQTT_KEEPALIVE=60,
 * CONFIG_MQTT_CLEAN_SESSION off.  The types are Zephyr's with the parts
 * those options remove left out; the functions are the ones that
 * imu_raw_mqtt.c uses and their helpers:
 *
 *   mqtt_client_init(), mqtt_connect(), mqtt_input(), mqtt_live(),
 *   mqtt_publish() (QoS 0 and 1; the QoS 2 encoding is the same code),
 *   mqtt_abort(), mqtt_disconnect(), mqtt_ping(),
 *   mqtt_keepalive_time_left(), mqtt_read_publish_payload*()
 *
 * Not provided: mqtt_subscribe(), mqtt_unsubscribe(),
 * mqtt_publish_qos1_ack() and the QoS 2 handshake functions,
 * mqtt_client_set_proxy(), mqtt_auth().  The types for the events of the
 * packets the library decodes (SUBACK, PUBREC, ...) are here because a
 * broker may send them; the client decodes them as Zephyr's does.
 *
 * The socket under the client is the port's thin layer (mqtt_min.h,
 * mqtt_min_sock_*()); client->transport.tcp.sock is its descriptor, as in
 * Zephyr, so the caller can poll() it.
 *
 * Differences in the types: struct mqtt_internal holds a struct k_mutex
 * (Zephyr: struct sys_mutex, which is a k_mutex without user mode),
 * struct mqtt_disconnect_param has one unused byte (Zephyr's is empty
 * without MQTT 5.0, which is not valid ISO C), and MQTT_UTF8_LITERAL()
 * casts the literal to const uint8_t *.
 *
 * Part of the small Zephyr compatibility layer of the port
 * (nuttx/drivers/compat).  Include NuttX's headers before the layer's (see
 * <zephyr/sys/atomic.h>).
 */

#ifndef __NUTTX_DRIVERS_COMPAT_ZEPHYR_NET_MQTT_H
#define __NUTTX_DRIVERS_COMPAT_ZEPHYR_NET_MQTT_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include <zephyr/kernel.h>

#ifdef __cplusplus
extern "C"
{
#endif

/* MQTT asynchronous events notified to the application through the
 * callback registered by the application.
 */

enum mqtt_evt_type
{
  /* Acknowledgment of connection request.  The result is the CONNACK
   * return code (0: accepted) or a negative errno if it was malformed.
   */

  MQTT_EVT_CONNACK,

  /* Disconnection.  The result is 0 or the negative errno that caused it. */

  MQTT_EVT_DISCONNECT,

  /* PUBLISH from the broker; only the payload size, the payload has to be
   * read with mqtt_read_publish_payload().
   */

  MQTT_EVT_PUBLISH,
  MQTT_EVT_PUBACK,    /* Acknowledgment of a QoS 1 PUBLISH. */
  MQTT_EVT_PUBREC,    /* QoS 2 */
  MQTT_EVT_PUBREL,    /* QoS 2 */
  MQTT_EVT_PUBCOMP,   /* QoS 2 */
  MQTT_EVT_SUBACK,
  MQTT_EVT_UNSUBACK,
  MQTT_EVT_PINGRESP,
};

/* MQTT version protocol level. */

enum mqtt_version
{
  MQTT_VERSION_3_1_0 = 3,
  MQTT_VERSION_3_1_1 = 4,
  MQTT_VERSION_5_0 = 5,     /* Not supported (as in the Zephyr build). */
};

/* MQTT Quality of Service types. */

enum mqtt_qos
{
  MQTT_QOS_0_AT_MOST_ONCE = 0x00,
  MQTT_QOS_1_AT_LEAST_ONCE = 0x01,
  MQTT_QOS_2_EXACTLY_ONCE = 0x02
};

/* MQTT 3.1 CONNACK return codes. */

enum mqtt_conn_return_code
{
  MQTT_CONNECTION_ACCEPTED = 0x00,
  MQTT_UNACCEPTABLE_PROTOCOL_VERSION = 0x01,
  MQTT_IDENTIFIER_REJECTED = 0x02,
  MQTT_SERVER_UNAVAILABLE = 0x03,
  MQTT_BAD_USER_NAME_OR_PASSWORD = 0x04,
  MQTT_NOT_AUTHORIZED = 0x05
};

/* MQTT SUBACK return codes. */

enum mqtt_suback_return_code
{
  MQTT_SUBACK_SUCCESS_QoS_0 = 0x00,
  MQTT_SUBACK_SUCCESS_QoS_1 = 0x01,
  MQTT_SUBACK_SUCCESS_QoS_2 = 0x02,
  MQTT_SUBACK_FAILURE = 0x80
};

/* MQTT 5.0 disconnect reason codes.  Only MQTT_DISCONNECT_NORMAL is used
 * (Zephyr's library names the type in its internal header for 3.1.1 too).
 */

enum mqtt_disconnect_reason_code
{
  MQTT_DISCONNECT_NORMAL = 0,
};

/* Abstracts UTF-8 encoded strings. */

struct mqtt_utf8
{
  const uint8_t *utf8;      /* Pointer to UTF-8 string. */
  uint32_t size;            /* Size of UTF string, in bytes. */
};

/* Initialize UTF-8 encoded string from C literal string.  (Zephyr's has no
 * cast: its build turns off -Wpointer-sign.)
 */

#define MQTT_UTF8_LITERAL(literal) \
  ((struct mqtt_utf8) {(const uint8_t *)(literal), sizeof(literal) - 1})

/* Abstracts binary strings. */

struct mqtt_binstr
{
  uint8_t *data;            /* Pointer to binary stream. */
  uint32_t len;             /* Length of binary stream. */
};

/* Abstracts MQTT UTF-8 encoded topic that can be subscribed to or
 * published.
 */

struct mqtt_topic
{
  struct mqtt_utf8 topic;   /* Topic. */
  uint8_t qos;              /* enum mqtt_qos */
};

/* Abstracts MQTT UTF-8 encoded string pair. */

struct mqtt_utf8_pair
{
  struct mqtt_utf8 name;
  struct mqtt_utf8 value;
};

/* Parameters for a publish message. */

struct mqtt_publish_message
{
  struct mqtt_topic topic;      /* Topic on which data was published. */
  struct mqtt_binstr payload;   /* Payload on the topic published. */
};

/* Parameters for a connection acknowledgment (CONNACK). */

struct mqtt_connack_param
{
  uint8_t session_present_flag;
  uint8_t return_code;          /* enum mqtt_conn_return_code */
};

/* Parameters for MQTT publish acknowledgment (PUBACK). */

struct mqtt_puback_param
{
  uint16_t message_id;
};

/* Parameters for MQTT publish receive (PUBREC). */

struct mqtt_pubrec_param
{
  uint16_t message_id;
};

/* Parameters for MQTT publish release (PUBREL). */

struct mqtt_pubrel_param
{
  uint16_t message_id;
};

/* Parameters for MQTT publish complete (PUBCOMP). */

struct mqtt_pubcomp_param
{
  uint16_t message_id;
};

/* Parameters for MQTT subscription acknowledgment (SUBACK). */

struct mqtt_suback_param
{
  uint16_t message_id;
  struct mqtt_binstr return_codes;
};

/* Parameters for MQTT unsubscribe acknowledgment (UNSUBACK). */

struct mqtt_unsuback_param
{
  uint16_t message_id;
};

/* Parameters for a publish message (PUBLISH). */

struct mqtt_publish_param
{
  struct mqtt_publish_message message;

  /* Message id used for the publish message. Redundant for QoS 0. */

  uint16_t message_id;

  /* Duplicate flag. If 1, it indicates the message is being
   * retransmitted. Has no meaning with QoS 0.
   */

  uint8_t dup_flag : 1;

  /* Retain flag. If 1, the message shall be stored persistently by the
   * broker.
   */

  uint8_t retain_flag : 1;
};

/* Parameters for subscribe/unsubscribe message (the type only:
 * mqtt_subscribe() is not provided).
 */

struct mqtt_subscription_list
{
  const struct mqtt_topic *list;
  uint16_t list_count;
  uint16_t message_id;
};

/* Parameters for disconnect message (none in MQTT 3.1.1). */

struct mqtt_disconnect_param
{
  uint8_t c;
};

union mqtt_evt_param
{
  struct mqtt_connack_param connack;
  struct mqtt_publish_param publish;
  struct mqtt_puback_param puback;
  struct mqtt_pubrec_param pubrec;
  struct mqtt_pubrel_param pubrel;
  struct mqtt_pubcomp_param pubcomp;
  struct mqtt_suback_param suback;
  struct mqtt_unsuback_param unsuback;
};

/* Defines event notification. */

struct mqtt_evt
{
  enum mqtt_evt_type type;      /* Identifies the event. */
  union mqtt_evt_param param;   /* Parameters accompanying the event. */

  /* Event result. 0 or a negative error code (errno.h) indicating reason
   * of failure.
   */

  int result;
};

struct mqtt_client;

/* Asynchronous event notification callback registered by the application. */

typedef void (*mqtt_evt_cb_t)(struct mqtt_client *client,
                              const struct mqtt_evt *evt);

/* MQTT transport type. */

enum mqtt_transport_type
{
  MQTT_TRANSPORT_NON_SECURE,    /* Non-secure TCP transport. */
  MQTT_TRANSPORT_NUM            /* Not a transport type. */
};

/* MQTT transport specific data. */

struct mqtt_transport
{
  enum mqtt_transport_type type;

  /* Name of the interface to bind to.  Leave NULL: binding is not
   * supported (mqtt_connect() fails with -ENOTSUP).
   */

  const char *if_name;

  union
  {
    struct
    {
      int sock;                 /* Socket descriptor. */
    } tcp;
  };
};

/* MQTT internal state. */

struct mqtt_internal
{
  /* Internal. Mutex to protect access to the client instance. */

  struct k_mutex mutex;

  /* Internal. Wall clock value (in milliseconds) of the last activity
   * that occurred. Needed for periodic PING.
   */

  uint32_t last_activity;

  /* Internal. Client's state in the connection. */

  uint32_t state;

  /* Internal.  Packet length read so far. */

  uint32_t rx_buf_datalen;

  /* Internal. Remaining payload length to read. */

  uint32_t remaining_payload;
};

/* MQTT Client definition to maintain information relevant to the client. */

struct mqtt_client
{
  struct mqtt_internal internal;    /* MQTT client internal state. */
  struct mqtt_transport transport;  /* Transport configuration and data. */

  /* Unique client identification to be used for the connection. */

  struct mqtt_utf8 client_id;

  /* Broker details: a struct sockaddr (struct sockaddr_in for IPv4). */

  const void *broker;

  /* User name and password (if any). NULL: none. */

  struct mqtt_utf8 *user_name;
  struct mqtt_utf8 *password;

  /* Will topic and QoS, and will message.  Can be NULL. */

  struct mqtt_topic *will_topic;
  struct mqtt_utf8 *will_message;

  /* Application callback registered with the module to get MQTT events. */

  mqtt_evt_cb_t evt_cb;

  /* Receive buffer used for MQTT packet reception in RX path. */

  uint8_t *rx_buf;
  uint32_t rx_buf_size;

  /* Transmit buffer used for creating MQTT packet in TX path. */

  uint8_t *tx_buf;
  uint32_t tx_buf_size;

  /* Keepalive interval for this client in seconds (default 60,
   * CONFIG_MQTT_KEEPALIVE of the Zephyr build).
   */

  uint16_t keepalive;

  /* MQTT protocol version. */

  uint8_t protocol_version;

  /* Unanswered PINGREQ count on this connection. */

  int8_t unacked_ping;

  /* Will retain flag, 1 if will message shall be retained persistently. */

  uint8_t will_retain : 1;

  /* Clean session flag indicating a fresh (1) or a retained session (0).
   * Default 0 (CONFIG_MQTT_CLEAN_SESSION off in the Zephyr build).
   */

  uint8_t clean_session : 1;

  /* User specific opaque data */

  void *user_data;
};

/* Initializes the client instance: zeroes it, protocol 3.1.1, clean
 * session 0, keepalive 60 s.
 */

void mqtt_client_init(struct mqtt_client *client);

/* Opens the TCP connection to client->broker and sends CONNECT.  The
 * CONNACK arrives later through mqtt_input() (MQTT_EVT_CONNACK).
 * 0 or a negative errno.
 */

int mqtt_connect(struct mqtt_client *client);

/* Sends a PUBLISH (header from the TX buffer, payload from param, in one
 * sendmsg() call when the socket takes it all).  QoS 1/2 need a nonzero
 * message_id.  -ENOTCONN before CONNACK.  0 or a negative errno; a socket
 * error closes the connection (MQTT_EVT_DISCONNECT).
 */

int mqtt_publish(struct mqtt_client *client,
                 const struct mqtt_publish_param *param);

/* Sends a PINGREQ (unacked_ping + 1). */

int mqtt_ping(struct mqtt_client *client);

/* Sends DISCONNECT and closes the connection (MQTT_EVT_DISCONNECT, 0). */

int mqtt_disconnect(struct mqtt_client *client,
                    const struct mqtt_disconnect_param *param);

/* Closes the connection without DISCONNECT (MQTT_EVT_DISCONNECT,
 * -ECONNABORTED) if it was open.  Returns 0.
 */

int mqtt_abort(struct mqtt_client *client);

/* Sends a PINGREQ if keepalive seconds have passed since the last
 * transmission: the PINGREQ's result, or -EAGAIN if none was due.
 */

int mqtt_live(struct mqtt_client *client);

/* Milliseconds until mqtt_live() would ping (0 if due), -1 without
 * keepalive.
 */

int mqtt_keepalive_time_left(const struct mqtt_client *client);

/* Reads what the socket has (without waiting) and handles a complete
 * packet: 0 (also when only part of a packet has arrived), -ENOTCONN
 * without a connection, -EBUSY while a PUBLISH payload is unread, or a
 * negative errno after which the connection is closed
 * (MQTT_EVT_DISCONNECT).
 */

int mqtt_input(struct mqtt_client *client);

/* The payload of a received PUBLISH (MQTT_EVT_PUBLISH). */

int mqtt_read_publish_payload(struct mqtt_client *client, void *buffer,
                              size_t length);
int mqtt_read_publish_payload_blocking(struct mqtt_client *client,
                                       void *buffer, size_t length);
int mqtt_readall_publish_payload(struct mqtt_client *client,
                                 uint8_t *buffer, size_t length);

#ifdef __cplusplus
}
#endif

#endif /* __NUTTX_DRIVERS_COMPAT_ZEPHYR_NET_MQTT_H */
