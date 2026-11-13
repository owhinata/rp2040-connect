/* SPDX-License-Identifier: Apache-2.0 */
/*
 * Copyright (c) 2018 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/* The port's minimal MQTT client: the part of Zephyr's MQTT library
 * (subsys/net/lib/mqtt/, Zephyr v4.5.0) that the IMU service
 * (imu_raw_mqtt.c) uses, copied, as the Zephyr build configures it
 * (CONFIG_MQTT_VERSION_3_1_1=y, no MQTT 5.0, TLS, WebSocket, custom
 * transport or SOCKS; CONFIG_MQTT_KEEPALIVE=60; CONFIG_MQTT_CLEAN_SESSION
 * off; CONFIG_MQTT_LOG_LEVEL=0, so no logging).  The API is Zephyr's,
 * <zephyr/net/mqtt.h> of the compatibility layer.
 *
 * The sections below are, in order, copies of mqtt_internal.h, mqtt_os.h,
 * mqtt_encoder.c, mqtt_decoder.c, mqtt_transport.c with
 * mqtt_transport_socket_tcp.c, mqtt_rx.c and mqtt.c.  The code is Zephyr's
 * text with these changes only:
 *
 * - The functions shared between Zephyr's files are static here (one
 *   file); the logging calls (NET_DBG(), ...) are left out (log level 0
 *   in the Zephyr build compiles them out).
 * - The branches for MQTT 5.0 (mqtt_is_version_5_0(), always false in the
 *   Zephyr build) and the functions that only those branches call are
 *   left out; so are the functions of the API not provided
 *   (mqtt_subscribe(), mqtt_unsubscribe(), mqtt_publish_qos1_ack(), the
 *   QoS 2 handshake, mqtt_client_set_proxy(), mqtt_auth()) and their
 *   encoders.
 * - The transport table has the TCP transport only, called directly.  Its
 *   socket calls are the port's socket layer (mqtt_min.h,
 *   mqtt_min_sock_*() for zsock_*()).  Binding to an interface
 *   (transport.if_name, SO_BINDTODEVICE) is not supported: -ENOTSUP.
 * - The mutex is the compatibility layer's k_mutex (Zephyr's sys_mutex is
 *   a k_mutex without user mode); the millisecond clock is k_uptime_get()
 *   truncated to 32 bits (Zephyr's k_uptime_get_32()).
 */

#ifdef __NuttX__
#  include <nuttx/config.h>
#endif

#include <sys/types.h>
#include <sys/socket.h>
#include <sys/uio.h>
#include <netinet/in.h>

#include <errno.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include <zephyr/kernel.h>
#include <zephyr/net/mqtt.h>
#include <zephyr/sys/byteorder.h>

#include "mqtt_min.h"

/* mqtt_internal.h ===========================================================*/

/**@brief Keep alive time for MQTT (in seconds). Sending of Ping Requests to
 *        keep the connection alive are governed by this value.
 */
#define MQTT_KEEPALIVE MQTT_MIN_KEEPALIVE

/**@brief Clean session on every connect (1) or keep subscriptions and messages
 *        between connects (0)
 */
#define MQTT_CLEAN_SESSION MQTT_MIN_CLEAN_SESSION

/**@brief Minimum mandatory size of fixed header. */
#define MQTT_FIXED_HEADER_MIN_SIZE 2

/**@brief Maximum size of the fixed header. Remaining length size is 4 in this
 *        case.
 */
#define MQTT_FIXED_HEADER_MAX_SIZE 5

/**@brief MQTT Control Packet Types. */
#define MQTT_PKT_TYPE_CONNECT     0x10
#define MQTT_PKT_TYPE_CONNACK     0x20
#define MQTT_PKT_TYPE_PUBLISH     0x30
#define MQTT_PKT_TYPE_PUBACK      0x40
#define MQTT_PKT_TYPE_PUBREC      0x50
#define MQTT_PKT_TYPE_PUBREL      0x60
#define MQTT_PKT_TYPE_PUBCOMP     0x70
#define MQTT_PKT_TYPE_SUBSCRIBE   0x80
#define MQTT_PKT_TYPE_SUBACK      0x90
#define MQTT_PKT_TYPE_UNSUBSCRIBE 0xA0
#define MQTT_PKT_TYPE_UNSUBACK    0xB0
#define MQTT_PKT_TYPE_PINGREQ     0xC0
#define MQTT_PKT_TYPE_PINGRSP     0xD0
#define MQTT_PKT_TYPE_DISCONNECT  0xE0
#define MQTT_PKT_TYPE_AUTH        0xF0

/**@brief Masks for MQTT header flags. */
#define MQTT_HEADER_DUP_MASK     0x08
#define MQTT_HEADER_QOS_MASK     0x06
#define MQTT_HEADER_RETAIN_MASK  0x01

/**@brief Masks for MQTT header flags. */
#define MQTT_CONNECT_FLAG_CLEAN_SESSION   0x02
#define MQTT_CONNECT_FLAG_WILL_TOPIC      0x04
#define MQTT_CONNECT_FLAG_WILL_RETAIN     0x20
#define MQTT_CONNECT_FLAG_PASSWORD        0x40
#define MQTT_CONNECT_FLAG_USERNAME        0x80

#define MQTT_CONNACK_FLAG_SESSION_PRESENT 0x01

/**@brief Maximum payload size of MQTT packet. */
#define MQTT_MAX_PAYLOAD_SIZE 0x0FFFFFFF

/**@brief Computes total size needed to pack a UTF8 string. */
#define GET_UT8STR_BUFFER_SIZE(STR) (sizeof(uint16_t) + (STR)->size)

/**@brief Computes total size needed to pack a binary stream. */
#define GET_BINSTR_BUFFER_SIZE(STR) (sizeof(uint16_t) + (STR)->len)

/**@brief Sets MQTT Client's state with one indicated in 'STATE'. */
#define MQTT_SET_STATE(CLIENT, STATE) ((CLIENT)->internal.state |= (STATE))

/**@brief Sets MQTT Client's state exclusive to 'STATE'. */
#define MQTT_SET_STATE_EXCLUSIVE(CLIENT, STATE) \
					((CLIENT)->internal.state = (STATE))

/**@brief Verifies if MQTT Client's state is set with one indicated in 'STATE'.
 */
#define MQTT_HAS_STATE(CLIENT, STATE) ((CLIENT)->internal.state & (STATE))

/**@brief Reset 'STATE' in MQTT Client's state. */
#define MQTT_RESET_STATE(CLIENT, STATE) ((CLIENT)->internal.state &= ~(STATE))

/**@brief Initialize MQTT Client's state. */
#define MQTT_STATE_INIT(CLIENT) ((CLIENT)->internal.state = MQTT_STATE_IDLE)

/**@brief Computes the first byte of MQTT message header based on message type,
 *        duplication flag, QoS and  the retain flag.
 */
#define MQTT_MESSAGES_OPTIONS(TYPE, DUP, QOS, RETAIN) \
	(((TYPE)      & 0xF0)  | \
	 (((DUP) << 3) & 0x08) | \
	 (((QOS) << 1) & 0x06) | \
	 ((RETAIN) & 0x01))

#define MQTT_MAX_LENGTH_BYTES 4
#define MQTT_LENGTH_VALUE_MASK 0x7F
#define MQTT_LENGTH_CONTINUATION_BIT 0x80
#define MQTT_LENGTH_SHIFT 7

/**@brief Check if the input pointer is NULL, if so it returns -EINVAL. */
#define NULL_PARAM_CHECK(param) \
	do { \
		if ((param) == NULL) { \
			return -EINVAL; \
		} \
	} while (false)

#define NULL_PARAM_CHECK_VOID(param) \
	do { \
		if ((param) == NULL) { \
			return; \
		} \
	} while (false)

/** Buffer context to iterate over buffer. */
struct buf_ctx {
	uint8_t *cur;
	uint8_t *end;
};

/**@brief MQTT States. */
enum mqtt_state {
	/** Idle state, implying the client entry in the table is unused/free.
	 */
	MQTT_STATE_IDLE                 = 0x00000000,

	/** TCP Connection has been requested, awaiting result of the request.
	 */
	MQTT_STATE_TCP_CONNECTING       = 0x00000001,

	/** TCP Connection successfully established. */
	MQTT_STATE_TCP_CONNECTED        = 0x00000002,

	/** MQTT Connection successful. */
	MQTT_STATE_CONNECTED            = 0x00000004,
};

static void event_notify(struct mqtt_client *client, const struct mqtt_evt *evt);
static int mqtt_handle_rx(struct mqtt_client *client);
static void mqtt_client_disconnect(struct mqtt_client *client, int result, bool notify);

/* mqtt_os.h =================================================================*/

static inline void mqtt_mutex_init(struct mqtt_client *client)
{
	k_mutex_init(&client->internal.mutex);
}

static inline void mqtt_mutex_lock(struct mqtt_client *client)
{
	(void)k_mutex_lock(&client->internal.mutex, K_FOREVER);
}

static inline void mqtt_mutex_unlock(struct mqtt_client *client)
{
	(void)k_mutex_unlock(&client->internal.mutex);
}

/**@brief Method to get the sys tick or a wall clock in millisecond resolution.
 *
 * @retval Current wall clock or sys tick value in milliseconds.
 */
static inline uint32_t mqtt_sys_tick_in_ms_get(void)
{
	return (uint32_t)k_uptime_get();
}

/**@brief Method to get elapsed time in milliseconds since the last activity.
 *
 * @param[in] last_activity The value since elapsed time is requested.
 *
 * @retval Time elapsed since last_activity time.
 */
static inline uint32_t mqtt_elapsed_time_in_ms_get(uint32_t last_activity)
{
	int32_t diff = (uint32_t)k_uptime_get() - last_activity;

	if (diff < 0) {
		return 0;
	}

	return diff;
}

/* mqtt_encoder.c ============================================================*/

static const struct mqtt_utf8 mqtt_3_1_0_proto_desc =
	MQTT_UTF8_LITERAL("MQIsdp");

static const struct mqtt_utf8 mqtt_proto_desc =
	MQTT_UTF8_LITERAL("MQTT");

/** Never changing ping request, needed for Keep Alive. */
static const uint8_t ping_packet[MQTT_FIXED_HEADER_MIN_SIZE] = {
	MQTT_PKT_TYPE_PINGREQ,
	0x00
};

/** Never changing disconnect request. */
static const uint8_t empty_disc_packet[MQTT_FIXED_HEADER_MIN_SIZE] = {
	MQTT_PKT_TYPE_DISCONNECT,
	0x00
};

/**
 * @brief Packs unsigned 8 bit value to the buffer at the offset requested.
 *
 * @retval 0 if procedure is successful.
 * @retval -ENOMEM if there is no place in the buffer to store the value.
 */
static int pack_uint8(uint8_t val, struct buf_ctx *buf)
{
	uint8_t *cur = buf->cur;
	uint8_t *end = buf->end;

	if ((end - cur) < sizeof(uint8_t)) {
		return -ENOMEM;
	}

	/* Pack value. */
	cur[0] = val;
	buf->cur = (cur + sizeof(uint8_t));

	return 0;
}

/**
 * @brief Packs unsigned 16 bit value to the buffer at the offset requested.
 *
 * @retval 0 if the procedure is successful.
 * @retval -ENOMEM if there is no place in the buffer to store the value.
 */
static int pack_uint16(uint16_t val, struct buf_ctx *buf)
{
	uint8_t *cur = buf->cur;
	uint8_t *end = buf->end;

	if ((end - cur) < sizeof(uint16_t)) {
		return -ENOMEM;
	}

	/* Pack value. */
	sys_put_be16(val, cur);
	buf->cur = (cur + sizeof(uint16_t));

	return 0;
}

/**
 * @brief Packs utf8 string to the buffer at the offset requested.
 *
 * @retval 0 if the procedure is successful.
 * @retval -ENOMEM if there is no place in the buffer to store the string.
 */
static int pack_utf8_str(const struct mqtt_utf8 *str, struct buf_ctx *buf)
{
	if ((buf->end - buf->cur) < GET_UT8STR_BUFFER_SIZE(str)) {
		return -ENOMEM;
	}

	/* Pack length followed by string. */
	(void)pack_uint16(str->size, buf);

	memcpy(buf->cur, str->utf8, str->size);
	buf->cur += str->size;

	return 0;
}

/**
 * @brief Computes and encodes variable length integer.
 *
 * @param[in] value Integer value to encode.
 * @param[inout] buf A pointer to the buf_ctx structure containing current
 *                   buffer position. May be NULL (in this case function will
 *                   only calculate number of bytes needed).
 *
 * @return Number of bytes needed to encode integer or a negative error code.
 */
static int pack_variable_int(uint32_t value, struct buf_ctx *buf)
{
	int encoded_bytes = 0U;

	do {
		encoded_bytes++;

		if (buf != NULL) {
			if (buf->cur >= buf->end) {
				return -ENOMEM;
			}

			*(buf->cur) = value & MQTT_LENGTH_VALUE_MASK;
		}

		value >>= MQTT_LENGTH_SHIFT;

		if (buf != NULL) {
			if (value > 0) {
				*(buf->cur) |= MQTT_LENGTH_CONTINUATION_BIT;
			}
			buf->cur++;
		}
	} while (value > 0);

	return encoded_bytes;
}

/**
 * @brief Encodes fixed header for the MQTT message and provides pointer to
 *        start of the header.
 *
 * The 5 bytes before the start of the message are assumed by the routine to
 * be available to pack the fixed header.  As output, the pointers will point
 * to beginning and the end of the frame.
 *
 * @retval 0 if the procedure is successful.
 * @retval -EMSGSIZE if the message is too big for MQTT.
 */
static uint32_t mqtt_encode_fixed_header(uint8_t message_type, uint8_t *start,
				      struct buf_ctx *buf)
{
	uint32_t length = buf->cur - start;
	uint8_t fixed_header_length;

	if (length > MQTT_MAX_PAYLOAD_SIZE) {
		return -EMSGSIZE;
	}

	fixed_header_length = pack_variable_int(length, NULL);
	fixed_header_length += sizeof(uint8_t);

	/* Set the pointer at the start of the frame before encoding. */
	buf->cur = start - fixed_header_length;

	(void)pack_uint8(message_type, buf);
	(void)pack_variable_int(length, buf);

	/* Set the cur pointer back at the start of the frame,
	 * and end pointer to the end of the frame.
	 */
	buf->cur = buf->cur - fixed_header_length;
	buf->end = buf->cur + length + fixed_header_length;

	return 0;
}

/**
 * @brief Encodes a string of a zero length.
 *
 * @retval 0 if the procedure is successful.
 * @retval -ENOMEM if there is no place in the buffer to store the binary
 *                 string.
 */
static int zero_len_str_encode(struct buf_ctx *buf)
{
	return pack_uint16(0x0000, buf);
}

static int connect_request_encode(const struct mqtt_client *client,
			   struct buf_ctx *buf)
{
	uint8_t connect_flags = client->clean_session << 1;
	const uint8_t message_type =
			MQTT_MESSAGES_OPTIONS(MQTT_PKT_TYPE_CONNECT, 0, 0, 0);
	const struct mqtt_utf8 *proto_desc;
	uint8_t *connect_flags_pos;
	int err_code;
	uint8_t *start;

	if (client->protocol_version == MQTT_VERSION_3_1_0) {
		proto_desc = &mqtt_3_1_0_proto_desc;
	} else {
		/* MQTT 3.1.1 and newer share the same protocol prefix. */
		proto_desc = &mqtt_proto_desc;
	}

	/* Reserve space for fixed header. */
	buf->cur += MQTT_FIXED_HEADER_MAX_SIZE;
	start = buf->cur;

	err_code = pack_utf8_str(proto_desc, buf);
	if (err_code != 0) {
		return err_code;
	}

	err_code = pack_uint8(client->protocol_version, buf);
	if (err_code != 0) {
		return err_code;
	}

	/* Remember position of connect flag and leave one byte for it to
	 * be packed once we determine its value.
	 */
	connect_flags_pos = buf->cur;

	err_code = pack_uint8(0, buf);
	if (err_code != 0) {
		return err_code;
	}

	err_code = pack_uint16(client->keepalive, buf);
	if (err_code != 0) {
		return err_code;
	}

	err_code = pack_utf8_str(&client->client_id, buf);
	if (err_code != 0) {
		return err_code;
	}

	/* Pack will topic and QoS */
	if (client->will_topic != NULL) {
		connect_flags |= MQTT_CONNECT_FLAG_WILL_TOPIC;
		/* QoS is always 1 as of now. */
		connect_flags |= ((client->will_topic->qos & 0x03) << 3);
		connect_flags |= client->will_retain << 5;

		err_code = pack_utf8_str(&client->will_topic->topic, buf);
		if (err_code != 0) {
			return err_code;
		}

		if (client->will_message != NULL) {
			err_code = pack_utf8_str(client->will_message, buf);
			if (err_code != 0) {
				return err_code;
			}
		} else {
			err_code = zero_len_str_encode(buf);
			if (err_code != 0) {
				return err_code;
			}
		}
	}

	/* Pack Username if any. */
	if (client->user_name != NULL) {
		connect_flags |= MQTT_CONNECT_FLAG_USERNAME;

		err_code = pack_utf8_str(client->user_name, buf);
		if (err_code != 0) {
			return err_code;
		}
	}

	/* Pack Password if any. */
	if (client->password != NULL) {
		connect_flags |= MQTT_CONNECT_FLAG_PASSWORD;

		err_code = pack_utf8_str(client->password, buf);
		if (err_code != 0) {
			return err_code;
		}
	}

	/* Write the flags the connect flags. */
	*connect_flags_pos = connect_flags;

	return mqtt_encode_fixed_header(message_type, start, buf);
}

static int publish_encode(const struct mqtt_client *client,
		   const struct mqtt_publish_param *param,
		   struct buf_ctx *buf)
{
	const uint8_t message_type = MQTT_MESSAGES_OPTIONS(
			MQTT_PKT_TYPE_PUBLISH, param->dup_flag,
			param->message.topic.qos, param->retain_flag);
	int err_code;
	uint8_t *start;

	ARG_UNUSED(client);

	/* Message id zero is not permitted by spec. */
	if ((param->message.topic.qos) && (param->message_id == 0U)) {
		return -EINVAL;
	}

	/* Reserve space for fixed header. */
	buf->cur += MQTT_FIXED_HEADER_MAX_SIZE;
	start = buf->cur;

	err_code = pack_utf8_str(&param->message.topic.topic, buf);
	if (err_code != 0) {
		return err_code;
	}

	if (param->message.topic.qos) {
		err_code = pack_uint16(param->message_id, buf);
		if (err_code != 0) {
			return err_code;
		}
	}

	/* Do not copy payload. We move the buffer pointer to ensure that
	 * message length in fixed header is encoded correctly.
	 */
	buf->cur += param->message.payload.len;

	err_code = mqtt_encode_fixed_header(message_type, start, buf);
	if (err_code != 0) {
		return err_code;
	}

	buf->end -= param->message.payload.len;

	return 0;
}

static int empty_disconnect_encode(struct buf_ctx *buf)
{
	uint8_t *cur = buf->cur;
	uint8_t *end = buf->end;

	if ((end - cur) < sizeof(empty_disc_packet)) {
		return -ENOMEM;
	}

	memcpy(cur, empty_disc_packet, sizeof(empty_disc_packet));
	buf->end = (cur + sizeof(empty_disc_packet));

	return 0;
}

static int disconnect_encode(const struct mqtt_client *client,
		      const struct mqtt_disconnect_param *param,
		      struct buf_ctx *buf)
{
	ARG_UNUSED(client);
	ARG_UNUSED(param);

	return empty_disconnect_encode(buf);
}

static int ping_request_encode(struct buf_ctx *buf)
{
	uint8_t *cur = buf->cur;
	uint8_t *end = buf->end;

	if ((end - cur) < sizeof(ping_packet)) {
		return -ENOMEM;
	}

	memcpy(cur, ping_packet, sizeof(ping_packet));
	buf->end = (cur + sizeof(ping_packet));

	return 0;
}

/* mqtt_decoder.c ============================================================*/

/**
 * @brief Unpacks unsigned 8 bit value from the buffer from the offset
 *        requested.
 *
 * @retval 0 if the procedure is successful.
 * @retval -EINVAL if the buffer would be exceeded during the read
 */
static int unpack_uint8(struct buf_ctx *buf, uint8_t *val)
{
	uint8_t *cur = buf->cur;
	uint8_t *end = buf->end;

	if ((end - cur) < sizeof(uint8_t)) {
		return -EINVAL;
	}

	*val = cur[0];
	buf->cur = (cur + sizeof(uint8_t));

	return 0;
}

/**
 * @brief Unpacks unsigned 16 bit value from the buffer from the offset
 *        requested.
 *
 * @retval 0 if the procedure is successful.
 * @retval -EINVAL if the buffer would be exceeded during the read
 */
static int unpack_uint16(struct buf_ctx *buf, uint16_t *val)
{
	uint8_t *cur = buf->cur;
	uint8_t *end = buf->end;

	if ((end - cur) < sizeof(uint16_t)) {
		return -EINVAL;
	}

	*val = sys_get_be16(cur);
	buf->cur = (cur + sizeof(uint16_t));

	return 0;
}

/**
 * @brief Unpacks utf8 string from the buffer from the offset requested.
 *
 * @retval 0 if the procedure is successful.
 * @retval -EINVAL if the buffer would be exceeded during the read
 */
static int unpack_utf8_str(struct buf_ctx *buf, struct mqtt_utf8 *str)
{
	uint16_t utf8_strlen;
	int err_code;

	err_code = unpack_uint16(buf, &utf8_strlen);
	if (err_code != 0) {
		return err_code;
	}

	if ((buf->end - buf->cur) < utf8_strlen) {
		return -EINVAL;
	}

	str->size = utf8_strlen;
	/* Zero length UTF8 strings permitted. */
	if (utf8_strlen) {
		/* Point to right location in buffer. */
		str->utf8 = buf->cur;
		buf->cur += utf8_strlen;
	} else {
		str->utf8 = NULL;
	}

	return 0;
}

/**
 * @brief Unpacks binary string from the buffer from the offset requested with
 *        the length provided.
 *
 * @retval 0 if the procedure is successful.
 * @retval -EINVAL if the buffer would be exceeded during the read
 */
static int unpack_raw_data(uint32_t length, struct buf_ctx *buf,
			   struct mqtt_binstr *str)
{
	if ((buf->end - buf->cur) < length) {
		return -EINVAL;
	}

	str->len = length;

	/* Zero length binary strings are permitted. */
	if (length > 0) {
		str->data = buf->cur;
		buf->cur += length;
	} else {
		str->data = NULL;
	}

	return 0;
}

/**
 * @brief Unpacks variable length integer from the buffer from the offset
 *        requested.
 *
 * @retval Number of bytes parsed if the procedure is successful.
 * @retval -EINVAL if the length decoding would use more that 4 bytes.
 * @retval -EAGAIN if the buffer would be exceeded during the read.
 */
static int unpack_variable_int(struct buf_ctx *buf, uint32_t *val)
{
	uint8_t shift = 0U;
	int bytes = 0;

	*val = 0U;
	do {
		if (bytes >= MQTT_MAX_LENGTH_BYTES) {
			return -EINVAL;
		}

		if (buf->cur >= buf->end) {
			return -EAGAIN;
		}

		*val += ((uint32_t)*(buf->cur) & MQTT_LENGTH_VALUE_MASK)
								<< shift;
		shift += MQTT_LENGTH_SHIFT;
		bytes++;
	} while ((*(buf->cur++) & MQTT_LENGTH_CONTINUATION_BIT) != 0U);

	if (*val > MQTT_MAX_PAYLOAD_SIZE) {
		return -EINVAL;
	}

	return bytes;
}

static int fixed_header_decode(struct buf_ctx *buf, uint8_t *type_and_flags,
			uint32_t *length)
{
	int err_code;

	err_code = unpack_uint8(buf, type_and_flags);
	if (err_code != 0) {
		return err_code;
	}

	err_code = unpack_variable_int(buf, length);
	if (err_code < 0) {
		return err_code;
	}

	return 0;
}

static int connect_ack_decode(const struct mqtt_client *client, struct buf_ctx *buf,
		       struct mqtt_connack_param *param)
{
	int err_code;
	uint8_t flags, ret_code;

	err_code = unpack_uint8(buf, &flags);
	if (err_code != 0) {
		return err_code;
	}

	err_code = unpack_uint8(buf, &ret_code);
	if (err_code != 0) {
		return err_code;
	}

	param->return_code = ret_code;

	if (client->protocol_version == MQTT_VERSION_3_1_0) {
		goto out;
	}

	param->session_present_flag = flags & MQTT_CONNACK_FLAG_SESSION_PRESENT;

out:
	return 0;
}

static int publish_decode(struct mqtt_client *client, uint8_t flags,
		   uint32_t var_length, struct buf_ctx *buf,
		   struct mqtt_publish_param *param)
{
	int err_code;
	uint32_t var_header_length;

	ARG_UNUSED(client);

	param->dup_flag = flags & MQTT_HEADER_DUP_MASK;
	param->retain_flag = flags & MQTT_HEADER_RETAIN_MASK;
	param->message.topic.qos = ((flags & MQTT_HEADER_QOS_MASK) >> 1);

	err_code = unpack_utf8_str(buf, &param->message.topic.topic);
	if (err_code != 0) {
		return err_code;
	}

	var_header_length = param->message.topic.topic.size + sizeof(uint16_t);

	if (param->message.topic.qos > MQTT_QOS_0_AT_MOST_ONCE) {
		err_code = unpack_uint16(buf, &param->message_id);
		if (err_code != 0) {
			return err_code;
		}

		var_header_length += sizeof(uint16_t);
	}

	if (var_length < var_header_length) {
		return -EINVAL;
	}

	param->message.payload.data = NULL;
	param->message.payload.len = var_length - var_header_length;

	return 0;
}

static int common_pub_ack_decode(struct buf_ctx *buf, uint16_t *message_id)
{
	int err;

	err = unpack_uint16(buf, message_id);
	if (err < 0) {
		return err;
	}

	return 0;
}

static int publish_ack_decode(const struct mqtt_client *client, struct buf_ctx *buf,
		       struct mqtt_puback_param *param)
{
	ARG_UNUSED(client);

	return common_pub_ack_decode(buf, &param->message_id);
}

static int publish_receive_decode(const struct mqtt_client *client, struct buf_ctx *buf,
			   struct mqtt_pubrec_param *param)
{
	ARG_UNUSED(client);

	return common_pub_ack_decode(buf, &param->message_id);
}

static int publish_release_decode(const struct mqtt_client *client, struct buf_ctx *buf,
			   struct mqtt_pubrel_param *param)
{
	ARG_UNUSED(client);

	return common_pub_ack_decode(buf, &param->message_id);
}

static int publish_complete_decode(const struct mqtt_client *client, struct buf_ctx *buf,
			    struct mqtt_pubcomp_param *param)
{
	ARG_UNUSED(client);

	return common_pub_ack_decode(buf, &param->message_id);
}

static int subscribe_ack_decode(const struct mqtt_client *client, struct buf_ctx *buf,
			 struct mqtt_suback_param *param)
{
	int err_code;

	ARG_UNUSED(client);

	err_code = unpack_uint16(buf, &param->message_id);
	if (err_code != 0) {
		return err_code;
	}

	return unpack_raw_data(buf->end - buf->cur, buf, &param->return_codes);
}

static int unsubscribe_ack_decode(const struct mqtt_client *client, struct buf_ctx *buf,
			   struct mqtt_unsuback_param *param)
{
	int err;

	ARG_UNUSED(client);

	err = unpack_uint16(buf, &param->message_id);
	if (err < 0) {
		return 0;
	}

	return 0;
}

/* mqtt_transport_socket_tcp.c ===============================================*/

static int mqtt_client_tcp_connect(struct mqtt_client *client)
{
	const struct sockaddr *broker = client->broker;
	int ret;

	/* Binding to an interface (SO_BINDTODEVICE) is not supported. */
	if (client->transport.if_name != NULL) {
		return -ENOTSUP;
	}

	client->transport.tcp.sock = mqtt_min_sock_socket(broker->sa_family, SOCK_STREAM,
						  IPPROTO_TCP);
	if (client->transport.tcp.sock < 0) {
		return -errno;
	}

	size_t peer_addr_size = sizeof(struct sockaddr_in6);

	if (broker->sa_family == AF_INET) {
		peer_addr_size = sizeof(struct sockaddr_in);
	}

	ret = mqtt_min_sock_connect(client->transport.tcp.sock, client->broker,
			    peer_addr_size);
	if (ret < 0) {
		goto error;
	}

	return 0;

error:
	(void)mqtt_min_sock_close(client->transport.tcp.sock);
	return -errno;
}

static int mqtt_client_tcp_write(struct mqtt_client *client, const uint8_t *data,
			  uint32_t datalen)
{
	uint32_t offset = 0U;
	int ret;

	while (offset < datalen) {
		ret = mqtt_min_sock_send(client->transport.tcp.sock, data + offset,
				 datalen - offset, 0);
		if (ret < 0) {
			return -errno;
		}

		offset += ret;
	}

	return 0;
}

static int mqtt_client_tcp_write_msg(struct mqtt_client *client,
			      const struct msghdr *message)

{
	int ret, i;
	size_t offset = 0;
	size_t total_len = 0;

	for (i = 0; i < message->msg_iovlen; i++) {
		total_len += message->msg_iov[i].iov_len;
	}

	while (offset < total_len) {
		ret = mqtt_min_sock_sendmsg(client->transport.tcp.sock, message, 0);
		if (ret < 0) {
			return -errno;
		}

		offset += ret;
		if (offset >= total_len) {
			break;
		}

		/* Update msghdr for the next iteration. */
		for (i = 0; i < message->msg_iovlen; i++) {
			if (ret < message->msg_iov[i].iov_len) {
				message->msg_iov[i].iov_len -= ret;
				message->msg_iov[i].iov_base =
					(uint8_t *)message->msg_iov[i].iov_base + ret;
				break;
			}

			ret -= message->msg_iov[i].iov_len;
			message->msg_iov[i].iov_len = 0;
		}
	}

	return 0;
}

static int mqtt_client_tcp_read(struct mqtt_client *client, uint8_t *data, uint32_t buflen,
			 bool shall_block)
{
	int flags = 0;
	int ret;

	if (!shall_block) {
		flags |= MSG_DONTWAIT;
	}

	ret = mqtt_min_sock_recv(client->transport.tcp.sock, data, buflen, flags);
	if (ret < 0) {
		return -errno;
	}

	return ret;
}

static int mqtt_client_tcp_disconnect(struct mqtt_client *client)
{
	int ret;

	ret = mqtt_min_sock_close(client->transport.tcp.sock);
	if (ret < 0) {
		return -errno;
	}

	return 0;
}

/* mqtt_transport.c (the TCP transport only) =================================*/

static int mqtt_transport_connect(struct mqtt_client *client)
{
	return mqtt_client_tcp_connect(client);
}

static int mqtt_transport_write(struct mqtt_client *client, const uint8_t *data,
			 uint32_t datalen)
{
	return mqtt_client_tcp_write(client, data, datalen);
}

static int mqtt_transport_write_msg(struct mqtt_client *client,
			     const struct msghdr *message)
{
	return mqtt_client_tcp_write_msg(client, message);
}

static int mqtt_transport_read(struct mqtt_client *client, uint8_t *data, uint32_t buflen,
			bool shall_block)
{
	return mqtt_client_tcp_read(client, data, buflen, shall_block);
}

static int mqtt_transport_disconnect(struct mqtt_client *client)
{
	return mqtt_client_tcp_disconnect(client);
}

/* mqtt_rx.c =================================================================*/

static int mqtt_handle_packet(struct mqtt_client *client,
			      uint8_t type_and_flags,
			      uint32_t var_length,
			      struct buf_ctx *buf)
{
	int err_code = 0;
	bool notify_event = true;
	struct mqtt_evt evt = { 0 };

	/* Success by default, overwritten in special cases. */
	evt.result = 0;

	switch (type_and_flags & 0xF0) {
	case MQTT_PKT_TYPE_CONNACK:
		evt.type = MQTT_EVT_CONNACK;
		err_code = connect_ack_decode(client, buf, &evt.param.connack);
		if (err_code == 0) {
			if (evt.param.connack.return_code ==
						MQTT_CONNECTION_ACCEPTED) {
				/* Set state. */
				MQTT_SET_STATE(client, MQTT_STATE_CONNECTED);
			} else {
				err_code = -ECONNREFUSED;
			}

			evt.result = evt.param.connack.return_code;
		} else {
			evt.result = err_code;
		}

		break;

	case MQTT_PKT_TYPE_PUBLISH:
		evt.type = MQTT_EVT_PUBLISH;
		err_code = publish_decode(client, type_and_flags, var_length,
					  buf, &evt.param.publish);
		evt.result = err_code;

		client->internal.remaining_payload =
					evt.param.publish.message.payload.len;

		break;

	case MQTT_PKT_TYPE_PUBACK:
		evt.type = MQTT_EVT_PUBACK;
		err_code = publish_ack_decode(client, buf, &evt.param.puback);
		evt.result = err_code;
		break;

	case MQTT_PKT_TYPE_PUBREC:
		evt.type = MQTT_EVT_PUBREC;
		err_code = publish_receive_decode(client, buf,
						  &evt.param.pubrec);
		evt.result = err_code;
		break;

	case MQTT_PKT_TYPE_PUBREL:
		evt.type = MQTT_EVT_PUBREL;
		err_code = publish_release_decode(client, buf,
						  &evt.param.pubrel);
		evt.result = err_code;
		break;

	case MQTT_PKT_TYPE_PUBCOMP:
		evt.type = MQTT_EVT_PUBCOMP;
		err_code = publish_complete_decode(client, buf,
						   &evt.param.pubcomp);
		evt.result = err_code;
		break;

	case MQTT_PKT_TYPE_SUBACK:
		evt.type = MQTT_EVT_SUBACK;
		err_code = subscribe_ack_decode(client, buf, &evt.param.suback);
		evt.result = err_code;
		break;

	case MQTT_PKT_TYPE_UNSUBACK:
		evt.type = MQTT_EVT_UNSUBACK;
		err_code = unsubscribe_ack_decode(client, buf,
						  &evt.param.unsuback);
		evt.result = err_code;
		break;

	case MQTT_PKT_TYPE_PINGRSP:
		if (client->unacked_ping <= 0) {
			client->unacked_ping = 0;
		} else {
			client->unacked_ping--;
		}

		evt.type = MQTT_EVT_PINGRESP;
		break;

	default:
		/* Nothing to notify. */
		notify_event = false;
		break;
	}

	if (notify_event == true) {
		event_notify(client, &evt);
	}

	return err_code;
}

static int mqtt_read_message_chunk(struct mqtt_client *client,
				   struct buf_ctx *buf, uint32_t length)
{
	uint32_t remaining;
	int len;

	/* In case all data requested has already been buffered, return. */
	if (length <= (buf->end - buf->cur)) {
		return 0;
	}

	/* Calculate how much data we need to read from the transport,
	 * given the already buffered data.
	 */
	remaining = length - (buf->end - buf->cur);

	/* Check if read does not exceed the buffer. */
	if ((buf->end + remaining > client->rx_buf + client->rx_buf_size) ||
	    (buf->end + remaining < client->rx_buf)) {
		return -ENOMEM;
	}

	len = mqtt_transport_read(client, buf->end, remaining, false);
	if (len < 0) {
		return len;
	}

	if (len == 0) {
		return -ENOTCONN;
	}

	client->internal.rx_buf_datalen += len;
	buf->end += len;

	if (len < remaining) {
		return -EAGAIN;
	}

	return 0;
}

static int mqtt_read_publish_var_header(struct mqtt_client *client,
					uint8_t type_and_flags,
					struct buf_ctx *buf)
{
	uint8_t qos = (type_and_flags & MQTT_HEADER_QOS_MASK) >> 1;
	int err_code;
	uint32_t variable_header_length;

	/* Read topic length field. */
	err_code = mqtt_read_message_chunk(client, buf, sizeof(uint16_t));
	if (err_code < 0) {
		return err_code;
	}

	variable_header_length = *buf->cur << 8; /* MSB */
	variable_header_length |= *(buf->cur + 1); /* LSB */

	/* Add two bytes for topic length field. */
	variable_header_length += sizeof(uint16_t);

	/* Add two bytes for message_id, if needed. */
	if (qos > MQTT_QOS_0_AT_MOST_ONCE) {
		variable_header_length += sizeof(uint16_t);
	}

	err_code = mqtt_read_message_chunk(client, buf,
					   variable_header_length);
	if (err_code < 0) {
		return err_code;
	}

	return 0;
}

static int mqtt_read_and_parse_fixed_header(struct mqtt_client *client,
					    uint8_t *type_and_flags,
					    uint32_t *var_length,
					    struct buf_ctx *buf)
{
	/* Read the mandatory part of the fixed header in first iteration. */
	uint8_t chunk_size = MQTT_FIXED_HEADER_MIN_SIZE;
	int err_code;

	do {
		err_code = mqtt_read_message_chunk(client, buf, chunk_size);
		if (err_code < 0) {
			return err_code;
		}

		/* Reset to pointer to the beginning of the frame. */
		buf->cur = client->rx_buf;
		chunk_size = 1U;

		err_code = fixed_header_decode(buf, type_and_flags, var_length);
	} while (err_code == -EAGAIN);

	return err_code;
}

static int mqtt_handle_rx(struct mqtt_client *client)
{
	int err_code;
	uint8_t type_and_flags;
	uint32_t var_length;
	struct buf_ctx buf;

	buf.cur = client->rx_buf;
	buf.end = client->rx_buf + client->internal.rx_buf_datalen;

	err_code = mqtt_read_and_parse_fixed_header(client, &type_and_flags,
						    &var_length, &buf);
	if (err_code < 0) {
		return (err_code == -EAGAIN) ? 0 : err_code;
	}

	if ((type_and_flags & 0xF0) == MQTT_PKT_TYPE_PUBLISH) {
		err_code = mqtt_read_publish_var_header(client, type_and_flags,
							&buf);
	} else {
		err_code = mqtt_read_message_chunk(client, &buf, var_length);
	}

	if (err_code < 0) {
		return (err_code == -EAGAIN) ? 0 : err_code;
	}

	/* At this point, packet is ready to be passed to the application. */
	err_code = mqtt_handle_packet(client, type_and_flags, var_length, &buf);
	if (err_code < 0) {
		return err_code;
	}

	client->internal.rx_buf_datalen = 0U;

	return 0;
}

/* mqtt.c ====================================================================*/

static void client_reset(struct mqtt_client *client)
{
	MQTT_STATE_INIT(client);

	client->internal.last_activity = 0U;
	client->internal.rx_buf_datalen = 0U;
	client->internal.remaining_payload = 0U;
}

/** @brief Initialize tx buffer. */
static void tx_buf_init(struct mqtt_client *client, struct buf_ctx *buf)
{
	memset(client->tx_buf, 0, client->tx_buf_size);
	buf->cur = client->tx_buf;
	buf->end = client->tx_buf + client->tx_buf_size;
}

static void event_notify(struct mqtt_client *client, const struct mqtt_evt *evt)
{
	if (client->evt_cb != NULL) {
		mqtt_mutex_unlock(client);

		client->evt_cb(client, evt);

		mqtt_mutex_lock(client);
	}
}

static void mqtt_client_disconnect(struct mqtt_client *client, int result, bool notify)
{
	int err_code;

	err_code = mqtt_transport_disconnect(client);
	(void)err_code; /* Zephyr logs "Failed to disconnect transport!" */

	/* Reset internal state. */
	client_reset(client);

	if (notify) {
		struct mqtt_evt evt = {
			.type = MQTT_EVT_DISCONNECT,
			.result = result,
		};

		/* Notify application. */
		event_notify(client, &evt);
	}
}

static int client_connect(struct mqtt_client *client)
{
	int err_code;
	struct buf_ctx packet;

	err_code = mqtt_transport_connect(client);
	if (err_code < 0) {
		return err_code;
	}

	tx_buf_init(client, &packet);
	MQTT_SET_STATE(client, MQTT_STATE_TCP_CONNECTED);

	err_code = connect_request_encode(client, &packet);
	if (err_code < 0) {
		goto error;
	}

	/* Send MQTT identification message to broker. */
	err_code = mqtt_transport_write(client, packet.cur,
					packet.end - packet.cur);
	if (err_code < 0) {
		goto error;
	}

	client->internal.last_activity = mqtt_sys_tick_in_ms_get();

	/* Reset the unanswered ping count for a new connection */
	client->unacked_ping = 0;

	return 0;

error:
	mqtt_client_disconnect(client, err_code, false);
	return err_code;
}

static int client_read(struct mqtt_client *client)
{
	int err_code;

	if (client->internal.remaining_payload > 0) {
		return -EBUSY;
	}

	err_code = mqtt_handle_rx(client);
	if (err_code < 0) {
		mqtt_client_disconnect(client, err_code, true);
	}

	return err_code;
}

static int client_write(struct mqtt_client *client, const uint8_t *data,
			uint32_t datalen)
{
	int err_code;

	err_code = mqtt_transport_write(client, data, datalen);
	if (err_code < 0) {
		mqtt_client_disconnect(client, err_code, true);
		return err_code;
	}

	client->internal.last_activity = mqtt_sys_tick_in_ms_get();

	return 0;
}

static int client_write_msg(struct mqtt_client *client,
			    const struct msghdr *message)
{
	int err_code;

	err_code = mqtt_transport_write_msg(client, message);
	if (err_code < 0) {
		mqtt_client_disconnect(client, err_code, true);
		return err_code;
	}

	client->internal.last_activity = mqtt_sys_tick_in_ms_get();

	return 0;
}

void mqtt_client_init(struct mqtt_client *client)
{
	NULL_PARAM_CHECK_VOID(client);

	memset(client, 0, sizeof(*client));

	MQTT_STATE_INIT(client);
	mqtt_mutex_init(client);

	client->protocol_version = MQTT_VERSION_3_1_1;
	client->clean_session = MQTT_CLEAN_SESSION;
	client->keepalive = MQTT_KEEPALIVE;
}

int mqtt_connect(struct mqtt_client *client)
{
	int err_code;

	NULL_PARAM_CHECK(client);
	NULL_PARAM_CHECK(client->client_id.utf8);

	mqtt_mutex_lock(client);

	if ((client->tx_buf == NULL) || (client->rx_buf == NULL)) {
		err_code = -ENOMEM;
		goto error;
	}

	err_code = client_connect(client);

error:
	if (err_code < 0) {
		client_reset(client);
	}

	mqtt_mutex_unlock(client);

	return err_code;
}

static int verify_tx_state(const struct mqtt_client *client)
{
	if (!MQTT_HAS_STATE(client, MQTT_STATE_CONNECTED)) {
		return -ENOTCONN;
	}

	return 0;
}

int mqtt_publish(struct mqtt_client *client,
		 const struct mqtt_publish_param *param)
{
	int err_code;
	struct buf_ctx packet;
	struct iovec io_vector[2];
	struct msghdr msg;

	NULL_PARAM_CHECK(client);
	NULL_PARAM_CHECK(param);

	mqtt_mutex_lock(client);

	tx_buf_init(client, &packet);

	err_code = verify_tx_state(client);
	if (err_code < 0) {
		goto error;
	}

	err_code = publish_encode(client, param, &packet);
	if (err_code < 0) {
		goto error;
	}

	io_vector[0].iov_base = packet.cur;
	io_vector[0].iov_len = packet.end - packet.cur;
	io_vector[1].iov_base = param->message.payload.data;
	io_vector[1].iov_len = param->message.payload.len;

	memset(&msg, 0, sizeof(msg));

	msg.msg_iov = io_vector;
	msg.msg_iovlen = ARRAY_SIZE(io_vector);

	err_code = client_write_msg(client, &msg);

error:
	mqtt_mutex_unlock(client);

	return err_code;
}

int mqtt_disconnect(struct mqtt_client *client,
		    const struct mqtt_disconnect_param *param)
{
	int err_code;
	struct buf_ctx packet;

	NULL_PARAM_CHECK(client);

	mqtt_mutex_lock(client);

	tx_buf_init(client, &packet);

	err_code = verify_tx_state(client);
	if (err_code < 0) {
		goto error;
	}

	err_code = disconnect_encode(client, param, &packet);
	if (err_code < 0) {
		goto error;
	}

	err_code = client_write(client, packet.cur, packet.end - packet.cur);
	if (err_code < 0) {
		goto error;
	}

	mqtt_client_disconnect(client, 0, true);

error:
	mqtt_mutex_unlock(client);

	return err_code;
}

int mqtt_ping(struct mqtt_client *client)
{
	int err_code;
	struct buf_ctx packet;

	NULL_PARAM_CHECK(client);

	mqtt_mutex_lock(client);

	tx_buf_init(client, &packet);

	err_code = verify_tx_state(client);
	if (err_code < 0) {
		goto error;
	}

	err_code = ping_request_encode(&packet);
	if (err_code < 0) {
		goto error;
	}

	err_code = client_write(client, packet.cur, packet.end - packet.cur);

	if (client->unacked_ping >= INT8_MAX) {
		/* Zephyr logs "PING count overflow!" */
	} else {
		client->unacked_ping++;
	}

error:
	mqtt_mutex_unlock(client);

	return err_code;
}

int mqtt_abort(struct mqtt_client *client)
{
	NULL_PARAM_CHECK(client);

	mqtt_mutex_lock(client);

	if (client->internal.state != MQTT_STATE_IDLE) {
		mqtt_client_disconnect(client, -ECONNABORTED, true);
	}

	mqtt_mutex_unlock(client);

	return 0;
}

int mqtt_live(struct mqtt_client *client)
{
	int err_code = 0;
	uint32_t elapsed_time;
	bool ping_sent = false;

	NULL_PARAM_CHECK(client);

	mqtt_mutex_lock(client);

	elapsed_time = mqtt_elapsed_time_in_ms_get(
				client->internal.last_activity);
	if ((client->keepalive > 0) &&
	    (elapsed_time >= (client->keepalive * 1000))) {
		err_code = mqtt_ping(client);
		ping_sent = true;
	}

	mqtt_mutex_unlock(client);

	if (ping_sent) {
		return err_code;
	} else {
		return -EAGAIN;
	}
}

int mqtt_keepalive_time_left(const struct mqtt_client *client)
{
	uint32_t elapsed_time = mqtt_elapsed_time_in_ms_get(
					client->internal.last_activity);
	uint32_t keepalive_ms = 1000U * client->keepalive;

	if (client->keepalive == 0) {
		/* Keep alive not enabled. */
		return -1;
	}

	if (keepalive_ms <= elapsed_time) {
		return 0;
	}

	return keepalive_ms - elapsed_time;
}

int mqtt_input(struct mqtt_client *client)
{
	int err_code = 0;

	NULL_PARAM_CHECK(client);

	mqtt_mutex_lock(client);

	if (MQTT_HAS_STATE(client, MQTT_STATE_TCP_CONNECTED)) {
		err_code = client_read(client);
	} else {
		err_code = -ENOTCONN;
	}

	mqtt_mutex_unlock(client);

	return err_code;
}

static int read_publish_payload(struct mqtt_client *client, void *buffer,
				size_t length, bool shall_block)
{
	int ret;

	NULL_PARAM_CHECK(client);

	mqtt_mutex_lock(client);

	if (client->internal.remaining_payload == 0U) {
		ret = 0;
		goto exit;
	}

	if (client->internal.remaining_payload < length) {
		length = client->internal.remaining_payload;
	}

	ret = mqtt_transport_read(client, buffer, length, shall_block);
	if (!shall_block && ret == -EAGAIN) {
		goto exit;
	}

	if (ret <= 0) {
		if (ret == 0) {
			ret = -ENOTCONN;
		}

		mqtt_client_disconnect(client, ret, true);
		goto exit;
	}

	client->internal.remaining_payload -= ret;

exit:
	mqtt_mutex_unlock(client);

	return ret;
}

int mqtt_read_publish_payload(struct mqtt_client *client, void *buffer,
			      size_t length)
{
	return read_publish_payload(client, buffer, length, false);
}

int mqtt_read_publish_payload_blocking(struct mqtt_client *client, void *buffer,
				       size_t length)
{
	return read_publish_payload(client, buffer, length, true);
}

int mqtt_readall_publish_payload(struct mqtt_client *client, uint8_t *buffer,
				 size_t length)
{
	uint8_t *end = buffer + length;

	while (buffer < end) {
		int ret = mqtt_read_publish_payload_blocking(client, buffer,
							     end - buffer);

		if (ret < 0) {
			return ret;
		} else if (ret == 0) {
			return -EIO;
		}

		buffer += ret;
	}

	return 0;
}
