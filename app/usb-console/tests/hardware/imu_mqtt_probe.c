/* SPDX-License-Identifier: Apache-2.0 */
/* Diagnostic protocol: sequence:u32le, words:u16le, bitmap bytes:u16le,
 * then one bit per word (1 = accelerometer) and 6 raw little-endian bytes
 * per word. MQTT QoS 0; this is a test format, not a stable wire protocol.
 */
#include <errno.h>
#include <stddef.h>
#include <stdlib.h>
#include <string.h>
#include <zephyr/kernel.h>
#include <zephyr/net/mqtt.h>
#include <zephyr/net/socket.h>
#include <zephyr/shell/shell.h>
#include <zephyr/sys/atomic.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/sys/crc.h>
#include "../../drivers/lsm6dso/lsm6dso_fifo.h"
#include "../../drivers/lsm6dso/lsm6dso_fifo_decoder.h"
#include "imu_mqtt_probe.h"

#define PROBE_FRAME_WORDS 160
#define PROBE_MAX_PAYLOAD (8 + (PROBE_FRAME_WORDS + 7) / 8 + 6 * PROBE_FRAME_WORDS)
struct probe_item { uint16_t length; uint8_t payload[PROBE_MAX_PAYLOAD]; };
K_MSGQ_DEFINE(probe_queue, sizeof(struct probe_item), 64, 4);
K_SEM_DEFINE(probe_start, 0, 1);
static struct probe_item producer_item, consumer_item;
static uint8_t publish_buffer[2872];
static atomic_t session, enabled, connected, stopping, dropped, submitted, sent, failure;
static atomic_t publishes, publish_bytes, publish_us;
static struct sockaddr_in broker;
static struct mqtt_client client;
static uint8_t rx_buf[512], tx_buf[512];
static uint32_t sequence;
static uint32_t submitted_crc;
static const char topic[] = "diagnostic/imu/raw";

int imu_mqtt_probe_submit(const uint8_t *buffer, size_t length)
{
	const struct lsm6dso_fifo_packet *packet = (const void *)buffer;
	struct probe_item *item = &producer_item;
	size_t n, bitmap;

	if (atomic_get(&failure)) {
		return -EIO;
	}
	if (!atomic_get(&enabled)) {
		return 0;
	}
	if (!buffer || length < offsetof(struct lsm6dso_fifo_packet, data) ||
	    packet->marker != LSM6DSO_FIFO_PACKET_MARKER ||
	    packet->words > LSM6DSO_FIFO_MAX_WORDS ||
	    length - offsetof(struct lsm6dso_fifo_packet, data) <
			(size_t)packet->words * LSM6DSO_FIFO_WORD_SIZE) {
		return -EINVAL;
	}
	for (size_t start = 0; start < packet->words; start += n) {
		n = MIN((size_t)packet->words - start, PROBE_FRAME_WORDS);
		bitmap = (n + 7) / 8;
		item->length = 8 + bitmap + n * 6;
		sys_put_le32(sequence, item->payload);
		sys_put_le16(n, item->payload + 4);
		sys_put_le16(bitmap, item->payload + 6);
		memset(item->payload + 8, 0, bitmap);
		for (size_t i = 0; i < n; ++i) {
			const uint8_t *word = packet->data + (start + i) * LSM6DSO_FIFO_WORD_SIZE;
			uint8_t tag = word[0] >> 3;

			if (tag != LSM6DSO_FIFO_ACCEL && tag != LSM6DSO_FIFO_GYRO) {
				return -EPROTO;
			}
			if (tag == LSM6DSO_FIFO_ACCEL) {
				item->payload[8 + i / 8] |= BIT(i % 8);
			}
			memcpy(item->payload + 8 + bitmap + 6 * i, word + 1, 6);
		}
		if (k_msgq_put(&probe_queue, item, K_NO_WAIT)) {
			atomic_inc(&dropped);
			atomic_set(&failure, -ENOBUFS);
			return -ENOBUFS;
		}
		submitted_crc = crc32_ieee_update(submitted_crc, item->payload, item->length);
		sequence++;
		atomic_inc(&submitted);
	}
	return 0;
}

static void mqtt_event(struct mqtt_client *c, const struct mqtt_evt *evt)
{
	ARG_UNUSED(c);
	if (evt->type == MQTT_EVT_CONNACK) {
		atomic_set(&connected, evt->result == 0 ? 1 : 0);
		if (evt->result) {
			atomic_set(&failure, evt->result);
		}
	} else if (evt->type == MQTT_EVT_DISCONNECT) {
		atomic_clear(&connected);
	}
}

static void probe_thread(void *a, void *b, void *c)
{
	ARG_UNUSED(a); ARG_UNUSED(b); ARG_UNUSED(c);
	for (;;) {
		struct zsock_pollfd fd;
		bool pending = false;
		size_t pending_offset = 0;
		int rc;

		k_sem_take(&probe_start, K_FOREVER);
		mqtt_client_init(&client);
		client.broker = (struct sockaddr *)&broker;
		client.evt_cb = mqtt_event;
		client.client_id.utf8 = (uint8_t *)"rp2040-imu-probe";
		client.client_id.size = strlen("rp2040-imu-probe");
		client.protocol_version = MQTT_VERSION_3_1_1;
		client.transport.type = MQTT_TRANSPORT_NON_SECURE;
		client.rx_buf = rx_buf; client.rx_buf_size = sizeof(rx_buf);
		client.tx_buf = tx_buf; client.tx_buf_size = sizeof(tx_buf);
		rc = mqtt_connect(&client);
		if (!rc) {
			fd.fd = client.transport.tcp.sock; fd.events = ZSOCK_POLLIN;
			rc = zsock_poll(&fd, 1, 5000) > 0 ? mqtt_input(&client) : -ETIMEDOUT;
		}
		if (!rc && !atomic_get(&connected)) {
			rc = -ECONNREFUSED;
		}
		if (rc) {
			atomic_set(&failure, rc);
			(void)mqtt_abort(&client);
			atomic_clear(&session);
			continue;
		}
		atomic_set(&enabled, 1);
		while (!atomic_get(&failure) &&
		       (!atomic_get(&stopping) || k_msgq_num_used_get(&probe_queue) || pending)) {
			struct mqtt_publish_param p = {0};
			size_t bytes = 0;
			unsigned int frames = 0;

			if (!pending && k_msgq_get(&probe_queue, &consumer_item, K_MSEC(100))) {
				continue;
			}
			pending = true;
			while (pending) {
				size_t n = MIN((size_t)consumer_item.length - pending_offset,
					       sizeof(publish_buffer) - bytes);

				memcpy(publish_buffer + bytes, consumer_item.payload + pending_offset, n);
				bytes += n;
				pending_offset += n;
				if (pending_offset == consumer_item.length) {
					frames++;
					pending_offset = 0;
					pending = false;
				}
				if (bytes == sizeof(publish_buffer)) {
					break;
				}
				if (!pending) {
					pending = k_msgq_get(&probe_queue, &consumer_item, K_MSEC(20)) == 0;
				}
			}
			p.message.topic.topic.utf8 = (uint8_t *)topic;
			p.message.topic.topic.size = sizeof(topic) - 1;
			p.message.topic.qos = MQTT_QOS_0_AT_MOST_ONCE;
			p.message.payload.data = publish_buffer;
			p.message.payload.len = bytes;
			uint32_t started = k_cycle_get_32();
			rc = mqtt_publish(&client, &p);
			atomic_inc(&publishes);
			atomic_add(&publish_bytes, bytes);
			atomic_add(&publish_us, k_cyc_to_us_floor32(k_cycle_get_32() - started));
			if (rc) {
				atomic_set(&failure, rc);
				break;
			}
			atomic_add(&sent, frames);
		}
		atomic_clear(&enabled);
		struct mqtt_disconnect_param disconnect = {0};
		(void)mqtt_disconnect(&client, &disconnect);
		atomic_clear(&connected);
		atomic_clear(&session);
	}
}
K_THREAD_DEFINE(imu_mqtt_probe_tid, 3072, probe_thread, NULL, NULL, NULL,
		CONFIG_IMU_MQTT_TEST_PRIORITY, 0, 0);

static int probe_cmd(const struct shell *sh, size_t argc, char **argv)
{
	if (argc == 2 && !strcmp(argv[1], "stop")) {
		atomic_set(&stopping, 1);
	} else if (argc == 4 && !strcmp(argv[1], "start")) {
		char *end;
		unsigned long port = strtoul(argv[3], &end, 10);

		if (atomic_get(&session) || *end || !port || port > 65535 ||
		    zsock_inet_pton(AF_INET, argv[2], &broker.sin_addr) != 1) {
			return -EINVAL;
		}
		broker.sin_family = AF_INET; broker.sin_port = htons(port);
		k_msgq_purge(&probe_queue);
		sequence = 0;
		submitted_crc = 0;
		atomic_clear(&stopping); atomic_clear(&dropped); atomic_clear(&submitted);
		atomic_clear(&sent); atomic_clear(&failure);
		atomic_clear(&publishes); atomic_clear(&publish_bytes); atomic_clear(&publish_us);
		atomic_set(&session, 1);
		k_sem_give(&probe_start);
	} else if (argc != 2 || strcmp(argv[1], "status")) {
		shell_error(sh, "Use imu_mqtt_probe start <IPv4> <port>|status|stop");
		return -EINVAL;
	}
	shell_print(sh, "connected=%d enabled=%d submitted=%d sent=%d dropped=%d queued=%u error=%d",
		    (int)atomic_get(&connected), (int)atomic_get(&enabled), (int)atomic_get(&submitted),
		    (int)atomic_get(&sent), (int)atomic_get(&dropped), k_msgq_num_used_get(&probe_queue),
		    (int)atomic_get(&failure));
	shell_print(sh, "publishes=%d bytes=%d publish_us=%d", (int)atomic_get(&publishes),
		    (int)atomic_get(&publish_bytes), (int)atomic_get(&publish_us));
	shell_print(sh, "submitted_crc32=%08x", submitted_crc);
	return 0;
}
SHELL_CMD_REGISTER(imu_mqtt_probe, NULL, "Experimental MQTT IMU delivery probe", probe_cmd);
