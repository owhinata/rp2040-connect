/* SPDX-License-Identifier: Apache-2.0 */
#include <errno.h>
#include <stdlib.h>
#include <string.h>
#include <zephyr/net/mqtt.h>
#include <zephyr/net/socket.h>
#include <zephyr/shell/shell.h>

static bool connack;
static void on_event(struct mqtt_client *client, const struct mqtt_evt *evt)
{
	ARG_UNUSED(client);
	if (evt->type == MQTT_EVT_CONNACK && evt->result == 0) {
		connack = true;
	}
}

static int mqtt_test(const struct shell *sh, size_t argc, char **argv)
{
	struct mqtt_client client;
	struct net_sockaddr_storage addr = {0};
	struct net_sockaddr_in *ip = (struct net_sockaddr_in *)&addr;
	struct mqtt_publish_param pub = {0};
	struct zsock_pollfd pfd;
	uint8_t rx[128], tx[128];
	char *end;
	unsigned long port = strtoul(argv[2], &end, 10);
	int ret;

	ARG_UNUSED(argc);
	if (*end || !port || port > 65535 ||
	    zsock_inet_pton(NET_AF_INET, argv[1], &ip->sin_addr) != 1) {
		return -EINVAL;
	}

	ip->sin_family = NET_AF_INET;
	ip->sin_port = net_htons(port);

	mqtt_client_init(&client);
	client.broker = &addr;
	client.evt_cb = on_event;
	client.client_id.utf8 = (uint8_t *)"rp2040-nina-test";
	client.client_id.size = 16;
	client.protocol_version = MQTT_VERSION_3_1_1;
	client.rx_buf = rx;
	client.rx_buf_size = sizeof(rx);
	client.tx_buf = tx;
	client.tx_buf_size = sizeof(tx);
	client.transport.type = MQTT_TRANSPORT_NON_SECURE;

	connack = false;
	ret = mqtt_connect(&client);
	if (ret) {
		shell_error(sh, "mqtt_connect: %d", ret);
		return ret;
	}

	pfd = (struct zsock_pollfd){.fd = client.transport.tcp.sock,
				    .events = ZSOCK_POLLIN};
	ret = zsock_poll(&pfd, 1, 5000);
	if (ret > 0) {
		ret = mqtt_input(&client);
	} else {
		ret = ret == 0 ? -ETIMEDOUT : -errno;
	}
	if (ret || !connack) {
		shell_error(sh, "MQTT CONNACK failed: %d", ret);
		mqtt_abort(&client);
		return ret ? ret : -ECONNREFUSED;
	}

	pub.message.topic.topic.utf8 = (uint8_t *)"rp2040/test";
	pub.message.topic.topic.size = strlen("rp2040/test");
	pub.message.topic.qos = MQTT_QOS_0_AT_MOST_ONCE;
	pub.message.payload.data = (uint8_t *)"nina mqtt test";
	pub.message.payload.len = strlen("nina mqtt test");
	ret = mqtt_publish(&client, &pub);
	if (ret) {
		shell_error(sh, "mqtt_publish: %d", ret);
	} else {
		shell_print(sh, "MQTT CONNACK and QoS 0 publish succeeded");
	}
	mqtt_disconnect(&client, NULL);
	return ret;
}
SHELL_CMD_ARG_REGISTER(nina_mqtt_test, NULL,
		       "Test MQTT: nina_mqtt_test <broker-ip> <port>",
		       mqtt_test, 3, 0);
