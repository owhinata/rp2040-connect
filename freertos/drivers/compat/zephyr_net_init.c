/* SPDX-License-Identifier: MIT */
/* The start of the layer's network part on FreeRTOS (zephyr_compat_os.h):
 * what Zephyr's net_init() (subsys/net/ip/net_core.c, a SYS_INIT() at boot)
 * does for the one offloaded interface of the build.  The driver defines its
 * device with NET_DEVICE_DT_OFFLOAD_DEFINE() (<zephyr/net/wifi_mgmt.h>: the
 * NINA-W102's Zephyr binding common/drivers/nina/port/zephyr/nina_net.c);
 * zephyr_net_initialize() starts the event thread (net_mgmt_event_init()),
 * registers the interface (<zephyr/net/net_if.h>, zephyr_net_if.c) with the
 * device's MTU, and calls the device's init and then the interface's, in
 * Zephyr's order.  The interface's requests go through net_mgmt()
 * (common/compat/zephyr_wifi_mgmt.c) to the driver's operations.  The NuttX
 * build does the same on its network device (common/drivers/nina/port/nuttx/
 * nina_netdev.c, nina_net_initialize()).  Part of the FreeRTOS
 * implementation of the small Zephyr compatibility layer of the port
 * (freertos/drivers/compat).
 */

#include <zephyr/kernel.h>
#include <zephyr/net/net_if.h>
#include <zephyr/net/net_mgmt.h>
#include <zephyr/net/wifi_mgmt.h>

#include "zephyr_compat_os.h"

static struct net_if offload_iface;

int zephyr_net_initialize(void)
{
	const struct net_wifi_mgmt_offload *api = z_net_offload_api;
	int ret;

	/* Zephyr's net_init(): the event thread, then the interfaces. */
	ret = net_mgmt_event_init();
	if (ret < 0) {
		return ret;
	}

	offload_iface.api = api;
	z_net_if_register(&offload_iface);
	net_if_set_mtu(&offload_iface, z_net_offload_mtu);

	ret = z_net_offload_init(NULL);
	if (ret < 0) {
		return ret;
	}

	api->wifi_iface.iface_api.init(&offload_iface);
	return 0;
}
