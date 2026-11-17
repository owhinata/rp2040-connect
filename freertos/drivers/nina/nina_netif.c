/* SPDX-License-Identifier: MIT */
/* The NINA-W102's network interface on the FreeRTOS build.  The Zephyr build
 * defines it with NET_DEVICE_DT_OFFLOAD_DEFINE() (common/drivers/nina/
 * nina_net.c); Zephyr's net_init() starts the event thread and then calls
 * the device's init (driver_init()) and the interface's (iface_init():
 * dormant, MAC address from the module, the resolver).  Here
 * nina_net_initialize() does the same in the same order on the layer's one
 * interface (<zephyr/net/net_if.h>, zephyr_net_if.c), as the NuttX build's
 * nuttx/drivers/nina/nina_netdev.c does on its network device.  The
 * interface's requests go through net_mgmt() (common/compat/
 * zephyr_wifi_mgmt.c) to the driver's operations in nina_net.c.
 */

#include <zephyr/kernel.h>
#include <zephyr/net/net_if.h>
#include <zephyr/net/net_mgmt.h>
#include <zephyr/net/wifi_mgmt.h>

#include "nina_net.h"

static struct net_if nina_iface;

int nina_net_initialize(void)
{
	const struct net_wifi_mgmt_offload *api = z_net_offload_api;
	int ret;

	/* Zephyr's net_init(): the event thread, then the interfaces. */
	ret = net_mgmt_event_init();
	if (ret < 0) {
		return ret;
	}

	nina_iface.api = api;
	z_net_if_register(&nina_iface);

	ret = z_net_offload_init(NULL);
	if (ret < 0) {
		return ret;
	}

	api->wifi_iface.iface_api.init(&nina_iface);
	return 0;
}
