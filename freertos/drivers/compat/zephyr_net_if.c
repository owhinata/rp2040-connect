/* SPDX-License-Identifier: MIT */
/* <zephyr/net/net_if.h> of the Zephyr compatibility layer on FreeRTOS: the
 * state of the one network interface (the NINA-W102's), under a spinlock
 * (a few words each time).  See the header.  The NuttX build keeps the
 * same state in its network device (nuttx/drivers/compat/zephyr_net_if.c).
 */

#include <errno.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include <zephyr/kernel.h>
#include <zephyr/net/net_if.h>
#include <zephyr/spinlock.h>

static struct k_spinlock lock;
static struct net_if *default_iface;
static bool admin_up;
static bool dormant = true;

/* What net_if_ipv4_addr_add() returns and net_if_ipv4_get_global_addr()
 * points to (Zephyr: the interface's address slot).  0.0.0.0 when none.
 */
static struct net_if_addr unicast;
static struct net_in_addr netmask;
static struct net_in_addr gateway;
static uint8_t link_addr[6];

void z_net_if_register(struct net_if *iface)
{
	k_spinlock_key_t key = k_spin_lock(&lock);

	default_iface = iface;
	admin_up = true;
	dormant = true;
	k_spin_unlock(&lock, key);
}

struct net_if *net_if_get_default(void)
{
	return default_iface;
}

struct net_if *net_if_get_wifi_sta(void)
{
	return default_iface;
}

struct net_if_addr *net_if_ipv4_addr_add(struct net_if *iface, const struct net_in_addr *addr,
					 enum net_addr_type addr_type, uint32_t vlifetime)
{
	k_spinlock_key_t key;

	(void)iface;
	(void)vlifetime;
	if (addr->s_addr == INADDR_ANY) {
		return NULL;
	}

	key = k_spin_lock(&lock);
	unicast.address = *addr;
	unicast.addr_type = addr_type;
	unicast.addr_state = NET_ADDR_PREFERRED;
	k_spin_unlock(&lock, key);
	return &unicast;
}

bool net_if_ipv4_addr_rm(struct net_if *iface, const struct net_in_addr *addr)
{
	k_spinlock_key_t key;
	bool removed = false;

	(void)iface;
	key = k_spin_lock(&lock);
	if (addr->s_addr != INADDR_ANY && unicast.address.s_addr == addr->s_addr) {
		unicast.address.s_addr = INADDR_ANY;
		netmask.s_addr = INADDR_ANY;
		removed = true;
	}
	k_spin_unlock(&lock, key);
	return removed;
}

struct net_in_addr *net_if_ipv4_get_global_addr(struct net_if *iface, enum net_addr_state state)
{
	k_spinlock_key_t key;
	bool found;

	if (iface == NULL) {
		return NULL;
	}

	key = k_spin_lock(&lock);
	found = unicast.address.s_addr != INADDR_ANY &&
		(state == NET_ADDR_ANY_STATE || state == unicast.addr_state);
	k_spin_unlock(&lock, key);
	return found ? &unicast.address : NULL;
}

bool net_if_ipv4_set_netmask_by_addr(struct net_if *iface, const struct net_in_addr *addr,
				     const struct net_in_addr *mask)
{
	k_spinlock_key_t key;
	bool set = false;

	(void)iface;
	key = k_spin_lock(&lock);
	if (unicast.address.s_addr != INADDR_ANY && unicast.address.s_addr == addr->s_addr) {
		netmask = *mask;
		set = true;
	}
	k_spin_unlock(&lock, key);
	return set;
}

void net_if_ipv4_set_gw(struct net_if *iface, const struct net_in_addr *gw)
{
	k_spinlock_key_t key = k_spin_lock(&lock);

	(void)iface;
	gateway = *gw;
	k_spin_unlock(&lock, key);
}

struct net_in_addr net_if_ipv4_get_gw(struct net_if *iface)
{
	k_spinlock_key_t key = k_spin_lock(&lock);
	struct net_in_addr gw = gateway;

	(void)iface;
	k_spin_unlock(&lock, key);
	return gw;
}

void net_if_dormant_on(struct net_if *iface)
{
	(void)iface;
	dormant = true;
}

void net_if_dormant_off(struct net_if *iface)
{
	(void)iface;
	dormant = false;
}

bool net_if_is_dormant(struct net_if *iface)
{
	(void)iface;
	return dormant;
}

bool net_if_is_admin_up(struct net_if *iface)
{
	(void)iface;
	return admin_up;
}

int net_if_set_link_addr(struct net_if *iface, uint8_t *addr, uint8_t len, enum net_link_type type)
{
	k_spinlock_key_t key;

	(void)iface;
	if (type != NET_LINK_ETHERNET || len != sizeof(link_addr)) {
		return -EINVAL;
	}

	key = k_spin_lock(&lock);
	memcpy(link_addr, addr, len);
	k_spin_unlock(&lock, key);
	return 0;
}
