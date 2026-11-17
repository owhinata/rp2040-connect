/* SPDX-License-Identifier: MIT */
/* The part of POSIX's <netinet/in.h> that the FreeRTOS build needs so far,
 * which newlib does not have: the IPv4 address and socket address types
 * (struct in_addr of common/imu/imu_raw_mqtt.h, through
 * <zephyr/net/net_ip.h>; struct sockaddr_in of the MQTT transport's stub,
 * freertos/drivers/imu/imu_raw_mqtt_null.c).  Addresses and ports are in
 * network byte order, as in POSIX and Zephyr.  The byte order functions are
 * <arpa/inet.h>'s.  Part of newlib's glue of the FreeRTOS build
 * (freertos/drivers/libc).
 */

#ifndef FREERTOS_DRIVERS_LIBC_NETINET_IN_H
#define FREERTOS_DRIVERS_LIBC_NETINET_IN_H

#include <stdint.h>
#include <sys/socket.h>

typedef uint16_t in_port_t;
typedef uint32_t in_addr_t;

struct in_addr {
	in_addr_t s_addr;
};

struct sockaddr_in {
	sa_family_t sin_family;
	in_port_t sin_port;
	struct in_addr sin_addr;
	uint8_t sin_zero[8];
};

#define INADDR_ANY      ((in_addr_t)0x00000000)
#define INET_ADDRSTRLEN 16

#endif /* FREERTOS_DRIVERS_LIBC_NETINET_IN_H */
