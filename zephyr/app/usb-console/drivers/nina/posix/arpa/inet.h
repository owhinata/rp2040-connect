/* SPDX-License-Identifier: MIT */
/* <arpa/inet.h> for the shared NINA core on Zephyr (see sys/socket.h
 * here): inet_pton() is zsock_inet_pton(); htons() comes with the
 * compatibility mode (net_htons()).  Found before the C library's
 * arpa/inet.h, which declares functions this build does not have.
 */

#ifndef NINA_POSIX_ARPA_INET_H_
#define NINA_POSIX_ARPA_INET_H_

#include <sys/socket.h>

#define inet_pton zsock_inet_pton

#endif /* NINA_POSIX_ARPA_INET_H_ */
